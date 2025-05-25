/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

#include "lua_scripts.h"
#include <AP_HAL/AP_HAL.h>
#include "AP_Scripting.h"
#include <AP_Logger/AP_Logger.h>
#if AP_SCRIPTING_ENCRYPTION_ENABLED
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_CheckFirmware/monocypher.h>
#endif

#include <AP_Scripting/lua_generated_bindings.h>

#define DISABLE_INTERRUPTS_FOR_SCRIPT_RUN 0

extern const AP_HAL::HAL& hal;
#define ENABLE_DEBUG_MODULE 0

bool lua_scripts::overtime;
jmp_buf lua_scripts::panic_jmp;
char *lua_scripts::error_msg_buf;
HAL_Semaphore lua_scripts::error_msg_buf_sem;
uint8_t lua_scripts::print_error_count;
uint32_t lua_scripts::last_print_ms;

uint32_t lua_scripts::loaded_checksum;
uint32_t lua_scripts::running_checksum;
HAL_Semaphore lua_scripts::crc_sem;

// return string error message for error object at top of stack
static const char *get_error_object_message(lua_State *L) {
    const char *m = lua_tostring(L, -1);
    if (!m) { // error object is not stringifiable
        return "<error>";
    }
    return m;
}

lua_scripts::lua_scripts(const AP_Int32 &vm_steps, const AP_Int32 &heap_size, AP_Int8 &debug_options)
    : _vm_steps(vm_steps),
      _debug_options(debug_options)
{
    const bool allow_heap_expansion = !option_is_set(AP_Scripting::DebugOption::DISABLE_HEAP_EXPANSION);
    _heap.create(heap_size, 10, allow_heap_expansion, 20*1024);
}

lua_scripts::~lua_scripts() {
    _heap.destroy();
}

void lua_scripts::hook(lua_State *L, lua_Debug *ar) {
    lua_scripts::overtime = true;

    // we need to aggressively bail out as we are over time
    // so we will aggressively trap errors until we clear out
    lua_sethook(L, hook, LUA_MASKCOUNT, 1);

    luaL_error(L, "Exceeded CPU time");
}

void lua_scripts::print_error(MAV_SEVERITY severity) {
    error_msg_buf_sem.take_blocking();
    if (error_msg_buf == nullptr) {
        error_msg_buf_sem.give();
        return;
    }
    last_print_ms = AP_HAL::millis();
    GCS_SEND_TEXT(severity, "Lua: %s", error_msg_buf);
    error_msg_buf_sem.give();
}

void lua_scripts::set_and_print_new_error_message(MAV_SEVERITY severity, const char *fmt, ...) {
    error_msg_buf_sem.take_blocking();

    // reset buffer and print count
    print_error_count = 0;
    if (error_msg_buf) {
        _heap.deallocate(error_msg_buf);
        error_msg_buf = nullptr;
    }

    // generate va_list and create a copy
    va_list arg_list, arg_list_copy;
    va_start(arg_list, fmt);
    va_copy(arg_list_copy, arg_list);

    // dry run to work out the required length
    int len = hal.util->vsnprintf(nullptr, 0, fmt, arg_list_copy);

    // finished with copy
    va_end(arg_list_copy);

    if (len <= 0) {
        // nothing to print, something has gone wrong
        va_end(arg_list);
        error_msg_buf_sem.give();
        return;
    }

    // allocate buffer on scripting heap
    error_msg_buf = (char *)_heap.allocate(len+1);
    if (!error_msg_buf) {
        // allocation failed
        va_end(arg_list);
        error_msg_buf_sem.give();
        return;
    }

    // do actual print to buffer and clear va list
    hal.util->vsnprintf(error_msg_buf, len+1, fmt, arg_list);
    va_end(arg_list);

    // print to cosole and GCS
    DEV_PRINTF("Lua: %s\n", error_msg_buf);

    error_msg_buf_sem.give();
    print_error(severity);
}

int lua_scripts::atpanic(lua_State *L) {
    set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "Panic: %s", get_error_object_message(L));
    longjmp(panic_jmp, 1);
    return 0;
}

// helper for print and log of runtime stats
void lua_scripts::update_stats(const char *name, uint32_t run_time, int total_mem, int run_mem)
{
    if (option_is_set(AP_Scripting::DebugOption::RUNTIME_MSG)) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Lua: Time: %u Mem: %d + %d",
                                            (unsigned int)run_time,
                                            (int)total_mem,
                                            (int)run_mem);
    }
#if HAL_LOGGING_ENABLED
    if (option_is_set(AP_Scripting::DebugOption::LOG_RUNTIME)) {
        struct log_Scripting pkt {
            LOG_PACKET_HEADER_INIT(LOG_SCRIPTING_MSG),
            time_us      : AP_HAL::micros64(),
            name         : {},
            run_time     : run_time,
            total_mem    : total_mem,
            run_mem      : run_mem
        };
        const char * name_short = strrchr(name, '/');
        if ((strlen(name) > sizeof(pkt.name)) && (name_short != nullptr)) {
            strncpy_noterm(pkt.name, name_short+1, sizeof(pkt.name));
        } else {
            strncpy_noterm(pkt.name, name, sizeof(pkt.name));
        }
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
#endif // HAL_LOGGING_ENABLED
}

void lua_scripts::load_script_error(lua_State *L, char *filename, int error)
{
    switch (error) {
        case LUA_ERRSYNTAX:
            set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "Error: %s", get_error_object_message(L));
            lua_pop(L, lua_gettop(L));
            break;
        case LUA_ERRMEM:
            set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "Insufficent memory loading %s", filename);
            lua_pop(L, lua_gettop(L));
            break;
        case LUA_ERRFILE:
            set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "Unable to load the file %s: %s", filename, get_error_object_message(L));
            lua_pop(L, lua_gettop(L));
            break;
        default:
            set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "Unknown error (%d) loading %s", error, filename);
            lua_pop(L, lua_gettop(L));
            break;
    }
}

lua_scripts::script_info *lua_scripts::load_script(lua_State *L, char *filename) {
    int error = 0;
#if AP_SCRIPTING_ENCRYPTION_ENABLED
    if (!strncmp(&filename[strlen(filename)-4], ".lxa", 4)) {    // encrypted script
        int fd = AP::FS().open(filename, O_RDONLY);

        if (fd < 0) {
            load_script_error(L, filename, LUA_ERRFILE);
            return nullptr;
        }

        // Get the size of the file
        AP_Filesystem::stat_t st;
        if (!AP::FS().stat(filename, st)) {
            AP::FS().close(fd);
            load_script_error(L, filename, LUA_ERRFILE);
            return nullptr;
        }

        off_t filesize = st.size - 46;

        // Allocate buffer
        char *buffer = (char *)malloc(filesize);
        if (buffer == nullptr) {
            AP::FS().close(fd);
            load_script_error(L, filename, LUA_ERRMEM);
            return nullptr;
        }

        char header[6];
        if (AP::FS().read(fd, header, 6) != 6 || strncmp(header, "LUA1.0", 6) != 0) {
            free(buffer);
            AP::FS().close(fd);
            load_script_error(L, filename, LUA_ERRFILE);
            return nullptr;
        }

        uint8_t mac[16] = {};
        if (AP::FS().read(fd, mac, 16) != 16) {
            free(buffer);
            AP::FS().close(fd);
            load_script_error(L, filename, LUA_ERRFILE);
            return nullptr;
        }

        uint8_t nonce[24] = {};
        if (AP::FS().read(fd, nonce, 24) != 24) {
            free(buffer);
            AP::FS().close(fd);
            load_script_error(L, filename, LUA_ERRFILE);
            return nullptr;
        }

        // Read into buffer
        ssize_t total_read = 0;
        while (total_read < filesize) {
            ssize_t bytes_read = AP::FS().read(fd, buffer + total_read, filesize - total_read);
            if (bytes_read < 0 ) {
                free(buffer);
                AP::FS().close(fd);
                load_script_error(L, filename, LUA_ERRFILE);
                return nullptr;
            }
            if (bytes_read == 0) {
                break; // EOF
            }
            total_read += bytes_read;
        }
        AP::FS().close(fd);
#if AP_SCRIPTING_ENCRYPTION_UUID_ENABLED
        create_nonce(nonce, filename);
#endif
        if (!decrypt_script(buffer, mac, nonce, filesize)) {
            load_script_error(L, filename, LUA_ERRFILE);
            return nullptr;
        }

        error = luaL_loadbuffer(L, buffer, filesize, buffer);

        free(buffer);
    } else
#endif
    {
        error = luaL_loadfile(L, filename);
    }
    if (error != 0) {
        load_script_error(L, filename, error);
        return nullptr;
    }

    const int loadMem = lua_gc(L, LUA_GCCOUNT, 0) * 1024 + lua_gc(L, LUA_GCCOUNTB, 0);
    const uint32_t loadStart = AP_HAL::micros();

    script_info *new_script = (script_info *)_heap.allocate(sizeof(script_info));
    if (new_script == nullptr) {
        // No memory, shouldn't happen, we even attempted to do a GC
        set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "Insufficent memory loading %s", filename);
        lua_pop(L, 1); // we can't use the function we just loaded, so ditch it
        return nullptr;
    }


    create_sandbox(L);
    lua_pushvalue(L, -1); // duplicate environment for reference below
    lua_setupvalue(L, -3, 1);

    const uint32_t loadEnd = AP_HAL::micros();
    const int endMem = lua_gc(L, LUA_GCCOUNT, 0) * 1024 + lua_gc(L, LUA_GCCOUNTB, 0);

    update_stats(filename, loadEnd-loadStart, endMem, loadMem);

    new_script->name = filename;
    new_script->env_ref = luaL_ref(L, LUA_REGISTRYINDEX); // store reference to script's environment
    new_script->run_ref = luaL_ref(L, LUA_REGISTRYINDEX); // store reference to function to run
    new_script->next_run_ms = AP_HAL::millis64() - 1; // force the script to be stale

    // Get checksum of file
    uint32_t crc = 0;
    if (AP::FS().crc32(filename, crc)) {
        // Record crc of this script
        new_script->crc = crc;
        {
            // Apply crc to checksum of all scripts
            WITH_SEMAPHORE(crc_sem);
            loaded_checksum ^= crc;
            running_checksum ^= crc;
        }
    }

    return new_script;
}

void lua_scripts::create_sandbox(lua_State *L) {
    lua_newtable(L);
    luaopen_base_sandbox(L);

#if ENABLE_DEBUG_MODULE
    lua_pushstring(L, "debug");
    luaopen_debug(L);
    lua_settable(L, -3);
#endif
    lua_pushstring(L, "math");
    luaopen_math(L);
    lua_settable(L, -3);
    lua_pushstring(L, "table");
    luaopen_table(L);
    lua_settable(L, -3);
    lua_pushstring(L, "string");
    luaopen_string(L);
    lua_settable(L, -3);
    lua_pushstring(L, "io");
    luaopen_io(L);
    lua_settable(L, -3);
    lua_pushstring(L, "utf8");
    luaopen_utf8(L);
    lua_settable(L, -3);
    lua_pushstring(L, "package");
    luaopen_package(L);
    lua_settable(L, -3);

    load_generated_sandbox(L);
}

void lua_scripts::load_all_scripts_in_dir(lua_State *L, const char *dirname) {
    if (dirname == nullptr) {
        return;
    }
    auto *d = AP::FS().opendir(dirname);
    if (d == nullptr) {
        // this disk_space check will return 0 if we don't have a real
        // filesystem (ie. no Posix or FatFs).  Do not warn in this case.
        if (AP::FS().disk_space(dirname) != 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Lua: open directory (%s) failed", dirname);
        }
        return;
    }

    // load anything that ends in .lua
    for (struct dirent *de=AP::FS().readdir(d); de; de=AP::FS().readdir(d)) {
        uint8_t length = strlen(de->d_name);
        if (length < 5) {
            // not long enough
            continue;
        }

        if ((de->d_name[0] == '.') || (strncmp(&de->d_name[length-4], ".lua", 4)
#if AP_SCRIPTING_ENCRYPTION_ENABLED
            && strncmp(&de->d_name[length-4], ".lxa", 4)
#endif
        )){
            // starts with . (hidden file) or doesn't end in .lua or .lxa
            continue;
        }

        // FIXME: because chunk name fetching is not working we are allocating and storing an extra string we shouldn't need to
        size_t size = strlen(dirname) + strlen(de->d_name) + 2;
        char * filename = (char *) _heap.allocate(size);
        if (filename == nullptr) {
            continue;
        }
        snprintf(filename, size, "%s/%s", dirname, de->d_name);

        // we have something that looks like a lua file, attempt to load it
        script_info * script = load_script(L, filename);
        if (script == nullptr) {
            _heap.deallocate(filename);
            continue;
        }

        reschedule_script(script);

#if HAL_LOGGER_FILE_CONTENTS_ENABLED
        if (!option_is_set(AP_Scripting::DebugOption::SUPPRESS_SCRIPT_LOG)) {
            AP::logger().log_file_content(filename);
        }
#endif
    }
    AP::FS().closedir(d);
}

#if AP_SCRIPTING_ENCRYPTION_ENABLED
void lua_scripts::encrypt_all_scripts_in_dir(const char *dirname)
{
    if (dirname == nullptr) {
        return;
    }

    auto *d = AP::FS().opendir(dirname);
    if (d == nullptr) {
        return;
    }

    // load anything that ends in .lua
    for (struct dirent *de=AP::FS().readdir(d); de; de=AP::FS().readdir(d)) {
        uint8_t length = strlen(de->d_name);
        if (length < 5) {
            // not long enough
            continue;
        }

        if ((de->d_name[0] == '.') || strncmp(&de->d_name[length-4], ".lua", 4)) {
            // starts with . (hidden file) or doesn't end in .lua
            continue;
        }

        size_t nmsize = strlen(dirname) + strlen(de->d_name) + 2;
        char * filename = (char *) malloc(nmsize);
        if (filename == nullptr) {
            return;
        }
        snprintf(filename, nmsize, "%s/%s", dirname, de->d_name);

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Lua: encrypting script %s", filename);

        int fd = AP::FS().open(filename, O_RDONLY);

        if (fd < 0) {
            continue;
        }

        // Get the size of the file
        AP_Filesystem::stat_t st;
        if (!AP::FS().stat(filename, st)) {
            AP::FS().close(fd);
            continue;
        }

        off_t filesize = st.size;

        // Allocate buffer
        char *buffer = (char *)malloc(filesize);
        if (buffer == nullptr) {
            AP::FS().close(fd);
            return;
        }

        // Read into buffer
        ssize_t total_read = 0;
        while (total_read < filesize) {
            ssize_t bytes_read = AP::FS().read(fd, buffer + total_read, filesize - total_read);
            if (bytes_read < 0 ) {
                free(buffer);
                AP::FS().close(fd);
                return;
            }
            if (bytes_read == 0) {
                break; // EOF
            }
            total_read += bytes_read;
        }
        AP::FS().close(fd);

        uint8_t mac[16];
        uint8_t nonce[24] = {};
        create_nonce(nonce, filename);
        if (!encrypt_script(buffer, mac, nonce, filesize)) {
            continue;
        }

        AP::FS().unlink(filename);

        strcpy(filename + (nmsize - 5), ".lxa");

        fd = AP::FS().open(filename, O_WRONLY | O_CREAT | O_TRUNC);

        if (fd < 0) {
            free(buffer);
            continue;
        }

        if (AP::FS().write(fd, "LUA1.0", 6) != 6) {
            free(buffer);
            AP::FS().close(fd);
            continue;
        }

        if (AP::FS().write(fd, mac, 16) != 16) {
            free(buffer);
            AP::FS().close(fd);
            continue;
        }

        if (AP::FS().write(fd, nonce, 24) != 24) {
            free(buffer);
            AP::FS().close(fd);
            continue;
        }

        if (AP::FS().write(fd, buffer, filesize) != filesize) {
            free(buffer);
            AP::FS().close(fd);
            continue;
        }
        AP::FS().close(fd);

        // Cleanup
        free(buffer);
    }
    AP::FS().closedir(d);
}
#endif

void lua_scripts::reset_loop_overtime(lua_State *L) {
    overtime = false;
    // reset the hook to clear the counter
    const int32_t vm_steps = MAX(_vm_steps, 1000);
    lua_sethook(L, hook, LUA_MASKCOUNT, vm_steps);
}

void lua_scripts::run_next_script(lua_State *L) {
    if (scripts == nullptr) {
#if defined(AP_SCRIPTING_CHECKS) && AP_SCRIPTING_CHECKS >= 1
        AP_HAL::panic("Lua: Attempted to run a script without any scripts queued");
#endif // defined(AP_SCRIPTING_CHECKS) && AP_SCRIPTING_CHECKS >= 1
        return;
    }

    uint64_t start_time_ms = AP_HAL::millis64();
    // strip the selected script out of the list
    script_info *script = scripts;
    scripts = script->next;

    // reset the hook to clear the counter
    reset_loop_overtime(L);

    // store top of stack so we can calculate the number of return values
    int stack_top = lua_gettop(L);

    // pop the function to the top of the stack
    lua_rawgeti(L, LUA_REGISTRYINDEX, script->run_ref);
    // set current environment for other users
    AP::scripting()->set_current_env_ref(script->env_ref);

    if(lua_pcall(L, 0, LUA_MULTRET, 0)) {
        if (overtime) {
            // script has consumed an excessive amount of CPU time
            set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "%s exceeded time limit", script->name);
        } else {
            set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "%s", get_error_object_message(L));
        }
        remove_script(L, script);
        lua_pop(L, 1);
        return;
    } else {
        int returned = lua_gettop(L) - stack_top;
        switch (returned) {
            case 0:
                // no time to reschedule so bail out
                remove_script(L, script);
                break;
            case 2:
                {
                    // sanity check the return types
                    if (lua_type(L, -1) != LUA_TNUMBER) {
                        set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "%s did not return a delay (0x%d)", script->name, lua_type(L, -1));
                        lua_pop(L, 2);
                        remove_script(L, script);
                        return;
                    }
                    if (lua_type(L, -2) != LUA_TFUNCTION) {
                        set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "%s did not return a function (0x%d)", script->name, lua_type(L, -2));
                        lua_pop(L, 2);
                        remove_script(L, script);
                        return;
                    }

                    // types match the expectations, go ahead and reschedule
                    script->next_run_ms = start_time_ms + (uint64_t)luaL_checknumber(L, -1);
                    lua_pop(L, 1);
                    int old_ref = script->run_ref;
                    script->run_ref = luaL_ref(L, LUA_REGISTRYINDEX);
                    luaL_unref(L, LUA_REGISTRYINDEX, old_ref);
                    reschedule_script(script);
                    break;
                }
            default:
                {
                    set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "%s returned bad result count (%d)", script->name, returned);
                    remove_script(L, script);
                    // pop all the results we got that we didn't expect
                    lua_pop(L, returned);
                    break;
                 }
         }
     }
}

void lua_scripts::remove_script(lua_State *L, script_info *script) {
    if (script == nullptr) {
        return;
    }

    // ensure that the script isn't in the loaded list for any reason
    if (scripts == nullptr) {
        // nothing to do, already not in the list
    } else if (scripts == script) {
        scripts = script->next;
    } else {
        for(script_info * current = scripts; current->next != nullptr; current = current->next) {
            if (current->next == script) {
                current->next = script->next;
                break;
            }
        }
    }

    {
        // Remove from running checksum
        WITH_SEMAPHORE(crc_sem);
        running_checksum ^= script->crc;
    }
    
    if (L != nullptr) {
        // state could be null if we are force killing all scripts
        luaL_unref(L, LUA_REGISTRYINDEX, script->env_ref);
        luaL_unref(L, LUA_REGISTRYINDEX, script->run_ref);
    }
    _heap.deallocate(script->name);
    _heap.deallocate(script);
}

void lua_scripts::reschedule_script(script_info *script) {
    if (script == nullptr) {
#if defined(AP_SCRIPTING_CHECKS) && AP_SCRIPTING_CHECKS >= 1
       AP_HAL::panic("Lua: Attempted to schedule a null pointer");
#endif // defined(AP_SCRIPTING_CHECKS) && AP_SCRIPTING_CHECKS >= 1
       return;
    }

    script->next = nullptr;
    if (scripts == nullptr) {
        scripts = script;
        return;
    }

    uint64_t next_run_ms = script->next_run_ms;

    if (scripts->next_run_ms > next_run_ms) {
        script->next = scripts;
        scripts = script;
        return;
    }

    script_info *previous = scripts;
    while (previous->next != nullptr) {
        if (previous->next->next_run_ms > next_run_ms) {
            script->next = previous->next;
            previous->next = script;
            return;
        }
        previous = previous->next;
    }

    previous->next = script;
}

MultiHeap lua_scripts::_heap;

void *lua_scripts::alloc(void *ud, void *ptr, size_t osize, size_t nsize) {
    (void)ud; /* not used */
    return _heap.change_size(ptr, osize, nsize);
}

void lua_scripts::run(void) {
    bool succeeded_initial_load = false;

    if (!_heap.available()) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Lua: Unable to allocate a heap");
        return;
    }

    // panic should be hooked first
    if (setjmp(panic_jmp)) {
        if (!succeeded_initial_load) {
            return;
        }
        if (lua_state != nullptr) {
            lua_close(lua_state); // shutdown the old state
        }
        // remove all the old scheduled scripts
        for (script_info *script = scripts; script != nullptr; script = scripts) {
            remove_script(nullptr, script);
        }
        scripts = nullptr;
        overtime = false;
    }

    lua_state = lua_newstate(alloc, NULL);
    lua_State *L = lua_state;
    if (L == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Lua: Couldn't allocate a lua state");
        return;
    }

#ifndef HAL_CONSOLE_DISABLED
    const int inital_mem = lua_gc(L, LUA_GCCOUNT, 0) * 1024 + lua_gc(L, LUA_GCCOUNTB, 0);
#endif

    lua_atpanic(L, atpanic);
    load_generated_bindings(L);

    // set up string metatable. we set up one for all scripts that no script has
    // access to, as it's impossible to set up one per-script and we don't want
    // any script to be able to mess with it.
    lua_pushliteral(L, "");  /* dummy string */
    lua_createtable(L, 0, 1);  /* table to be metatable for strings */
    luaopen_string(L);  /* get string library */
    lua_setfield(L, -2, "__index");  /* metatable.__index = string */
    lua_setmetatable(L, -2);  /* set table as metatable for strings */
    lua_pop(L, 1);  /* pop dummy string */

#ifndef HAL_CONSOLE_DISABLED
    const int loaded_mem = lua_gc(L, LUA_GCCOUNT, 0) * 1024 + lua_gc(L, LUA_GCCOUNTB, 0);
    DEV_PRINTF("Lua: State memory usage: %i + %i\n", inital_mem, loaded_mem - inital_mem);
#endif

    // Scan the filesystem in an appropriate manner and autostart scripts
    // Skip those directores disabled with SCR_DIR_DISABLE param
    uint16_t dir_disable = AP_Scripting::get_singleton()->get_disabled_dir();
    bool loaded = false;
    if ((dir_disable & uint16_t(AP_Scripting::SCR_DIR::SCRIPTS)) == 0) {
#if AP_SCRIPTING_ENCRYPTION_ENABLED
        encrypt_all_scripts_in_dir(SCRIPTING_DIRECTORY);
#endif
        load_all_scripts_in_dir(L, SCRIPTING_DIRECTORY);
        loaded = true;
    }
#ifdef HAL_HAVE_AP_ROMFS_EMBEDDED_LUA
    if ((dir_disable & uint16_t(AP_Scripting::SCR_DIR::ROMFS)) == 0) {
        load_all_scripts_in_dir(L, "@ROMFS/scripts");
        loaded = true;
    }
#endif
    if (!loaded) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Lua: All directory's disabled see SCR_DIR_DISABLE");
    }

#ifndef __clang_analyzer__
    succeeded_initial_load = true;
#endif // __clang_analyzer__

    uint32_t expansion_size = 0;

    while (AP_Scripting::get_singleton()->should_run()) {
#if defined(AP_SCRIPTING_CHECKS) && AP_SCRIPTING_CHECKS >= 1
        if (lua_gettop(L) != 0) {
            AP_HAL::panic("Lua: Stack should be empty before running scripts");
        }
#endif // defined(AP_SCRIPTING_CHECKS) && AP_SCRIPTING_CHECKS >= 1

        if (scripts != nullptr) {
#if defined(AP_SCRIPTING_CHECKS) && AP_SCRIPTING_CHECKS >= 1
              // Sanity check that the scripts list is ordered correctly
              script_info *sanity = scripts;
              while (sanity->next != nullptr) {
                  if (sanity->next_run_ms > sanity->next->next_run_ms) {
                      AP_HAL::panic("Lua: Script tasking order has been violated");
                  }
                  sanity = sanity->next;
              }
#endif // defined(AP_SCRIPTING_CHECKS) && AP_SCRIPTING_CHECKS >= 1

            // compute delay time
            uint64_t now_ms = AP_HAL::millis64();
            if (now_ms < scripts->next_run_ms) {
                hal.scheduler->delay(scripts->next_run_ms - now_ms);
            }

            if (option_is_set(AP_Scripting::DebugOption::RUNTIME_MSG)) {
                GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Lua: Running %s", scripts->name);
            }
            // take a copy of the script name for the purposes of
            // logging statistics.  "scripts" may become invalid
            // during the "run_next_script" call, below.
            char script_name[128+1] {};
            strncpy_noterm(script_name, scripts->name, 128);

#if DISABLE_INTERRUPTS_FOR_SCRIPT_RUN
            void *istate = hal.scheduler->disable_interrupts_save();
#endif

            const int startMem = lua_gc(L, LUA_GCCOUNT, 0) * 1024 + lua_gc(L, LUA_GCCOUNTB, 0);
            const uint32_t loadEnd = AP_HAL::micros();

            // NOTE!  the base pointer of our scripts linked list,
            // *and all its contents* may become invalid as part of
            // "run_next_script"!  So do *NOT* attempt to access
            // anything that was in *scripts after this call.
            run_next_script(L);

            const uint32_t runEnd = AP_HAL::micros();
            const int endMem = lua_gc(L, LUA_GCCOUNT, 0) * 1024 + lua_gc(L, LUA_GCCOUNTB, 0);

#if DISABLE_INTERRUPTS_FOR_SCRIPT_RUN
            hal.scheduler->restore_interrupts(istate);
#endif

            update_stats(script_name, runEnd - loadEnd, endMem, endMem - startMem);


            // garbage collect after each script, this shouldn't matter, but seems to resolve a memory leak
            lua_gc(L, LUA_GCCOLLECT, 0);

        } else {
            if (option_is_set(AP_Scripting::DebugOption::NO_SCRIPTS_TO_RUN)) {
                GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Lua: No scripts to run");
            }
            hal.scheduler->delay(1000);
        }

        /*
          report a warning if SCR_HEAP_SIZE wasn't adequate and we
          expanded at runtime, so the user can fix it for future
          flights
         */
        const uint32_t new_expansion_size = _heap.get_expansion_size();
        if (new_expansion_size > expansion_size) {
            expansion_size = new_expansion_size;
            set_and_print_new_error_message(MAV_SEVERITY_WARNING, "Required SCR_HEAP_SIZE over %u", unsigned(expansion_size));
        }

        // re-print the latest error message every 10 seconds 10 times
        const uint8_t error_prints = 10;
        if ((print_error_count < error_prints) && (AP_HAL::millis() - last_print_ms > 10000)) {
            // note that we do not clear the buffer after we have finished printing, this allows it to be used for a pre-arm check
            print_error(MAV_SEVERITY_DEBUG);
            print_error_count++;
        }
    }

    // make sure all scripts have been removed
    while (scripts != nullptr) {
        remove_script(lua_state, scripts);
    }

    if (lua_state != nullptr) {
        lua_close(lua_state); // shutdown the old state
        lua_state = nullptr;
    }

    error_msg_buf_sem.take_blocking();
    if (error_msg_buf != nullptr) {
        _heap.deallocate(error_msg_buf);
        error_msg_buf = nullptr;
    }
    error_msg_buf_sem.give();
}

// Return the file checksums of running and loaded scripts
uint32_t lua_scripts::get_loaded_checksum()
{
    WITH_SEMAPHORE(crc_sem);
    return loaded_checksum;
}

uint32_t lua_scripts::get_running_checksum()
{
    WITH_SEMAPHORE(crc_sem);
    return running_checksum;
}

#if AP_SCRIPTING_ENCRYPTION_ENABLED
void lua_scripts::create_nonce(uint8_t nonce[24], const char* scriptname)
{
    for (uint8_t i = 0; i < 24; i++) {
        nonce[i] = rand();
    }
#if AP_SCRIPTING_ENCRYPTION_UUID_ENABLED
    uint8_t nonce_len;
    // use the serial number of the board for the nonce so that it can only be
    // decrypted on this board
    hal.util->get_system_id_unformatted(nonce, nonce_len);
#endif
}

bool lua_scripts::encrypt_script(char* script, uint8_t mac[16], const uint8_t nonce[24], size_t scriptlen)
{
    const struct ap_secure_data* keys = AP_CheckFirmware::find_public_keys();

    if (keys == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "No encryption key found");
        return false;
    }

    // the first three keys will be ArduPilot_public_keyx.dat
    crypto_lock(mac, (uint8_t*)script, keys->public_key[3].key, nonce, (const uint8_t*)script, scriptlen);
    return true;
}

bool lua_scripts::decrypt_script(char* script, const uint8_t mac[16], const uint8_t nonce[24], size_t scriptlen)
{
    const struct ap_secure_data* keys = AP_CheckFirmware::find_public_keys();

    if (keys == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "No encryption key found");
        return false;
    }

    // try decrypting with all the available keys until we have success or run out of key
    for (uint8_t i = 0; i < AP_PUBLIC_KEY_MAX_KEYS; i++) {
        if (crypto_unlock((uint8_t*)script, keys->public_key[i].key, nonce, mac, (const uint8_t*)script, scriptlen) == 0) {
            return true;
        }
    }
    return false;
}
#endif

#endif  // AP_SCRIPTING_ENABLED
