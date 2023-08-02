#include "LogReader.h"

#include <AP_LoggerFileReader/MsgHandler.h>

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <signal.h>


#define DEBUG 1
#if DEBUG
# define debug(fmt, args...)     printf(fmt "\n", ##args)
#else
# define debug(fmt, args...)
#endif

#define streq(x, y) (!strcmp(x, y))

LogReader::LogReader() :
    AP_LoggerFileReader() { }

/*
  see if a type is in a list of types
 */
bool LogReader::in_list(const char *type, const char *list[])
{
    if (list == NULL) {
        return false;
    }
    for (uint8_t i=0; list[i] != NULL; i++) {
        if (strcmp(type, list[i]) == 0) {
            return true;
        }
    }
    return false;
}

bool LogReader::handle_log_format_msg(const struct log_Format &f)
{
    // emit the output as we receive it:
    AP::logger().WriteBlock((void*)&f, sizeof(f));

	char name[5];
	memset(name, '\0', 5);
	memcpy(name, f.name, 4);

    if (msgparser[f.type] != NULL) {
        return true;
    }

    // map from format name to a parser subclass:
	if (streq(name, "PARM")) {
        msgparser[f.type] = new LR_MsgHandler(formats[f.type]);
    }
    return true;
}

bool LogReader::handle_msg(const struct log_Format &f, uint8_t *msg) {
    // emit the output as we receive it:
    AP::logger().WriteBlock(msg, f.length);

    MsgHandler *p = msgparser[f.type];
    if (p == NULL) {
        return true;
    }

    p->process_message(msg);

    return true;
}

bool LogReader::set_parameter(const char *name, float value)
{
    enum ap_var_type var_type;
    AP_Param *vp = AP_Param::find(name, &var_type);
    if (vp == NULL) {
        // a lot of parameters will not be found - e.g. FORMAT_VERSION
        // and all of the vehicle-specific parameters, ....
        return false;
    }
    float old_value = 0;
    if (var_type == AP_PARAM_FLOAT) {
        old_value = ((AP_Float *)vp)->cast_to_float();
        ((AP_Float *)vp)->set(value);
    } else if (var_type == AP_PARAM_INT32) {
        old_value = ((AP_Int32 *)vp)->cast_to_float();
        ((AP_Int32 *)vp)->set(value);
    } else if (var_type == AP_PARAM_INT16) {
        old_value = ((AP_Int16 *)vp)->cast_to_float();
        ((AP_Int16 *)vp)->set(value);
    } else if (var_type == AP_PARAM_INT8) {
        old_value = ((AP_Int8 *)vp)->cast_to_float();
        ((AP_Int8 *)vp)->set(value);
    } else {
        AP_HAL::panic("What manner of evil is var_type=%u", var_type);
    }
    if (fabsf(old_value - value) > 1.0e-12) {
        ::printf("Changed %s to %.8f from %.8f\n", name, value, old_value);
    }
    return true;
}

