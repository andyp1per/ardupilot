#include <GCS_MAVLink/GCS.h>
#include <cstdint>
#include <limits>
#include <skybrush/skybrush.h>

#include "AC_DroneShowManager.h"
#include "DroneShow_Constants.h"
#include "DroneShow_CustomPackets.h"
#include "DroneShowPyroDevice.h"
#include "include/mavlink/v2.0/all/mavlink.h"
#include "skybrush/lights.h"

static bool uint64_sub_safe(uint64_t a, uint64_t b, int32_t* result);

MAV_RESULT AC_DroneShowManager::handle_command_int_packet(const mavlink_command_int_t &packet)
{
    switch (packet.command) {

    case MAV_CMD_USER_1: {
        // param1: command code
        // remaining params depend on param1
        //
        // 0 = reload current show
        // 1 = clear current show
        // 2 = trigger pyro test
        // 3 = execute another COMMAND_INT when the group index of the drone is
        //     set to a specific value. param6 (y) contains the _real_ command
        //     code to execute in its lower 16 bits, while bits 30:16 (inclusive)
        //     contain the group index plus 1, zero meaning "all groups". MSB
        //     (sign bit) must be zero. We reserve the right to repurpose high
        //     bits in future versions and sacrifice higher group indices.
        //     param7 of the command is remapped to param1 of the injected command,
        //     while param7 in the injected command is always zero.
        if (is_zero(packet.param1)) {
            // Reload current show
            if (reload_or_clear_show(/* do_clear = */ 0)) {
                return MAV_RESULT_ACCEPTED;
            } else {
                return MAV_RESULT_FAILED;
            }
        } else if (is_equal(packet.param1, 1.0f)) {
            // Clear current show
            if (reload_or_clear_show(/* do_clear = */ 1)) {
                return MAV_RESULT_ACCEPTED;
            } else {
                return MAV_RESULT_FAILED;
            }
        } else if (is_equal(packet.param1, 2.0f)) {
            // Trigger pyro test
            uint8_t start = packet.param2 >= 0 && packet.param2 < 255 ? packet.param2 : 255;
            uint8_t num_channels = packet.param3 >= 0 && packet.param3 < 256 ?
                static_cast<uint8_t>(packet.param3) : 255;
            uint32_t delay_msec = isfinite(packet.param4) && packet.param4 >= 0 ? (packet.param4 * 1000.0f) : 0;
            
            if (num_channels == 0) {
                // num_channels == 0 means all channels starting from 'start'
                // up to whatever the pyro device supports. We just use 255 and
                // then clamp it later
                num_channels = 255;
            }

            if (start < 255) {
                if (_pyro_device == nullptr) {
                    // No pyro device is configured, cannot start the test
                    return MAV_RESULT_FAILED;
                } else {
                    if (static_cast<uint16_t>(start) + num_channels > _pyro_device->num_channels()) {
                        num_channels = _pyro_device->num_channels() - start;
                    }
                    _pyro_test_state.start(start, num_channels, delay_msec);
                    return MAV_RESULT_ACCEPTED;
                }
            }
        } else if (is_equal(packet.param1, 3.0f)) {
            // Execute group-specific command.
            int group_index_plus_one = packet.y >> 16;
            if (packet.y < 0) {
                // MSB is 1, ignore.
                return MAV_RESULT_UNSUPPORTED;
            } else if (group_index_plus_one <= 0 || is_in_group(group_index_plus_one - 1)) {
                // Broadcast to all groups (group_index_plus_one == 0) or
                // targeted to our group
                mavlink_command_int_t injected_packet = packet;

                injected_packet.command = packet.y & UINT16_MAX;
                injected_packet.param1 = packet.z;
                injected_packet.y = 0;
                injected_packet.z = 0;

                return gcs().inject_command_int_packet(injected_packet);
            } else {
                return MAV_RESULT_ACCEPTED;
            }
        }

        // Unsupported command code
        return MAV_RESULT_UNSUPPORTED;
    }

    case MAV_CMD_USER_2: {
        // param1: command code
        // remaining params depend on param1
        if (is_zero(packet.param1)) {
            // Set show origin, orientation and AMSL with a single command.
            // This is supported with COMMAND_INT MAVLink packets only as we
            // do not want to lose precision in the lat/lng direction due to
            // float representation
            //
            // param4: orientation
            // param5 (x): latitude (degE7)
            // param6 (y): longitude (degE7)
            // param7 (z): AMSL (mm)
            if (configure_show_coordinate_system(
                packet.x, packet.y, static_cast<int32_t>(packet.z),
                packet.param4
            )) {
                return MAV_RESULT_ACCEPTED;
            } else {
                return MAV_RESULT_FAILED;
            }
        }

        // Unsupported command code
        return MAV_RESULT_UNSUPPORTED;
    }

    default:
        // Unsupported command code
        return MAV_RESULT_UNSUPPORTED;
    }
}

bool AC_DroneShowManager::handle_message(mavlink_channel_t chan, const mavlink_message_t& msg)
{
    switch (msg.msgid)
    {
        // DATA16, DATA32, DATA64, DATA96 packets are used for custom commands.
        // We do not distinguish between them because MAVLink2 truncates the
        // trailing zeros anyway.
        case MAVLINK_MSG_ID_DATA16:
            return _handle_data16_message(chan, msg);

        case MAVLINK_MSG_ID_DATA32:
            return _handle_data32_message(chan, msg);

        case MAVLINK_MSG_ID_DATA64:
            return _handle_data64_message(chan, msg);

        case MAVLINK_MSG_ID_DATA96:
            return _handle_data96_message(chan, msg);

        case MAVLINK_MSG_ID_LED_CONTROL:
            // The drone show LED listens on the "secret" LED ID 42 with a
            // pattern of 42 as well. Any message that does not match this
            // specification is handled transparently by the "core" MAVLink
            // GCS module.
            return _handle_led_control_message(msg);

        default:
            return false;
    }
}

bool AC_DroneShowManager::_handle_custom_data_message(mavlink_channel_t chan, uint8_t type, void* data, uint8_t length)
{
    uint8_t reply[16];
    
    if (data == nullptr) {
        return false;
    }

    // We allocate type 0x5C for the GCS-to-drone packets (0X5B is the drone-to-GCS
    // status packet), and sacrifice the first byte of the payload to identify
    // the _real_ message type. This reduces the chance of clashes with other
    // DATA* messages from third parties. The type that we receive in this
    // function is the _real_ message type.
    switch (type) {
        // Broadcast start time and authorization state of the show
        case CustomPackets::START_CONFIG:
            {
                if (length < offsetof(CustomPackets::start_config_t, optional_part)) {
                    // Packet too short
                    return false;
                }

                CustomPackets::start_config_t* start_config = static_cast<CustomPackets::start_config_t*>(data);

                // Update start time expressed in GPS time of week
                if (start_config->start_time < 0) {
                    _params.start_time_gps_sec.set(-1);
                } else if (start_config->start_time < GPS_WEEK_LENGTH_SEC) {
                    _params.start_time_gps_sec.set(start_config->start_time);
                }

                // Update authorization flag
                _params.authorization.set(start_config->authorization);

                // Do we have the optional second part?
                if (length >= sizeof(CustomPackets::start_config_t)) {
                    // Optional second part is used by the GCS to convey how many
                    // milliseconds there are until the start of the show. If this
                    // part exists and is positive, _and_ we are using the internal
                    // clock to synchronize the start, then we update the start
                    // time based on this
                    if (!uses_gps_time_for_show_start()) {
                        int32_t countdown_msec = start_config->optional_part.countdown_msec;

                        if (countdown_msec < -GPS_WEEK_LENGTH_MSEC) {
                            clear_scheduled_start_time();
                        } else if (countdown_msec >= 0 && countdown_msec < GPS_WEEK_LENGTH_MSEC) {
                            schedule_delayed_start_after(countdown_msec);
                        }
                    }
                }

                return true;
            }
            break;

        // Schedule collective RTL; obsolete, does nothing, kept for backward compatibility
        case CustomPackets::DEPRECATED_CRTL_TRIGGER:
            return true;

        // Configure geofences with a single call
        case CustomPackets::SIMPLE_GEOFENCE_SETUP:
            if (length >= sizeof(CustomPackets::simple_geofence_setup_header_t)) {
                CustomPackets::simple_geofence_setup_header_t* geofence_setup = static_cast<CustomPackets::simple_geofence_setup_header_t*>(data);
                DroneShow_FenceConfig fence_config;
                CustomPackets::acknowledgment_t* ack_packet = reinterpret_cast<CustomPackets::acknowledgment_t*>(reply + 1);
                size_t points_payload_length_in_bytes = length - sizeof(CustomPackets::simple_geofence_setup_header_t);
                size_t num_points = points_payload_length_in_bytes / sizeof(DroneShow_FencePoint);
                MAV_RESULT result = MAV_RESULT_FAILED;

                // Convert from geofence_setup to fence_config
                fence_config.max_altitude_dm = geofence_setup->max_altitude_dm;
                fence_config.radius_dm = geofence_setup->radius_dm;
                fence_config.action = geofence_setup->flags & 0x0f;
                fence_config.num_points = geofence_setup->num_points;
                fence_config.points = reinterpret_cast<DroneShow_FencePoint*>(
                    reinterpret_cast<uint8_t*>(geofence_setup) +
                    sizeof(CustomPackets::simple_geofence_setup_header_t)
                );
                if (fence_config.num_points <= num_points) {
                    result = configure_fences(fence_config) ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
                }

                // Prepare the reply packet
                memset(reply, 0, sizeof(reply));
                reply[0] = CustomPackets::ACKNOWLEDGMENT;
                ack_packet->ack_token = geofence_setup->ack_token;
                ack_packet->result = result;

                // Send the reply packet
                mavlink_msg_data16_send(
                    chan,
                    CustomPackets::DRONE_TO_GCS,   // Skybrush status packet type marker
                    sizeof(CustomPackets::acknowledgment_t),     // effective packet length
                    reply
                );

                return true;
            }
            break;

        // Acknowledgment packets; these can be ignored (but we still return
        // true as we do not want any other handlers to handle them)
        case CustomPackets::ACKNOWLEDGMENT:
            if (length >= sizeof(CustomPackets::acknowledgment_t)) {
                return true;
            }
            break;
            
        // Time axis configuration packet, used to implement suspension and resume
        case CustomPackets::TIME_AXIS_CONFIG:
            return _handle_time_axis_configuration_packet(data, length);
    }

    return false;
}

bool AC_DroneShowManager::_handle_data16_message(mavlink_channel_t chan, const mavlink_message_t& msg)
{
    mavlink_data16_t packet;
    mavlink_msg_data16_decode(&msg, &packet);
    if (packet.type != CustomPackets::GCS_TO_DRONE || packet.len < 1) {
        return false;
    }
    return _handle_custom_data_message(chan, packet.data[0], packet.data + 1, packet.len - 1);
}

bool AC_DroneShowManager::_handle_data32_message(mavlink_channel_t chan, const mavlink_message_t& msg)
{
    mavlink_data32_t packet;
    mavlink_msg_data32_decode(&msg, &packet);
    if (packet.type != CustomPackets::GCS_TO_DRONE || packet.len < 1) {
        return false;
    }
    return _handle_custom_data_message(chan, packet.data[0], packet.data + 1, packet.len - 1);
}

bool AC_DroneShowManager::_handle_data64_message(mavlink_channel_t chan, const mavlink_message_t& msg)
{
    mavlink_data64_t packet;
    mavlink_msg_data64_decode(&msg, &packet);
    if (packet.type != CustomPackets::GCS_TO_DRONE || packet.len < 1) {
        return false;
    }
    return _handle_custom_data_message(chan, packet.data[0], packet.data + 1, packet.len - 1);
}

bool AC_DroneShowManager::_handle_data96_message(mavlink_channel_t chan, const mavlink_message_t& msg)
{
    mavlink_data96_t packet;
    mavlink_msg_data96_decode(&msg, &packet);
    if (packet.type != CustomPackets::GCS_TO_DRONE || packet.len < 1) {
        return false;
    }
    return _handle_custom_data_message(chan, packet.data[0], packet.data + 1, packet.len - 1);
}

bool AC_DroneShowManager::_handle_time_axis_configuration_packet(void* data, uint8_t length)
{
    static uint16_t previous_seq_no = 0xFFFF;   // 0xFFFF is never a valid sequence number
    CustomPackets::time_axis_config_header_t* header;
    CustomPackets::time_axis_config_scene_header_t* scene_header;
    CustomPackets::time_axis_config_scene_entry_t* entry;
    CustomPackets::time_axis_config_trailer_t* trailer;
    int32_t origin_msec;
    uint8_t num_scenes, num_entries, scene_index, entry_index;
    uint8_t *ptr, *end;
    sb_screenplay_t new_screenplay;
    sb_screenplay_scene_t *scene;
    sb_time_axis_t* time_axis;
    sb_time_segment_t segment;
    bool success;

    if (length < sizeof(CustomPackets::time_axis_config_header_t) + sizeof(CustomPackets::time_axis_config_trailer_t)) {
        // Packet too short - even with no scenes the packet must be at least this long
        return false;
    }

    header = static_cast<CustomPackets::time_axis_config_header_t*>(data);
    num_scenes = header->num_scenes;
    
    if (length < (
        sizeof(CustomPackets::time_axis_config_header_t) +
        num_scenes * sizeof(CustomPackets::time_axis_config_scene_header_t) +
        sizeof(CustomPackets::time_axis_config_trailer_t)
    )) {
        // Packet too short - even with no segments in each of the scenes the packet
        // must be at least this long
        return false;
    }
    
    if (header->seq_no == previous_seq_no) {
        // Duplicate packet
        return true;
    }
    
    // Remember the sequence number
    previous_seq_no = header->seq_no;
    
    // We need to be extra careful here; if an error happens while we are setting up the
    // new scenes, we want to leave the existing screenplay intact. Therefore, we first
    // create a new screenplay, and then swap it with the existing one only if everything
    // went well.
    if (sb_screenplay_init(&new_screenplay) != SB_SUCCESS) {
        return false;
    }
    
    // From this point onwards we need to clean up the new screenplay if anything
    // goes wrong, so we can't return directly -- we need to jump to the exit label
    // instead. We use the 'success' variable to decide whether everything went well
    // (in which case we need to swap the new screenplay with the old one and destroy
    // the old one) or something went wrong (in which case we just destroy the new
    // screenplay and leave the old one intact).
    success = false;
    
    // Make sure that the new screenplay refers to the same RTH plan as the existing one
    sb_screenplay_set_rth_plan(&new_screenplay, sb_screenplay_get_rth_plan(&_screenplay));

    // Header processed; now process each of the scenes
    ptr = reinterpret_cast<uint8_t*>(data);
    end = ptr + length;
    ptr += sizeof(CustomPackets::time_axis_config_header_t);
    end -= sizeof(CustomPackets::time_axis_config_trailer_t);

    for (scene_index = 0; scene_index < num_scenes; scene_index++) {
        if (ptr + sizeof(CustomPackets::time_axis_config_scene_header_t) > end) {
            // Packet too short
            goto exit;
        }
        
        scene_header = reinterpret_cast<CustomPackets::time_axis_config_scene_header_t*>(ptr);
        ptr += sizeof(CustomPackets::time_axis_config_scene_header_t);
        
        // Add a new scene to the screenplay
        if (sb_screenplay_append_new_scene(&new_screenplay, &scene) != SB_SUCCESS) {
            // Could not add new scene
            goto exit;
        }
        
        // Check the scene ID and figure out whether this scene is for the main show
        // or for a coordinated RTH plan
        if (scene_header->scene_id == 0) {
            // Main show
            sb_screenplay_scene_update_contents_from(scene, &_main_show_scene);
        } else if ((scene_header->scene_id & 0xC000) == 0xC000) {
            // Coordinated RTH plan, starting at the number of seconds described by the 
            // lower 14 bits
            sb_rth_plan_t* rth_plan = sb_screenplay_get_rth_plan(&new_screenplay);
            sb_rth_plan_entry_t rth_plan_entry;
            if (rth_plan == NULL || sb_rth_plan_evaluate_at(rth_plan, scene_header->scene_id & 0x3FFF, &rth_plan_entry) != SB_SUCCESS) {
                // Could not evaluate RTH plan at the given time
                goto exit;
            }

            // TODO(ntamas): create a trajectory based on rth_plan_entry and set it to the scene
            sb_screenplay_scene_set_trajectory(scene, nullptr);
            
            // Use a fixed light program with RTH color
            {
                sb_light_program_t* rth_light_program = sb_light_program_new();
                if (rth_light_program) {
                    if (sb_light_program_set_constant_color(rth_light_program, get_rth_transition_color()) != SB_SUCCESS) {
                        // Probably out of memory; clean up and proceed without setting
                        // a light program
                        SB_DECREF(rth_light_program);
                        rth_light_program = nullptr;
                    }
                } else {
                    // Out of memory; proceed without setting a light program
                }
                sb_screenplay_scene_set_light_program(scene, rth_light_program);
                SB_XDECREF(rth_light_program);
            }
       } else {
            // Unknown scene ID; may be used in the future but it has no meaning now
            goto exit;
        }

        num_entries = scene_header->num_entries;
    
        // Find the time axis to manipulate
        time_axis = scene ? sb_screenplay_scene_get_time_axis(scene) : nullptr;
        if (time_axis == nullptr) {
            // Could not get time axis; this should not happen but let's be defensive
            goto exit;
        }
        
        // First, we set the origin of the new time axis
        {
            uint64_t epoch_msec;
    
            if (uses_gps_time_for_show_start()) {
                // When using GPS time for show start, the origin is assumed to be an absolute
                // time in milliseconds since the UNIX epoch
                epoch_msec = _start_time_unix_usec / 1000;
            } else {
                // When using the internal clock for show start, the origin is assumed to be
                // relative to the show start time. This is not really recommended but we need
                // to handle it nevertheless.
                epoch_msec = 0;
            }
            
            if (!uint64_sub_safe(scene_header->origin_msec, epoch_msec, &origin_msec)) {
                // Overflow or underflow
                goto exit;
            }
        }
    
        sb_time_axis_set_origin_msec(time_axis, origin_msec);
        
        // Add segments
        for (entry_index = 0; entry_index < num_entries; entry_index++) {
            if (ptr + sizeof(CustomPackets::time_axis_config_scene_entry_t) > end) {
                // Packet too short
                goto exit;
            }
            
            entry = reinterpret_cast<CustomPackets::time_axis_config_scene_entry_t*>(ptr);
            ptr += sizeof(CustomPackets::time_axis_config_scene_entry_t);
    
            if (entry->duration_msec == std::numeric_limits<uint32_t>::max()) {
                // Invalid duration for a finite segment
                goto exit;
            }
            
            segment = sb_time_segment_make(
                entry->duration_msec,
                entry->initial_rate_scaled / 65535.0f,
                entry->final_rate_scaled / 65535.0f
            );
            if (sb_time_axis_append_segment(time_axis, segment) != SB_SUCCESS) {
                goto exit;
            }
        }
        
        // Finally, set total duration of scene to be the duration of its segments on
        // the time axis
        sb_screenplay_scene_set_duration_msec(scene, sb_time_axis_get_total_duration_msec(time_axis));
    }
    
    // Add final, infinite segment to last scene. If there are no scenes, add a new
    // scene first.
    if (sb_screenplay_size(&new_screenplay) == 0) {
        if (sb_screenplay_append_new_scene(&new_screenplay, &scene) != SB_SUCCESS) {
            // Could not add new scene
            goto exit;
        }
    } else {
        scene = sb_screenplay_get_scene_ptr(
            &new_screenplay,
            sb_screenplay_size(&new_screenplay) - 1
        );
    }

    time_axis = scene ? sb_screenplay_scene_get_time_axis(scene) : nullptr;
    if (time_axis == nullptr) {
        // Could not get time axis; this should not happen but let's be defensive
        goto exit;
    }

    trailer = reinterpret_cast<CustomPackets::time_axis_config_trailer_t*>(ptr);
    segment = sb_time_segment_make_constant_rate(
        std::numeric_limits<uint32_t>::max(),
        trailer->initial_rate_scaled / 65535.0f
    );
    if (sb_time_axis_append_segment(time_axis, segment) == SB_SUCCESS) {
        sb_screenplay_scene_set_duration_msec(scene, sb_time_axis_get_total_duration_msec(time_axis));
        success = true;
    }

exit:
    if (success) {
        // Clean up the old screenplay and replace it with the new one
        sb_screenplay_destroy(&_screenplay);
        _screenplay = new_screenplay;
        
        // Add log entries containing the current screenplay
        write_screenplay_log_messages(header->seq_no, NULL);
    } else {
        // Clean up the new screenplay that we tried to prepare
        sb_screenplay_destroy(&new_screenplay);
    }
    
    return success;
}

/**
 * @brief Safely compute the difference of two uint64_t values and store the result
 *        in an int32_t variable.
 * 
 * @param a The minuend
 * @param b The subtrahend
 * @param result Pointer to the variable where the result will be stored
 * @return true if the subtraction was successful and the result fits in int32_t;
 *         false if an overflow or underflow occurred or the result does not fit
 */
static bool uint64_sub_safe(uint64_t a, uint64_t b, int32_t* result)
{
    if (a >= b) {
        uint64_t diff = a - b;
        if (diff <= static_cast<uint64_t>(std::numeric_limits<int32_t>::max())) {
            *result = static_cast<int32_t>(diff);
            return true;
        } else {
            return false;
        }
    } else {
        uint64_t diff = b - a;
        if (diff <= static_cast<uint64_t>(std::numeric_limits<int32_t>::max())) {
            *result = -static_cast<int32_t>(diff);
            return true;
        } else {
            return false;
        }
    }
}
