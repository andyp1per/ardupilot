#pragma once

#define LOG_IDS_FROM_AC_ATTITUDECONTROL \
    LOG_RATE_MSG, \
    LOG_ANG_MSG

// @LoggerMessage: RATE
// @Description: Desired and achieved vehicle attitude rates. Not logged in Fixed Wing Plane modes.
// @Field: TimeUS: Time since system startup
// @Field: RDes: vehicle desired roll rate
// @Field: R: achieved vehicle roll rate
// @Field: ROut: normalized output for Roll
// @Field: PDes: vehicle desired pitch rate
// @Field: P: vehicle pitch rate
// @Field: POut: normalized output for Pitch
// @Field: Y: achieved vehicle yaw rate
// @Field: YOut: normalized output for Yaw
// @Field: YDes: vehicle desired yaw rate
// @Field: ADes: desired vehicle vertical acceleration
// @Field: A: achieved vehicle vertical acceleration
// @Field: AOut: percentage of vertical thrust output current being used
// @Field: AOutSlew: vertical thrust output slew rate
struct PACKED log_Rate {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float   control_roll;
    float   roll;
    float   roll_out;
    float   control_pitch;
    float   pitch;
    float   pitch_out;
    float   control_yaw;
    float   yaw;
    float   yaw_out;
    float   control_accel;
    float   accel;
    float   accel_out;
    float   throttle_slew;
};

// @LoggerMessage: ANG
// @Description: Attitude control attitude
// @Field: TimeUS: Timestamp of the current Attitude loop
// @Field: DesRoll: vehicle desired roll
// @Field: Roll: achieved vehicle roll
// @Field: DesPitch: vehicle desired pitch
// @Field: Pitch: achieved vehicle pitch
// @Field: DesYaw: vehicle desired yaw
// @Field: Yaw: achieved vehicle yaw
// @Field: Dt: attitude delta time
struct PACKED log_ANG {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float control_roll;
    float roll;
    float control_pitch;
    float pitch;
    float control_yaw;
    float yaw;
    float sensor_dt;
};

#define LOG_STRUCTURE_FROM_AC_ATTITUDECONTROL        \
    { LOG_RATE_MSG, sizeof(log_Rate), \
        "RATE", "Qfffffffffffff",  "TimeUS,RDes,R,ROut,PDes,P,POut,YDes,Y,YOut,ADes,A,AOut,AOutSlew", "skk-kk-kk-oo--", "F?????????BB--" , true }, \
    { LOG_ANG_MSG, sizeof(log_ANG),\
        "ANG", "Qfffffff", "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,Dt", "sddddhhs", "F0000000" , true }
