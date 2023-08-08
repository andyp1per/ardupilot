#pragma once

#include <AP_LoggerFileReader/VehicleType.h>
#include <AP_LoggerFileReader/AP_LoggerFileReader.h>
#include <AP_LoggerFileReader/MsgHandler.h>

class LogReader : public AP_LoggerFileReader
{
    friend class GYR_MsgHandler;
    friend class FTN_MsgHandler;
public:
    LogReader();

    VehicleType::vehicle_type vehicle;

    static bool set_parameter(const char *name, float value);

    bool handle_log_format_msg(const struct log_Format &f) override;
    bool handle_msg(const struct log_Format &f, uint8_t *msg) override;
    const log_GYR& get_next_gyr_msg();

    static bool in_list(const char *type, const char *list[]);

protected:

private:
    log_GYR gyr;
    bool new_gyr;
    float notchF;

    class MsgHandler *msgparser[LOGREADER_MAX_FORMATS] {};
};

class GYR_MsgHandler : public MsgHandler {
public:
    GYR_MsgHandler(struct log_Format &f, LogReader& _reader);
    void process_message(uint8_t *msg) override;
private:
    LogReader& reader;
};

class FTN_MsgHandler : public MsgHandler {
public:
    FTN_MsgHandler(struct log_Format &f, LogReader& _reader);
    void process_message(uint8_t *msg) override;
private:
    LogReader& reader;
};
