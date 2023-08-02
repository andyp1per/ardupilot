#pragma once

#include <AP_LoggerFileReader/VehicleType.h>
#include <AP_LoggerFileReader/AP_LoggerFileReader.h>
#include <AP_LoggerFileReader/MsgHandler.h>

class LR_MsgHandler : public MsgHandler {
public:
    LR_MsgHandler(struct log_Format &f);
    void process_message(uint8_t *msg) override {
    }
};

class LogReader : public AP_LoggerFileReader
{
public:
    LogReader();

    VehicleType::vehicle_type vehicle;

    static bool set_parameter(const char *name, float value);

    bool handle_log_format_msg(const struct log_Format &f) override;
    bool handle_msg(const struct log_Format &f, uint8_t *msg) override;

    static bool in_list(const char *type, const char *list[]);

protected:

private:
    struct LogStructure *_log_structure;
    uint8_t _log_structure_count;

    class MsgHandler *msgparser[LOGREADER_MAX_FORMATS] {};
};
