#pragma once

#include "Settings.h"

extern StringSetting* config_filename;

extern StringSetting* build_info;

extern StringSetting* start_message;

extern IntSetting* status_mask;

extern IntSetting* sd_fallback_cs;

// Milliseconds between automatic status reports on the console channel, or 0
// for none.  Config-file UART channels get this from report_interval_ms; the
// console is built in code, so it needs a setting of its own.
extern IntSetting* console_report_interval;

extern EnumSetting* message_level;

extern EnumSetting* gcode_echo;

void make_proxies();
void make_coordinates();
