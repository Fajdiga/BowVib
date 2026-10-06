#pragma once
#include <stdbool.h>
/* Caller holds capture_control_lock; diagnostics and downloads stay on local USB. */
bool shot_usb_command(const char *command);
