#pragma once

#include <Arduino.h>

namespace WifiConsole {

void begin();
void process();
void printHelp();
bool handleCommand(const String &cmd, const String &args, bool machineBusy);

} // namespace WifiConsole

