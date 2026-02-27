#pragma once

#include <Arduino.h>

namespace WifiConsole {

void begin();
void process();
void printHelp();
void printStatus();
bool handleCommand(const String &cmd, const String &args, bool machineBusy);

} // namespace WifiConsole

