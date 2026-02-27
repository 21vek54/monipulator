#pragma once

#include <Arduino.h>

namespace MqttLink {

using CommandHandler = void (*)(const String &cmd);

void setCommandHandler(CommandHandler handler);
void begin();
void process();
void printStatus();

} // namespace MqttLink
