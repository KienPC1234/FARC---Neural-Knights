#pragma once

#include <Arduino.h>

enum LogLevel {
  LOG_INFO,
  LOG_DEBUG,
  LOG_ERROR
};

void initTelnet(bool devMode = true);
void handleTelnet();
void logMessage(LogLevel level, const String& msg);
void logPrintf(LogLevel level, const char* fmt, ...);