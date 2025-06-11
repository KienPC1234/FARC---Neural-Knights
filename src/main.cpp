#include <Arduino.h>
#include "TelnetServer.h"

void setup() {
  Serial.begin(115200);
  delay(100);
  initTelnet(true);
  logMessage(LOG_INFO, "Starting logging stress test");
}

void loop() {
  handleTelnet();

  // Normal logging every 100ms
  unsigned long startTime = micros();
  logPrintf(LOG_INFO, "🌡️ Temperature: %.2f°C", 25.3);
  unsigned long logTime = micros() - startTime;
  logPrintf(LOG_DEBUG, "⏱️ Log time: %lu us", logTime);

  // Burst logging test (10 rapid logs every 10 iterations)
  static int counter = 0;
  if (counter++ % 10 == 0) {
    logMessage(LOG_INFO, "Starting burst log test");
    for (int i = 0; i < 10; i++) {
      logPrintf(LOG_INFO, "🔥 Burst log #%d: %.2f°C", i + 1, 25.3 + (i * 0.1));
    }
    logPrintf(LOG_DEBUG, "Free heap: %lu bytes", ESP.getFreeHeap());
  }

  delay(100); // Reduced from 2000ms for faster logging
}