#include "TelnetServer.h"
#include <WiFi.h>
#include <ESPTelnet.h>
#include <stdarg.h>

static const char* ssid = "KIEN-PC 4614";
static const char* password = "12345678";
static const uint16_t TELNET_PORT = 23;
static const uint8_t MAX_WIFI_RETRIES = 20;
static const uint16_t WIFI_RETRY_DELAY = 200;
static const uint16_t BUFFER_SIZE = 128;

static IPAddress local_IP(192, 168, 137, 2);
static IPAddress gateway(192, 168, 137, 1);
static IPAddress subnet(255, 255, 255, 0);

static ESPTelnet telnet;
static bool isDevMode = true;
static bool isTelnetInitialized = false;

static String getLogPrefix(LogLevel level) {
  switch (level) {
    case LOG_INFO:  return "[INFO] ";
    case LOG_DEBUG: return "[DEBUG] ";
    case LOG_ERROR: return "[ERROR] ";
    default:        return "[UNKNOWN] ";
  }
}

void initTelnet(bool devMode) {
  isDevMode = devMode;

  if (!WiFi.config(local_IP, gateway, subnet)) {
    logMessage(LOG_ERROR, "Failed to configure static IP!");
    return;
  }

  WiFi.begin(ssid, password);
  logMessage(LOG_INFO, "Connecting to WiFi");

  uint8_t retries = 0;
  unsigned long startTime = millis();
  const unsigned long timeout = 10000;

  while (WiFi.status() != WL_CONNECTED && retries++ < MAX_WIFI_RETRIES && (millis() - startTime) < timeout) {
    delay(WIFI_RETRY_DELAY);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    logMessage(LOG_ERROR, "WiFi connection failed.");
    return;
  }

  logMessage(LOG_INFO, "WiFi connected.");
  logMessage(LOG_INFO, "IP: " + WiFi.localIP().toString());
  logPrintf(LOG_INFO, "Telnet command: telnet %s %d", WiFi.localIP().toString().c_str(), TELNET_PORT);

  if (telnet.begin(TELNET_PORT)) {
    logMessage(LOG_INFO, "Telnet server started.");
    isTelnetInitialized = true;
  } else {
    logMessage(LOG_ERROR, "Failed to start Telnet server.");
    return;
  }

  telnet.onConnect([](String ip) {
    logMessage(LOG_INFO, "🔗 New connection from: " + ip);
  });

  telnet.onInputReceived([](String input) {
    input.trim();
    if (input.length() == 0) return;

    logMessage(LOG_DEBUG, "📨 Received: " + input);

    if (input.equalsIgnoreCase("led_on")) {
      digitalWrite(2, HIGH);
      logMessage(LOG_INFO, "💡 LED turned ON");
    } else if (input.equalsIgnoreCase("led_off")) {
      digitalWrite(2, LOW);
      logMessage(LOG_INFO, "💡 LED turned OFF");
    } else {
      logMessage(LOG_ERROR, "❓ Invalid command.");
    }
  });

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
}

void handleTelnet() {
  if (isDevMode && isTelnetInitialized) {
    telnet.loop();
  }
}

void logMessage(LogLevel level, const String& msg) {
  String output = getLogPrefix(level) + msg;
  if (isDevMode && isTelnetInitialized && telnet.isConnected()) {
    telnet.println(output);
  }
  else{
    Serial.println(output);
  }
}

void logPrintf(LogLevel level, const char* fmt, ...) {
  char buffer[BUFFER_SIZE];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, BUFFER_SIZE, fmt, args);
  va_end(args);

  String msg = getLogPrefix(level) + String(buffer);
  if (isDevMode && isTelnetInitialized && telnet.isConnected()) {
    telnet.println(msg); 
  }
  else{
    Serial.println(msg);
  }
}