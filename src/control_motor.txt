#include <Arduino.h>
#include <TelnetServer.h>
#include <PS2X_lib.h>
#include <MecanumControl.h>

#define PS2_DAT   19  // MISO
#define PS2_CMD   23  // MOSI
#define PS2_SEL    5  // SS
#define PS2_CLK   18  // SCLK

#define PRESSURES false
#define RUMBLE    false

PS2X ps2x;
MecanumControl mecanum;
bool isStopped = false;
int ps2Type = 0;

// Deadband & curve
const float DEADBAND = 0.05;
const float EXPONENT = 1.5;
float applyDeadbandAndCurve(float v) {
  if (fabs(v) < DEADBAND) return 0;
  float s = (v > 0) ? 1 : -1;
  v = (fabs(v) - DEADBAND) / (1.0 - DEADBAND);
  return s * pow(v, EXPONENT);
}

// Slew-rate limiter
const float SLEW_RATE = 0.02;
float slewFilter(float target, float current) {
  float d = target - current;
  if (fabs(d) > SLEW_RATE)
    return current + SLEW_RATE * (d > 0 ? 1 : -1);
  return target;
}

// Loop timing
const unsigned long LOOP_INTERVAL = 20000; // 20ms
unsigned long lastLoopTime = 0;

// Trạng thái tốc độ đã lọc
float currX = 0, currY = 0, currOmega = 0;

void setup() {
  Serial.begin(115200);
  delay(100);

  // Khởi tạo tay cầm PS2
  int error = -1;
  while (error != 0) {
    delay(500);
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, PRESSURES, RUMBLE);
  }
  ps2Type = ps2x.readType();
  if (ps2Type == 1) logMessage(LOG_INFO, "DualShock Controller found");

  // Khởi tạo Telnet
  initTelnet(true);
  logMessage(LOG_INFO, "Starting logging with Mecanum control");

  lastLoopTime = micros();
}

void loop() {
  handleTelnet();

  unsigned long now = micros();
  if (now - lastLoopTime < LOOP_INTERVAL) return;
  lastLoopTime = now;

  if (ps2Type == 1) {
    ps2x.read_gamepad(false, 0);

    // Chuyển trạng thái dừng/tiếp tục
    if (ps2x.Button(PSB_SELECT)) {
      isStopped = !isStopped;
      logMessage(LOG_INFO, isStopped ? "Vehicle stopped" : "Vehicle resumed");
      delay(200);
    }

    if (!isStopped) {
      // Đọc input [-1…1]
      float rawY = (ps2x.Analog(PSS_LY) - 128.0f) / 128.0f;
      float rawX = (ps2x.Analog(PSS_LX) - 128.0f) / 128.0f;
      float rawO = (ps2x.Analog(PSS_RX) - 128.0f) / 128.0f;
      // Deadzone + curve
      float tgtY = applyDeadbandAndCurve(-rawY); // chú ý đảo chiều nếu cần
      float tgtX = applyDeadbandAndCurve(rawX);
      float tgtO = applyDeadbandAndCurve(rawO);
      // Slew-rate
      currY     = slewFilter(tgtY, currY);
      currX     = slewFilter(tgtX, currX);
      currOmega = slewFilter(tgtO, currOmega);

      logPrintf(LOG_INFO, "🎮 Ctrl: Fwd=%.2f  Str=%.2f  Rot=%.2f", currY, currX, currOmega);
      mecanum.moveWithRotation(currX, currY, currOmega);
    } else {
      mecanum.moveWithRotation(0, 0, 0);
    }
  }
}
