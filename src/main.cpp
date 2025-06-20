#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// PS2 Controller pins
#define PS2_DAT 12 // MISO
#define PS2_CMD 13 // MOSI
#define PS2_SEL 15 // SS
#define PS2_CLK 14 // SCLK
#define pressures false
#define rumble false

// Initialize two PWM drivers with same I2C address
Adafruit_PWMServoDriver pwm_mecanum = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm_servo = Adafruit_PWMServoDriver(0x40);
PS2X ps2x;

// Robot geometry for Mecanum wheels
const float wheelBase = 0.18;        // 18cm
const float trackWidth = 0.26;       // 26cm
const float wheelDiameter = 0.096;   // 96mm

// Motor correction factors
const float motor1Correction = 0.95; // MT1 - right bottom
const float motor2Correction = 1.05; // MT2 - right top
const float motor3Correction = 1.05; // MT3 - left top
const float motor4Correction = 0.95; // MT4 - left bottom

// Mecanum motor channels
const int motor1Channels[] = {8, 9};   // MT1
const int motor2Channels[] = {10, 11}; // MT2
const int motor3Channels[] = {12, 13}; // MT3
const int motor4Channels[] = {14, 15}; // MT4

// Speed levels (25–100%)
int speedLevel = 0;  // 0:25%, 1:50%, 2:75%, 3:100%
const float speedScales[] = {0.25, 0.5, 0.75, 1.0};

// Servo parameters
const int SERVO_MIN_PULSE = 150;           // Min PWM for 180° servo (~0.5ms)
const int SERVO_MAX_PULSE = 650;           // Max PWM for 180° servo (~2.6ms)
const int CONTINUOUS_FORWARD_PULSE = 204;  // PWM for 360° servo forward
const int CONTINUOUS_BACKWARD_PULSE = 409; // PWM for 360° servo backward
const int CONTINUOUS_STOP_PULSE = 0;       // PWM to stop 360° servo
const int SERVO_FREQ = 50;                 // Servo PWM frequency (50Hz)
const int MECANUM_FREQ = 60;               // Mecanum PWM frequency (60Hz)
const int DEADZONE = 30;                   // Joystick deadzone
const int STEP_SIZE = 2;                   // Angle step for 180° servo
const int SPEED_STEP = 1;                  // Speed step for 360° servo
const int MAX_SPEED = 10;                  // Max speed for 360° servo

// Servo configuration
struct ServoConfig {
  int pin;
  int value;
  int minPulse;
  int maxPulse;
  bool isContinuous;
  int minAngle;
  int maxAngle;
};

ServoConfig servos[] = {
  {6, 90, SERVO_MIN_PULSE, SERVO_MAX_PULSE, false, 0, 40},   // Servo 6 (180°, 0-40°)
  {7, 90, SERVO_MIN_PULSE, SERVO_MAX_PULSE, false, 30, 120}, // Servo 7 (180°, 30-120°)
  {4, 0, CONTINUOUS_BACKWARD_PULSE, CONTINUOUS_FORWARD_PULSE, true}, // Servo 4 (360°)
  {5, 0, CONTINUOUS_BACKWARD_PULSE, CONTINUOUS_FORWARD_PULSE, true}  // Servo 5 (360°)
};
const int SERVO_COUNT = 4;

// Task handles and flags
TaskHandle_t mecanumTaskHandle = NULL;
TaskHandle_t servo5TaskHandle = NULL;
bool isTestMode = false;
bool isServo5Held = false;
bool servo4Running = false;

// Shared variables for Mecanum control (protected by volatile)
volatile int volatileSpeedLevel = 0;
volatile float volatileVx = 0;
volatile float volatileVy = 0;
volatile float volatileOmega = 0;

// Set motor speed for Mecanum wheels
void setMotorSpeedSmooth(int motorNum, const int channels[2], float speed) {
  speed = constrain(speed, -1.0, 1.0);
  int pwmValue = abs(speed) * 4095;

  if (abs(speed) < 0.05) {
    // Coast
    pwm_mecanum.setPWM(channels[0], 0, 0);
    pwm_mecanum.setPWM(channels[1], 0, 0);
    return;
  }

  if (speed > 0) {
    pwm_mecanum.setPWM(channels[0], 0, pwmValue);
    pwm_mecanum.setPWM(channels[1], 0, 0);
  } else {
    pwm_mecanum.setPWM(channels[0], 0, 0);
    pwm_mecanum.setPWM(channels[1], 0, pwmValue);
  }
}

// Mecanum drive control
void mecanumDrivePWM(float vx, float vy, float omega, int speedLevel) {
  float L = wheelBase;
  float W = trackWidth;
  float r = (L + W) / 2.0;

  // Invert joystick directions
  vx = -vx;
  omega = -omega;

  // Calculate wheel speeds
  float vMT1 = (vy - vx - omega * r) * motor1Correction;
  float vMT2 = (vy + vx + omega * r) * motor2Correction;
  float vMT3 = (vy - vx + omega * r) * motor3Correction;
  float vMT4 = (vy + vx - omega * r) * motor4Correction;

  // Normalize
  float maxVal = max(max(max(abs(vMT1), abs(vMT2)), abs(vMT3)), abs(vMT4));
  if (maxVal < 1e-5) maxVal = 1.0;
  vMT1 /= maxVal;
  vMT2 /= maxVal;
  vMT3 /= maxVal;
  vMT4 /= maxVal;

  // Fix MT4 direction
  vMT4 = -vMT4;

  // Apply speed scale
  float scale = speedScales[speedLevel];
  vMT1 *= scale;
  vMT2 *= scale;
  vMT3 *= scale;
  vMT4 *= scale;

  // Set PWM
  setMotorSpeedSmooth(1, motor1Channels, vMT1);
  setMotorSpeedSmooth(2, motor2Channels, vMT2);
  setMotorSpeedSmooth(3, motor3Channels, vMT3);
  setMotorSpeedSmooth(4, motor4Channels, vMT4);

  // Debug
  Serial.print("SPD "); Serial.print(scale * 100);
  Serial.print(" | MT1: "); Serial.print(vMT1, 2);
  Serial.print(" MT2: "); Serial.print(vMT2, 2);
  Serial.print(" MT3: "); Serial.print(vMT3, 2);
  Serial.print(" MT4: "); Serial.println(vMT4, 2);
}

// Mecanum control task (runs on separate core)
void mecanumControlTask(void *pvParameters) {
  while (1) {
    if (!isTestMode) {
      mecanumDrivePWM(volatileVx, volatileVy, volatileOmega, volatileSpeedLevel);
    } else {
      mecanumDrivePWM(0, 0, 0, 0); // Stop motors in test mode
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Update continuous servo
void updateContinuousServo(int index, int targetSpeed) {
  ServoConfig &servo = servos[index];
  int currentSpeed = servo.value;
  if (targetSpeed > currentSpeed) {
    currentSpeed = min(currentSpeed + SPEED_STEP, targetSpeed);
  } else if (targetSpeed < currentSpeed) {
    currentSpeed = max(currentSpeed - SPEED_STEP, targetSpeed);
  }
  servo.value = constrain(currentSpeed, -MAX_SPEED, MAX_SPEED);

  int pulse;
  if (servo.value == 0) {
    pulse = CONTINUOUS_STOP_PULSE;
  } else if (servo.value > 0) {
    pulse = CONTINUOUS_FORWARD_PULSE;
  } else {
    pulse = CONTINUOUS_BACKWARD_PULSE;
  }
  pwm_servo.setPWM(servo.pin, 0, pulse);

  Serial.print("Servo ");
  Serial.print(servo.pin);
  Serial.print(" - Direction: ");
  Serial.println(servo.value > 0 ? "Forward" : (servo.value < 0 ? "Backward" : "Stopped"));
}

// Servo 5 task
void servo5Task(void *pvParameters) {
  while (1) {
    if (isServo5Held) {
      pwm_servo.setPWM(5, 0, 348); // Backward pulse
      vTaskDelay(pdMS_TO_TICKS(1000));
      pwm_servo.setPWM(5, 0, CONTINUOUS_STOP_PULSE);
      vTaskDelay(pdMS_TO_TICKS(1000));
      Serial.println("Servo 5 Backward");
    } else {
      pwm_servo.setPWM(5, 0, CONTINUOUS_STOP_PULSE);
      vTaskDelete(NULL);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialize PS2 controller
  int error = -1;
  for (int i = 0; i < 10; i++) {
    delay(1000);
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print(".");
    if (!error) break;
  }
  if (error) {
    Serial.println("PS2 connection failed after 10 attempts");
  } else {
    Serial.println("PS2 controller connected.");
  }

  // Initialize Mecanum PWM
  pwm_mecanum.begin();
  pwm_mecanum.setOscillatorFrequency(27000000);
  pwm_mecanum.setPWMFreq(MECANUM_FREQ);
  Serial.println("Mecanum PWM initialized at 60Hz");

  // Initialize Servo PWM
  pwm_servo.begin();
  pwm_servo.setOscillatorFrequency(27000000);
  pwm_servo.setPWMFreq(SERVO_FREQ);
  Serial.println("Servo PWM initialized at 50Hz");

  // Initialize servos
  for (int i = 0; i < SERVO_COUNT; i++) {
    if (servos[i].isContinuous) {
      pwm_servo.setPWM(servos[i].pin, 0, CONTINUOUS_STOP_PULSE);
    } else {
      pwm_servo.setPWM(servos[i].pin, 0, map(servos[i].value, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
    }
  }

  // Create Mecanum control task on core 0 (assuming loop runs on core 1)
  xTaskCreatePinnedToCore(
    mecanumControlTask,
    "MecanumTask",
    4096,
    NULL,
    1,
    &mecanumTaskHandle,
    0
  );

  Serial.println("READY: Left joystick = move, Right joystick X = rotate, DPad Up/Down = Speed");
}

void loop() {
  ps2x.read_gamepad(false, 0);

  // Toggle test mode
  if (ps2x.ButtonPressed(PSB_L1) && ps2x.ButtonPressed(PSB_L2)) {
    isTestMode = !isTestMode;
    Serial.print("Switched to ");
    Serial.println(isTestMode ? "Test Mode" : "Normal Mode");
    delay(200);
  }

  // Update Mecanum inputs
  if (!isTestMode) {
    if (ps2x.ButtonPressed(PSB_PAD_UP)) {
      speedLevel = min(speedLevel + 1, 3);
      volatileSpeedLevel = speedLevel;
      Serial.print("Speed increased to: ");
      Serial.println(speedScales[speedLevel] * 100);
    } else if (ps2x.ButtonPressed(PSB_PAD_DOWN)) {
      speedLevel = max(speedLevel - 1, 0);
      volatileSpeedLevel = speedLevel;
      Serial.print("Speed decreased to: ");
      Serial.println(speedScales[speedLevel] * 100);
    }

    int lx = ps2x.Analog(PSS_LX);
    int ly = ps2x.Analog(PSS_LY);
    int rx = ps2x.Analog(PSS_RX);

    float vx = map(lx, 0, 255, -100, 100) / 100.0;
    float vy = map(ly, 0, 255, 100, -100) / 100.0;
    float omega = map(rx, 0, 255, -100, 100) / 100.0;

    float deadzone = 0.15;
    if (abs(vx) < deadzone) vx = 0;
    if (abs(vy) < deadzone) vy = 0;
    if (abs(omega) < deadzone) omega = 0;

    float mag = sqrt(vx * vx + vy * vy);
    if (mag > 1) {
      vx /= mag;
      vy /= mag;
    }

    volatileVx = vx;
    volatileVy = vy;
    volatileOmega = omega;
  }

  // Servo control
  if (isTestMode) {
    // Servo 6 control (Left Y)
    int leftY = ps2x.Analog(PSS_LY);
    if (abs(leftY - 128) > DEADZONE) {
      int target = map(leftY, 0, 255, 0, 40);
      int newAngle = servos[0].value;
      if (target > newAngle) {
        newAngle = min(newAngle + STEP_SIZE, servos[0].maxAngle);
      } else if (target < newAngle) {
        newAngle = max(newAngle - STEP_SIZE, servos[0].minAngle);
      }
      servos[0].value = newAngle;
      pwm_servo.setPWM(6, 0, map(servos[0].value, 0, 40, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.print("Servo 6 angle: ");
      Serial.println(servos[0].value);
    }

    // Servo 7 control (Right Y)
    int rightY = ps2x.Analog(PSS_RY);
    if (abs(rightY - 128) > DEADZONE) {
      int target = map(rightY, 0, 255, 30, 120);
      int newAngle = servos[1].value;
      if (target > newAngle) {
        newAngle = min(newAngle + STEP_SIZE, servos[1].maxAngle);
      } else if (target < newAngle) {
        newAngle = max(newAngle - STEP_SIZE, servos[1].minAngle);
      }
      servos[1].value = newAngle;
      pwm_servo.setPWM(7, 0, map(servos[1].value, 30, 120, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.print("Servo 7 angle: ");
      Serial.println(servos[1].value);
    }

    // Servo 4 control (L1/L2)
    if (ps2x.ButtonPressed(PSB_L1)) {
      updateContinuousServo(2, MAX_SPEED);
    } else if (ps2x.ButtonPressed(PSB_L2)) {
      updateContinuousServo(2, -MAX_SPEED);
    } else if (!ps2x.Button(PSB_L1) && !ps2x.Button(PSB_L2) && servos[2].value != 0) {
      updateContinuousServo(2, 0);
    }

    // Servo 5 control (R1/R2)
    if (ps2x.ButtonPressed(PSB_R1)) {
      updateContinuousServo(3, MAX_SPEED);
    } else if (ps2x.ButtonPressed(PSB_R2)) {
      updateContinuousServo(3, -MAX_SPEED);
    } else if (!ps2x.Button(PSB_R1) && !ps2x.Button(PSB_R2) && !isServo5Held && servos[3].value != 0) {
      updateContinuousServo(3, 0);
    }

    // Servo 5 cycle (Triangle)
    if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
      if (!isServo5Held) {
        isServo5Held = true;
        xTaskCreate(servo5Task, "Servo5Task", 2048, NULL, 1, &servo5TaskHandle);
        Serial.println("Servo 5 starting Backward cycle task");
      } else {
        isServo5Held = false;
        if (servo5TaskHandle != NULL) {
          vTaskDelete(servo5TaskHandle);
          servo5TaskHandle = NULL;
        }
        pwm_servo.setPWM(5, 0, CONTINUOUS_STOP_PULSE);
        Serial.println("Servo 5 stopped");
      }
      delay(200);
    }

    // Release servos 6 & 7 (Square)
    if (ps2x.ButtonPressed(PSB_SQUARE)) {
      pwm_servo.setPWM(6, 0, 0);
      pwm_servo.setPWM(7, 0, 0);
      isServo5Held = false;
      if (servo5TaskHandle != NULL) {
        vTaskDelete(servo5TaskHandle);
        servo5TaskHandle = NULL;
      }
      Serial.println("Servo 6 and 7 released");
    }

    // Stop servo 4 (Circle)
    if (ps2x.ButtonPressed(PSB_CIRCLE)) {
      pwm_servo.setPWM(4, 0, CONTINUOUS_STOP_PULSE);
      Serial.println("Servo 4 stopped");
    }
  } else {
    // Servo actions
    if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
      servos[0].value = 5;
      pwm_servo.setPWM(6, 0, map(servos[0].value, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 6 set to 5°");
      delay(500);
      servos[1].value = 110;
      pwm_servo.setPWM(7, 0, map(servos[1].value, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 7 set to 110°");
      delay(500);
      pwm_servo.setPWM(6, 0, 0);
      pwm_servo.setPWM(7, 0, 0);
      Serial.println("Both servos released");
    }

    if (ps2x.ButtonPressed(PSB_CROSS)) {
      servos[0].value = 5;
      pwm_servo.setPWM(6, 0, map(servos[0].value, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 6 set to 5°");
      delay(500);
      servos[1].value = 90;
      pwm_servo.setPWM(7, 0, map(servos[1].value, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 7 set to 90°");
      delay(500);
      servos[0].value = 70;
      pwm_servo.setPWM(6, 0, map(servos[0].value, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 6 set to 70°");
      delay(500);
      pwm_servo.setPWM(6, 0, 0);
      pwm_servo.setPWM(7, 0, 0);
      Serial.println("Both servos released");
    }

    if (ps2x.ButtonPressed(PSB_SQUARE)) {
      servos[0].value = 5;
      pwm_servo.setPWM(6, 0, map(servos[0].value, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 6 set to 5°");
      delay(500);
      servos[1].value = 100;
      pwm_servo.setPWM(7, 0, map(servos[1].value, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 7 set to 100°");
      delay(500);
      servos[0].value = 70;
      pwm_servo.setPWM(6, 0, map(servos[0].value, 0, 180, SERVO_MAX_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 6 set to 70°");
      delay(500);
      isServo5Held = true;
      xTaskCreate(servo5Task, "Servo5Task", 2048, NULL, 1, &servo5TaskHandle);
      Serial.println("Servo 5 starting Backward cycle task");
      delay(500);
      servos[0].value = 10;
      pwm_servo.setPWM(6, 0, map(servos[0].value, 0, 40, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 6 set to 10°");
      delay(500);
      servos[1].value = 40;
      pwm_servo.setPWM(7, 0, map(servos[1].value, 30, 120, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 7 set to 40°");
      delay(500);
      isServo5Held = false;
      if (servo5TaskHandle != NULL) {
        vTaskDelete(servo5TaskHandle);
        servo5TaskHandle = NULL;
      }
      pwm_servo.setPWM(5, 0, 348);
      delay(200);
      pwm_servo.setPWM(5, 0, CONTINUOUS_STOP_PULSE);
      Serial.println("Servo 5 stopped");
      servos[0].value = 23;
      pwm_servo.setPWM(6, 0, map(servos[0].value, 0, 40, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 6 set to 23°");
      delay(500);
    }

    if (ps2x.Button(PSB_CIRCLE)) {
      servos[0].value = 26;
      pwm_servo.setPWM(6, 0, map(servos[0].value, 0, 40, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      pwm_servo.setPWM(4, 0, CONTINUOUS_BACKWARD_PULSE);
      Serial.println("Servo 6 at 26°, Servo 4 BACKWARD");
    } else if (ps2x.ButtonReleased(PSB_CIRCLE)) {
      pwm_servo.setPWM(4, 0, CONTINUOUS_STOP_PULSE);
      servos[0].value = 0;
      pwm_servo.setPWM(6, 0, map(servos[0].value, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 6 set to 0°");
      delay(1000);
      servos[1].value = 110;
      pwm_servo.setPWM(7, 0, map(servos[1].value, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 7 set to 110°");
      delay(500);
      pwm_servo.setPWM(6, 0, 0);
      pwm_servo.setPWM(7, 0, 0);
      Serial.println("All servos released");
    }

    if (ps2x.ButtonPressed(PSB_L1)) {
      servo4Running = !servo4Running;
      if (servo4Running) {
        pwm_servo.setPWM(4, 0, CONTINUOUS_FORWARD_PULSE);
        Serial.println("Servo 4 FORWARD");
      } else {
        pwm_servo.setPWM(4, 0, CONTINUOUS_STOP_PULSE);
        Serial.println("Servo 4 stopped");
      }
      delay(200);
    }

    if (ps2x.ButtonPressed(PSB_R1)) {
      updateContinuousServo(3, MAX_SPEED);
    } else if (ps2x.ButtonPressed(PSB_R2)) {
      updateContinuousServo(3, -MAX_SPEED);
    } else if (!ps2x.Button(PSB_R1) && !ps2x.Button(PSB_R2) && !isServo5Held && servos[3].value != 0) {
      updateContinuousServo(3, 0);
    }
  }

  delay(10);
}