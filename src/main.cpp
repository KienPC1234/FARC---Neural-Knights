#include <Arduino.h>
#include <PS2X_lib.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PS2_DAT 12 // MISO
#define PS2_CMD 13 // MOSI
#define PS2_SEL 15 // SS
#define PS2_CLK 14 // SLK

#define pressures false
#define rumble false

PS2X ps2x;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int SERVO_MIN_PULSE = 150;           // PWM tối thiểu cho servo 180° (~0.5ms)
const int SERVO_MAX_PULSE = 650;           // PWM tối đa cho servo 180° (~2.6ms)
const int CONTINUOUS_FORWARD_PULSE = 204;  // Giá trị PWM để servo 360° quay thuận (đã hiệu chỉnh)
const int CONTINUOUS_BACKWARD_PULSE = 409; // Giá trị PWM để servo 360° quay ngược (đã hiệu chỉnh)
const int CONTINUOUS_STOP_PULSE = 307;     // Giá trị PWM để dừng servo 360° (trung điểm)
const int SERVO_FREQ = 50;                 // Tần số PWM cho servo (50Hz = 20ms chu kỳ)
const int DEADZONE = 30;                   // Vùng chết của joystick để giảm nhiễu
const int STEP_SIZE = 2;                   // Bước thay đổi góc cho servo 180°
const int SPEED_STEP = 1;                  // Bước thay đổi tốc độ cho servo 360°
const int MAX_SPEED = 10;                  // Tốc độ tối đa cho servo 360°

struct ServoConfig {
  int pin;
  int value;
  int minPulse;
  int maxPulse;
  bool isContinuous;
  int minAngle; // Góc tối thiểu
  int maxAngle; // Góc tối đa
};

ServoConfig servos[] = {
    {6, 90, SERVO_MIN_PULSE, SERVO_MAX_PULSE, false, 0, 40},           // Servo 6 (180°, giới hạn 0-40°)
    {7, 90, SERVO_MIN_PULSE, SERVO_MAX_PULSE, false, 30, 120},         // Servo 7 (180°, giới hạn 30-120°)
    {4, 0, CONTINUOUS_BACKWARD_PULSE, CONTINUOUS_FORWARD_PULSE, true}, // Servo 4 (360°)
    {5, 0, CONTINUOUS_BACKWARD_PULSE, CONTINUOUS_FORWARD_PULSE, true}  // Servo 5 (360°)
};
const int SERVO_COUNT = 4;

bool isTestMode = false;
bool isServo5Held = false; // Cờ để theo dõi trạng thái giữ servo 5
int heldPulse = 0;         // Lưu giá trị xung khi giữ servo 5
bool servo4Running = false; // Cờ để toggle trạng thái servo 4 (L1)

TaskHandle_t servo5TaskHandle = NULL;

void updateContinuousServo(int index, int targetSpeed) {
  ServoConfig &servo = servos[index];
  int currentSpeed = servo.value;
  if (targetSpeed > currentSpeed) {
    currentSpeed = min(currentSpeed + SPEED_STEP, targetSpeed);
  } else if (targetSpeed < currentSpeed) {
    currentSpeed = max(currentSpeed - STEP_SIZE, targetSpeed);
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
  pwm.setPWM(servo.pin, 0, pulse);

  Serial.print("Servo ");
  Serial.print(servo.pin);
  Serial.print(" - Direction: ");
  Serial.println(servo.value > 0 ? "Forward" : (servo.value < 0 ? "Backward" : "Stopped"));
}

void servo5Task(void *pvParameters) {
  while (1) {
    if (isServo5Held) {
      pwm.setPWM(5, 0, 348);                   // Quay Backward với xung 348 (giá trị tùy chỉnh)
      vTaskDelay(pdMS_TO_TICKS(1500));         // Chờ 1.5 giây
      pwm.setPWM(5, 0, CONTINUOUS_STOP_PULSE); // Dừng servo 5
      vTaskDelay(pdMS_TO_TICKS(1000));         // Chờ 1 giây
      Serial.println("Servo 5 Backward");
    } else {
      pwm.setPWM(5, 0, CONTINUOUS_STOP_PULSE);
      vTaskDelete(NULL);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Delay nhỏ để tránh chiếm CPU
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  int error = -1;
  for (int i = 0; i < 10; i++) {
    delay(1000);
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print(".");
    if (!error) break;
  }
  if (error) Serial.println("PS2 connection failed after 10 attempts");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000); // Tần số dao động PWM (27MHz)
  pwm.setPWMFreq(SERVO_FREQ);
}

void loop() {
  ps2x.read_gamepad(false, 0);

  if (ps2x.ButtonPressed(PSB_L1) && ps2x.ButtonPressed(PSB_L2)) {
    isTestMode = !isTestMode;
    Serial.print("Switched to ");
    Serial.println(isTestMode ? "Test Mode" : "Normal Mode");
    delay(200);
  }

  if (isTestMode) {
    int leftY = ps2x.Analog(PSS_LY);
    if (abs(leftY - 128) > DEADZONE) {
      int target = map(leftY, 0, 255, 0, 40);
      int newAngle = servos[0].value;
      if (target > newAngle) newAngle = min(newAngle + STEP_SIZE, servos[0].maxAngle);
      else if (target < newAngle) newAngle = max(newAngle - STEP_SIZE, servos[0].minAngle);
      servos[0].value = newAngle;
      pwm.setPWM(6, 0, map(servos[0].value, 0, 40, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.print("Servo 6 angle: ");
      Serial.println(servos[0].value);
    }

    int rightY = ps2x.Analog(PSS_RY);
    if (abs(rightY - 128) > DEADZONE) {
      int target = map(rightY, 0, 255, 30, 120);
      int newAngle = servos[1].value;
      if (target > newAngle) newAngle = min(newAngle + STEP_SIZE, servos[1].maxAngle);
      else if (target < newAngle) newAngle = max(newAngle - STEP_SIZE, servos[1].minAngle);
      servos[1].value = newAngle;
      pwm.setPWM(7, 0, map(servos[1].value, 30, 120, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.print("Servo 7 angle: ");
      Serial.println(servos[1].value);
    }

    if (ps2x.ButtonPressed(PSB_L1)) {
      updateContinuousServo(2, MAX_SPEED);
    } else if (ps2x.ButtonPressed(PSB_L2)) {
      updateContinuousServo(2, -MAX_SPEED);
    } else if (!ps2x.Button(PSB_L1) && !ps2x.Button(PSB_L2) && servos[2].value != 0) {
      updateContinuousServo(2, 0);
    }

    if (ps2x.ButtonPressed(PSB_R1)) {
      updateContinuousServo(3, MAX_SPEED);
    } else if (ps2x.ButtonPressed(PSB_R2)) {
      updateContinuousServo(3, -MAX_SPEED);
    } else if (!ps2x.Button(PSB_R1) && !ps2x.Button(PSB_R2) && !isServo5Held && servos[3].value != 0) {
      updateContinuousServo(3, 0);
    }

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
        pwm.setPWM(5, 0, CONTINUOUS_STOP_PULSE);
        Serial.println("Servo 5 stopped");
      }
      delay(200);
    }

    if (ps2x.ButtonPressed(PSB_SQUARE)) {
      pwm.setPWM(6, 0, 0);
      pwm.setPWM(7, 0, 0);
      isServo5Held = false;
      if (servo5TaskHandle != NULL) {
        vTaskDelete(servo5TaskHandle);
        servo5TaskHandle = NULL;
      }
      Serial.println("Servo 6 and 7 released");
    }
    if (ps2x.ButtonPressed(PSB_CIRCLE)) {
      pwm.setPWM(4, 0, 0);
      Serial.println("Servo 4 stopped");
    }
  } else {
    if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
      servos[0].value = 5;
      pwm.setPWM(6, 0, map(servos[0].value, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 6 set to 5°");
      delay(500);
      servos[1].value = 110;
      pwm.setPWM(7, 0, map(servos[1].value, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 7 set to 110°");
      delay(500);
      pwm.setPWM(6, 0, 0);
      pwm.setPWM(7, 0, 0);
      Serial.println("Both servos released");
    }
    if (ps2x.ButtonPressed(PSB_CROSS)) {
      servos[0].value = 5;
      pwm.setPWM(6, 0, map(servos[0].value, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 6 set to 5°");
      delay(500);
      servos[1].value = 100;
      pwm.setPWM(7, 0, map(servos[1].value, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 7 set to 100°");
      delay(500);
      servos[0].value = 70;
      pwm.setPWM(6, 0, map(servos[0].value, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 6 set to 70°");
      delay(500);
      pwm.setPWM(6, 0, 0);
      pwm.setPWM(7, 0, 0);
      Serial.println("Both servos released");
    }
    if (ps2x.ButtonPressed(PSB_SQUARE)) {
      servos[0].value = 5;
      pwm.setPWM(6, 0, map(servos[0].value, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 6 set to 5°");
      delay(500);
      servos[1].value = 100;
      pwm.setPWM(7, 0, map(servos[1].value, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 7 set to 100°");
      delay(500);
      servos[0].value = 70;
      pwm.setPWM(6, 0, map(servos[0].value, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 6 set to 70°");
      delay(500);
      isServo5Held = true;
      xTaskCreate(servo5Task, "Servo5Task", 2048, NULL, 1, &servo5TaskHandle);
      Serial.println("Servo 5 starting Backward cycle task");
      delay(500);
      servos[0].value = 12;
      pwm.setPWM(6, 0, map(servos[0].value, 0, 40, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 6 set to 12°");
      delay(500);
      servos[1].value = 50;
      pwm.setPWM(7, 0, map(servos[1].value, 30, 120, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 7 set to 50°");
      delay(500);
      isServo5Held = false;
      if (servo5TaskHandle != NULL) {
        vTaskDelete(servo5TaskHandle);
        servo5TaskHandle = NULL;
      }
      pwm.setPWM(5, 0, 348); // Quay Backward với xung 348 (giá trị tùy chỉnh)
      delay(500);
      pwm.setPWM(5, 0, CONTINUOUS_STOP_PULSE);
      Serial.println("Servo 5 stopped");
      servos[0].value = 23;
      pwm.setPWM(6, 0, map(servos[0].value, 0, 40, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 6 set to 23°");
      delay(500);
    }
    if (ps2x.Button(PSB_CIRCLE)) { // Nhấn giữ nút tròn
      servos[0].value = 30;
      pwm.setPWM(6, 0, map(servos[0].value, 0, 40, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      pwm.setPWM(4, 0, CONTINUOUS_BACKWARD_PULSE); // Servo 4 quay thuận
      Serial.println("Servo 6 at 30°, Servo 4 BACKWARD");
    } else if (ps2x.ButtonReleased(PSB_CIRCLE)) { // Thả nút tròn
      pwm.setPWM(4, 0, CONTINUOUS_STOP_PULSE); // Dừng servo 4
      servos[0].value = 5;
      pwm.setPWM(6, 0, map(servos[0].value, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 6 set to 5°");
      delay(500);
      servos[1].value = 110;
      pwm.setPWM(7, 0, map(servos[1].value, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println("Servo 7 set to 110°");
      delay(500);
      pwm.setPWM(6, 0, 0);
      pwm.setPWM(7, 0, 0);
      Serial.println("Both servos released");
    }
    if (ps2x.ButtonPressed(PSB_L1)) { // Nhấn L1 để toggle servo 4
      servo4Running = !servo4Running;
      if (servo4Running) {
        pwm.setPWM(4, 0, CONTINUOUS_FORWARD_PULSE); // Quay ngược
        Serial.println("Servo 4 FORWARD");
      } else {
        pwm.setPWM(4, 0, CONTINUOUS_STOP_PULSE); // Dừng
        Serial.println("Servo 4 stopped");
      }
      delay(200); // Tránh chập nút
    }
    if (ps2x.ButtonPressed(PSB_R1)) {
      updateContinuousServo(3, -MAX_SPEED); // Servo 5 quay ngược
    } else if (ps2x.ButtonPressed(PSB_R2)) {
      updateContinuousServo(3, MAX_SPEED); // Servo 5 quay thuận
    } else if (!ps2x.Button(PSB_R1) && !ps2x.Button(PSB_R2) && !isServo5Held && servos[3].value != 0) {
      updateContinuousServo(3, 0); // Dừng servo 5 nếu không nhấn
    }
  }
  delay(100);
}