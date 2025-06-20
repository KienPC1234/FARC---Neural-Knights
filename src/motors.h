#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>
#include <algorithm>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


// PS2 Controller pins
#define PS2_DAT 12 // MISO
#define PS2_CMD 13 // MOSI
#define PS2_SEL 15 // SS
#define PS2_CLK 14 // SCLK
#define PS2_pressures false
#define PS2_rumble false


const int SERVO_FREQ = 50;                 // Servo PWM frequency (50Hz)
const int MECANUM_FREQ = 60;               // Mecanum PWM frequency (60Hz)


class MecanumHub;
class MecanumMotor;


enum MecanumPosition {
    MECANUM_RREAR, MECANUM_RFRONT,
    MECANUM_LFRONT, MECANUM_LREAR
};

struct MecanumChannels {
    int forward, backward;
};

struct MecanumMotorConfig {
    MecanumChannels channels;
    const float correction_rate;
};

struct Controller {
    int speedLevel = 0;
    static float* speedScales;

    int volatileVx, volatileVy, volatileOmega;

    // The controller
    PS2X ps2x;

    byte config(uint8_t clk, uint8_t cmd, uint8_t att, uint8_t dat, bool pressures, bool rumble) {
        return ps2x.config_gamepad(clk, cmd, att, dat, pressures, rumble);
    }

    void tick(){
        if (ps2x.ButtonPressed(PSB_PAD_UP)) {
            speedLevel = min(speedLevel + 1, 3);
        } else if (ps2x.ButtonPressed(PSB_PAD_DOWN)) {
            speedLevel = max(speedLevel - 1, 0);
        }

        int lx = ps2x.Analog(PSS_LX);
        int ly = ps2x.Analog(PSS_LY);
        int rx = ps2x.Analog(PSS_RX);

        float vx = (lx / 255 - 0.5) * 2;
        float vy = (ly / 255 - 0.5) * 2;
        float omega = (rx / 255 - 0.5) * 2;

        float deadzone = 0.15;
        if (abs(vx) < deadzone) vx = 0;
        if (abs(vy) < deadzone) vy = 0;
        if (abs(omega) < deadzone) omega = 0;

        float mag = sqrt(vx * vx + vy * vy);
        if (mag > 1) {
            vx /= mag;
            vy /= mag;
        }

        // Invert joystick directions
        vx = -vx;
        omega = -omega;

        volatileVx = vx;
        volatileVy = vy;
        volatileOmega = omega;
    }
};


class MecanumRobot {
protected:
    static void mecanumControlTask(void *pvParameters) {
        MecanumRobot* robot = static_cast<MecanumRobot*>(pvParameters);
        while (1) {
            if (!robot->isTestMode) {
                robot->mecanumHub->update();
            } else {
                // Stop motors in test mode
                robot->mecanumHub->stop_all();
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }


    bool isTestMode;

public: 
    // Robot geometry for Mecanum wheels
    const float wheelBase;
    const float trackWidth;
    const float wheelDiameter;

    Adafruit_PWMServoDriver pwm_mecanum = Adafruit_PWMServoDriver(0x40);
    Adafruit_PWMServoDriver pwm_servo = Adafruit_PWMServoDriver(0x40);

    Controller ctrl;
    MecanumHub* mecanumHub;
    TaskHandle_t mecanumTaskHandle = NULL;

    MecanumRobot(
        const float wheelBase,
        const float trackWidth,
        const float wheelDiameter
    ) : 
        wheelBase(wheelBase),
        trackWidth(trackWidth),
        wheelDiameter(wheelDiameter),
        mecanumHub(new MecanumHub(this))
    {}

    ~MecanumRobot() {
        delete mecanumHub;
    }


    void setup(){
        if (!mecanumHub->is_fully_setup()) {
            Serial.println("Not all mecanum wheels are set up");
        }

        // Initialize PS2 controller
        int error = -1;
        for (int i = 0; i < 10; i++) {
            delay(1000);
            error = ctrl.config(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, PS2_pressures, PS2_rumble);
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

        // Create Mecanum control task on core 0 (assuming loop runs on core 1)
        xTaskCreatePinnedToCore(
            mecanumControlTask,
            "MecanumTask",
            4096,
            this,
            1,
            &mecanumTaskHandle,
            0
        );
    }

    void tick(){
        ctrl.tick();
        mecanumHub->update();
    }
};


int speedtoPWM(float speed){
    return abs(constrain(speed, -1.0, 1.0)) * 4095;
}


class MecanumHub {
protected:
    MecanumRobot* robot;
    MecanumMotor motors[4];
    int ready_mecanum_motors = 0;

public:

    MecanumHub(MecanumRobot* robot) : robot(robot) {}

    void setup_motor(MecanumPosition pos, MecanumMotorConfig config) {
        if (!motors[pos].ready){
            ++ready_mecanum_motors;
        }

        motors[pos].setup(
            &robot->pwm_mecanum,
            pos,
            config
        );
    }

    bool is_fully_setup(){
        return ready_mecanum_motors == 4;
    }

    void stop_all(){
        for (int i = 0; i < 4; ++i){
            motors[i].release();
        }
    }

    void update(){
        float vMT[4];

        for (int i = 0; i < 4; ++i){
            vMT[i] = calcSpeed(i);
        }

        const float maxVal = max(*std::max_element(vMT, vMT + 4), (float)1.0);
        
        for (int i = 0; i < 4; ++i) {
            motors[i].setSpeed(vMT[i] / maxVal);
        }
    }

    float calcSpeed(int index) {
        const MecanumMotor& motor = motors[index];
        Controller& ctrl = robot->ctrl;

        int vx = ctrl.volatileVx,
            vy = ctrl.volatileVy,
            omega = ctrl.volatileOmega;
        const float r = (robot->trackWidth + robot->wheelBase) / 2.0;
        
        int speedScale = ctrl.speedScales[ctrl.speedLevel];
        int factor = speedScale * motor.correction;

        switch (motor.wheelPosition) {
            case MECANUM_RREAR:
                return (vy - vx - omega * r) * factor;
            case MECANUM_RFRONT:
                return (vy + vx + omega * r) * factor;
            case MECANUM_LFRONT:
                return (vy - vx + omega * r) * factor;
            case MECANUM_LREAR:
                // negating tp fix motor 4 direction
                return -(vy + vx - omega * r) * factor;
        }
    }
};

class MecanumMotor {
public:
    static const int MECANUM_FREQ = 60;
    static const float effective_abs_speed_threshold = 0.05;

    bool ready = false;

    float correction;
    MecanumPosition wheelPosition;
    MecanumChannels channels;

    void setup(
        Adafruit_PWMServoDriver* pwm_driver,
        MecanumPosition pos,
        MecanumMotorConfig config
    ) {
        pwm_driver = pwm_driver;
        wheelPosition = pos;
        correction = config.correction_rate;
        channels = config.channels;

        ready = true;
    }

protected:
    Adafruit_PWMServoDriver* pwm_driver;

public:
    void forward(float speed){
        pwm_driver->setPWM(channels.forward, 0, speedtoPWM(speed));
        pwm_driver->setPWM(channels.backward, 0, 0);
    }
    void backward(float speed){
        pwm_driver->setPWM(channels.forward, 0, 0);
        pwm_driver->setPWM(channels.backward, 0, speedtoPWM(speed));
    }
    void release(){
        pwm_driver->setPWM(channels.forward, 0, 0);
        pwm_driver->setPWM(channels.backward, 0, 0);
    }

    void setSpeed(float speed){ 
        speed *= correction;

        if (abs(speed) < MecanumMotor::effective_abs_speed_threshold) {
            release();
        }else if (speed < 0) {
            backward(speed);
        }else{
            forward(speed);
        }
    }
};