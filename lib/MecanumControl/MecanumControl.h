#ifndef MECANUM_CONTROL_H
#define MECANUM_CONTROL_H

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class MecanumControl {
private:
    Adafruit_PWMServoDriver pwm;
    float wheelBase = 0.18; // 18cm
    float trackWidth = 0.26; // 26cm
    float wheelDiameter = 0.096; // 96mm
    float rpmFront = 300.0;
    float rpmRear = 319.0;
    const int MIN_PULSE = 0;
    const int MAX_PULSE = 4096;
    const int PWM_FREQ = 50;

    int mapSpeed(float speed, float rpm);

public:
    MecanumControl();
    void setMotor(int motor, float speed, bool direction);
    void move(float x, float y, float omega);
    void moveWithRotation(float x, float y, float omega);
};

#endif