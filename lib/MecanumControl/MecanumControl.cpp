#include "MecanumControl.h"

MecanumControl::MecanumControl() {
    if (!pwm.begin()) {
        // Xử lý lỗi khởi tạo PWM (nếu cần, có thể thêm log hoặc hành động khác)
        while (1); // Dừng nếu không khởi tạo được
    }
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(PWM_FREQ);
}

int MecanumControl::mapSpeed(float speed, float rpm) {
    // Tối ưu hóa bằng cách tránh phép chia lặp lại, sử dụng hàm constrain trực tiếp
    return constrain(static_cast<int>((speed + 1.0) * MAX_PULSE / 2), MIN_PULSE, MAX_PULSE);
}

void MecanumControl::setMotor(int motor, float speed, bool direction) {
    static const struct {
        uint8_t channel1, channel2;
    } motorChannels[4] = {
        {8, 9},  // Dòng 1
        {10, 11}, // Dòng 2
        {12, 13}, // Dòng 3
        {14, 15}  // Dòng 4
    };

    if (motor < 1 || motor > 4) return;

    const auto& channels = motorChannels[motor - 1];
    float adjustedSpeed = speed * (direction ? 1.0 : -1.0);
    int pwmValue = mapSpeed(adjustedSpeed, (motor <= 2 ? rpmFront : rpmRear));

    pwm.setPWM(channels.channel1, 0, (adjustedSpeed >= 0) ? pwmValue : 0);
    pwm.setPWM(channels.channel2, 0, (adjustedSpeed < 0) ? pwmValue : 0);
}

void MecanumControl::move(float x, float y, float omega) {
    // Tính toán vector tốc độ cho mỗi bánh, sử dụng hằng số để tối ưu
    const float radius = (wheelBase + trackWidth) / 2.0;
    float v1 = y + x + omega * radius;
    float v2 = y - x - omega * radius;
    float v3 = y - x + omega * radius;
    float v4 = y + x - omega * radius;

    // Chuẩn hóa tốc độ, tránh phép toán lặp
    float maxSpeed = std::max({std::abs(v1), std::abs(v2), std::abs(v3), std::abs(v4)});
    if (maxSpeed > 1.0f) {
        v1 /= maxSpeed; v2 /= maxSpeed; v3 /= maxSpeed; v4 /= maxSpeed;
    }

    // Gọi setMotor với tham số đã chuẩn hóa
    setMotor(1, v1, v1 >= 0);
    setMotor(2, v2, v2 >= 0);
    setMotor(3, v3, v3 >= 0);
    setMotor(4, v4, v4 >= 0);
}

void MecanumControl::moveWithRotation(float x, float y, float omega) {
    move(x, y, omega);
}