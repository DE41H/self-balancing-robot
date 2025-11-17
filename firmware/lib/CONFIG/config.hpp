#pragma once
#include <Adafruit_MPU6050.h>


namespace Config {
    static constexpr unsigned int SAMPLE_FREQ_HZ = 150;
    static constexpr unsigned int SAMPLE_TIME = 1000 / SAMPLE_FREQ_HZ;

    static constexpr mpu6050_accel_range_t ACCEL_RANGE = MPU6050_RANGE_8_G;
    static constexpr mpu6050_gyro_range_t GYRO_RANGE = MPU6050_RANGE_500_DEG;
    static constexpr mpu6050_bandwidth_t FILTER_BAND = MPU6050_BAND_44_HZ;

    static constexpr unsigned int GYRO_TASK_STACK_SIZE = 4096;
    static constexpr unsigned int GYRO_TASK_PRIORITY = 10;
    static constexpr unsigned int GYRO_TASK_CORE_ID = 0;
    static constexpr unsigned int GYRO_CALIBRATION_SAMPLES = 500;
    static constexpr unsigned int GYRO_CALIBRATION_DELAY_MS = 5;
    static constexpr unsigned int WIRE_CLOCK = 400000;
    static constexpr unsigned int SDA_PIN = 0;
    static constexpr unsigned int SCL_PIN = 0;

    static constexpr unsigned int MOTOR_A_PWM_PIN = 0;
    static constexpr unsigned int MOTOR_A_IN1_PIN = 0;
    static constexpr unsigned int MOTOR_A_IN2_PIN = 0;
    static constexpr unsigned int MOTOR_A_ENC_PIN = 0;
    static constexpr unsigned int MOTOR_A_PWM_CHAN = 0;

    static constexpr unsigned int MOTOR_B_PWM_PIN = 0;
    static constexpr unsigned int MOTOR_B_IN1_PIN = 0;
    static constexpr unsigned int MOTOR_B_IN2_PIN = 0;
    static constexpr unsigned int MOTOR_B_ENC_PIN = 0;
    static constexpr unsigned int MOTOR_B_PWM_CHAN = 0;

    static constexpr unsigned int PWM_FREQUENCY = 20000;
    static constexpr unsigned int PWM_RESOLUTION = 8;
    static constexpr unsigned int PWM_LIMIT = (1 << PWM_RESOLUTION) - 1;
    static constexpr unsigned int STBY_PIN = 0;
    static constexpr unsigned int ENCODER_PPR = 20;
    static constexpr float RPM_FACTOR = (60.0f * Config::SAMPLE_FREQ_HZ) / ENCODER_PPR;

    static constexpr float SPEED_KP = 0.0f;
    static constexpr float SPEED_KI = 0.0f;
    static constexpr float SPEED_KD = 0.0f;
    static constexpr float SPEED_MIN = 0.0f;
    static constexpr float SPEED_MAX = 0.0f;
    static constexpr float SPEED_ALPHA = 0.95f;

    static constexpr float BALANCE_KP = 0.0f;
    static constexpr float BALANCE_KI = 0.0f;
    static constexpr float BALANCE_KD = 0.0f;
    static constexpr float BALANCE_MIN = 0.0f;
    static constexpr float BALANCE_MAX = 0.0f;

    static constexpr float TURN_KP = 0.0f;
    static constexpr float TURN_KI = 0.0f;
    static constexpr float TURN_KD = 0.0f;
    static constexpr float TURN_MIN = 0.0f;
    static constexpr float TURN_MAX = 0.0f;

    static constexpr unsigned int MAIN_TASK_STACK_SIZE = 4096;
    static constexpr unsigned int MAIN_TASK_PRIORITY = 5;
    static constexpr unsigned int MAIN_TASK_CORE_ID = 0;

    static constexpr float DEADMAN_ANGLE = 70.0f;
    static constexpr unsigned int MAX_WAIT_MS = 20;
};