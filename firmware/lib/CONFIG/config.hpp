#pragma once
#include <driver/pcnt.h>


namespace Config {
    static constexpr int SAMPLE_FREQ_HZ = 250;
    static constexpr int SAMPLE_TIME = 1000 / SAMPLE_FREQ_HZ;

    static constexpr mpu6050_accel_range_t ACCEL_RANGE = MPU6050_RANGE_8_G;
    static constexpr mpu6050_gyro_range_t GYRO_RANGE = MPU6050_RANGE_500_DEG;
    static constexpr mpu6050_bandwidth_t FILTER_BAND = MPU6050_BAND_44_HZ;

    static constexpr int GYRO_TASK_STACK_SIZE = 4096;
    static constexpr byte GYRO_TASK_PRIORITY = 10;
    static constexpr byte GYRO_TASK_CORE_ID = 0;
    static constexpr int GYRO_CALIBRATION_SAMPLES = 500;
    static constexpr int GYRO_CALIBRATION_DELAY_MS = 5;
    static constexpr int WIRE_CLOCK = 400000;
    static constexpr byte SDA_PIN = 0;
    static constexpr byte SCL_PIN = 0;

    static constexpr byte MOTOR_A_PWM_PIN = 0;
    static constexpr byte MOTOR_A_IN1_PIN = 0;
    static constexpr byte MOTOR_A_IN2_PIN = 0;
    static constexpr byte MOTOR_A_ENCA_PIN = 0;
    static constexpr byte MOTOR_A_ENCB_PIN = 0;
    static constexpr byte MOTOR_A_PWM_CHAN = 0;
    static constexpr pcnt_unit_t MOTOR_A_PCNT_UNIT = PCNT_UNIT_0;

    static constexpr byte MOTOR_B_PWM_PIN = 0;
    static constexpr byte MOTOR_B_IN1_PIN = 0;
    static constexpr byte MOTOR_B_IN2_PIN = 0;
    static constexpr byte MOTOR_B_ENCA_PIN = 0;
    static constexpr byte MOTOR_B_ENCB_PIN = 0;
    static constexpr byte MOTOR_B_PWM_CHAN = 0;
    static constexpr pcnt_unit_t MOTOR_B_PCNT_UNIT = PCNT_UNIT_1;

    static constexpr int PWM_FREQUENCY = 20000;
    static constexpr byte PWM_RESOLUTION = 8;
    static constexpr int PWM_LIMIT = (1 << PWM_RESOLUTION) - 1;
    static constexpr byte STBY_PIN = 0;
    static constexpr int ENCODER_PPR = 20;
    static constexpr float RPM_FACTOR = (60.0 * Config::SAMPLE_FREQ_HZ) / ENCODER_PPR;
    static constexpr byte PCNT_FILTER_VALUE = 100;

    static constexpr float SPEED_KP = 0.0;
    static constexpr float SPEED_KI = 0.0;
    static constexpr float SPEED_KD = 0.0;

    static constexpr float BALANCE_KP = 0.0;
    static constexpr float BALANCE_KI = 0.0;
    static constexpr float BALANCE_KD = 0.0;
    static constexpr float BALANCE_MIN = 0.0;
    static constexpr float BALANCE_MAX = 0.0;

    static constexpr float TURN_KP = 0.0;
    static constexpr float TURN_KI = 0.0;
    static constexpr float TURN_KD = 0.0;
    static constexpr float TURN_MIN = 0.0;
    static constexpr float TURN_MAX = 0.0;

    static constexpr int MAIN_TASK_STACK_SIZE = 4096;
    static constexpr int MAIN_TASK_PRIORITY = 0;
    static constexpr int MAIN_TASK_CORE_ID = 0;
};