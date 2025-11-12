#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHRS.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"


class Gyro {
    public:
        Gyro();

        static constexpr int SAMPLE_FREQ_HZ = 100;
        
        bool begin();

        QueueHandle_t getPitchQueue() const { return _pitchQueue; }
        QueueHandle_t getYawQueue() const { return _yawQueue; }

    private:
        static constexpr int TASK_STACK_SIZE = 4096;
        static constexpr int TASK_PRIORITY = 10;
        static constexpr int TASK_CORE_ID = 0;
        static constexpr int CALIBRATION_SAMPLES = 500;
        static constexpr int CALIBRATION_DELAY_MS = 5;
        static constexpr mpu6050_accel_range_t ACCEL_RANGE = MPU6050_RANGE_8_G;
        static constexpr mpu6050_gyro_range_t GYRO_RANGE = MPU6050_RANGE_500_DEG;
        static constexpr mpu6050_bandwidth_t FILTER_BAND = MPU6050_BAND_44_HZ;

        Adafruit_MPU6050 _mpu;
        Adafruit_Madgwick _filter;
        sensors_event_t _a, _g, _temp;
        QueueHandle_t _pitchQueue;
        QueueHandle_t _yawQueue;
        TaskHandle_t _taskHandle;

        struct GyroOffset {
            float x, y, z;
        } _offset;

        bool setup();
        void taskLoop();
        void calibrate();
        void update();
        static void taskTrampoline(void *pvParameters);
};
