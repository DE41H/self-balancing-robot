#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHRS.h>


class Gyro {
    public:
        Gyro();

        static constexpr int SAMPLE_FREQ_HZ = 100;
        
        bool begin();

        QueueHandle_t getPitchQueue() const { return pitchQueue; }
        QueueHandle_t getYawQueue() const { return yawQueue; }

    private:
        static constexpr int TASK_STACK_SIZE = 4096;
        static constexpr int TASK_PRIORITY = 3;
        static constexpr int TASK_CORE_ID = 0;
        static constexpr int CALIBRATION_SAMPLES = 500;
        static constexpr int CALIBRATION_DELAY_MS = 5;

        Adafruit_MPU6050 mpu;
        Adafruit_Madgwick filter;
        sensors_event_t a, g, temp;
        QueueHandle_t pitchQueue;
        QueueHandle_t yawQueue;
        TaskHandle_t taskHandle;

        struct GyroOffset {
            float x, y, z;
        } offset;

        bool setup();
        void taskLoop();
        void calibrate();
        void update();
        static void taskTrampoline(void *pvParameters);
};
