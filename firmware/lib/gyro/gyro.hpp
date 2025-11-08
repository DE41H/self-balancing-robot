#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHRS.h>

class gyro {
    public:
        gyro();
        
        bool begin();
        QueueHandle_t getPitchQueue() const;
        QueueHandle_t getYawQueue() const;

    private:
        Adafruit_MPU6050 mpu;
        Adafruit_Madgwick filter;
        sensors_event_t a, g, temp;

        QueueHandle_t pitchQueue;
        QueueHandle_t yawQueue;
        TaskHandle_t taskHandle;

        struct GyroOffset {
            float x, y, z;
        } offset;
        unsigned long lastUpdateTime;

        bool setup();
        void taskLoop();
        void calibrate();
        void update();
        static void taskTrampoline(void *pvParameters);
};