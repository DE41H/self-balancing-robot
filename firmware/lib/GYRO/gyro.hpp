#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHRS.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <config.hpp>


class Gyro {
    public:
        Gyro();
        
        bool begin();
        QueueHandle_t getPitchQueue() const { return _pitchQueue; }
        QueueHandle_t getYawQueue() const { return _yawQueue; }

    private:
        Adafruit_MPU6050 _mpu;
        Adafruit_Mahony _filter;
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
