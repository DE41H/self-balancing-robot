#include <gyro.hpp>

#define SAMPLE_FREQ_HZ 100
const TickType_t xLoopPeriod = (1000 / SAMPLE_FREQ_HZ) / portTICK_PERIOD_MS;

gyro::gyro() {

}

bool gyro::begin() {
    pitchQueue = xQueueCreate(1, sizeof(float));
    yawQueue = xQueueCreate(1, sizeof(float));

    if (!setup()) {
        Serial.println("IMU Hardware Setup Failed!");
        return false;
    }
    else {
        Serial.println("MPU-6050 Found!");
    }

    xTaskCreatePinnedToCore(
        taskTrampoline,
        "Gyro Task",
        4096,
        this,
        3,
        &taskHandle,
        0
    );

    return true;
}

QueueHandle_t gyro::getPitchQueue() const {
    return pitchQueue;
}

QueueHandle_t gyro::getYawQueue() const {
    return yawQueue;
}

void gyro::update() {
    mpu.getEvent(&a, &g, &temp);
    filter.updateIMU(
        g.gyro.x - offset.x,
        g.gyro.y - offset.y,
        g.gyro.z - offset.z,
        a.acceleration.x,
        a.acceleration.y,
        a.acceleration.z
    );
    
    float currentPitch = filter.getPitch();
    float currentYaw = filter.getYaw();
    
    xQueueOverwrite(pitchQueue, &currentPitch);
    xQueueOverwrite(yawQueue, &currentYaw);
}

void gyro::calibrate() {
    const int samples = 500;
    float sumX = 0, sumY = 0, sumZ = 0;
    
    for (int i = 0; i < samples; i++) {
        mpu.getEvent(&a, &g, &temp);
        sumX += g.gyro.x;
        sumY += g.gyro.y;
        sumZ += g.gyro.z;
        delay(5);
    }
    
    offset.x = sumX / samples;
    offset.y = sumY / samples;
    offset.z = sumZ / samples;
}

bool gyro::setup() {
    Wire.begin();

    if (!mpu.begin()) {
        return false;
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    offset = {0, 0, 0};
    calibrate();

    filter.begin(SAMPLE_FREQ_HZ);

    return true;
}

void gyro::taskLoop() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        update();
        xTaskDelayUntil(&xLastWakeTime, xLoopPeriod);
    }
}

void gyro::taskTrampoline(void* pvParameters) {
    gyro* instance = static_cast<gyro*>(pvParameters);
    
    instance->taskLoop();

    vTaskDelete(NULL);
}