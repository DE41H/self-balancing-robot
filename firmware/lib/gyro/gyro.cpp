#include <IMU.hpp>

#define SAMPLE_FREQ_HZ 100
const TickType_t xLoopPeriod = (1000 / SAMPLE_FREQ_HZ) / portTICK_PERIOD_MS;

IMU::IMU() {

}

bool IMU::begin() {
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
        "IMU Task",
        4096,
        this,
        3,
        &taskHandle,
        0
    );

    return true;
}

QueueHandle_t IMU::getPitchQueue() const {
    return pitchQueue;
}

QueueHandle_t IMU::getYawQueue() const {
    return yawQueue;
}

void IMU::update() {
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

void IMU::calibrate() {
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

bool IMU::setup() {
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

void IMU::taskLoop() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        update();
        xTaskDelayUntil(&xLastWakeTime, xLoopPeriod);
    }
}

void IMU::taskTrampoline(void* pvParameters) {
    IMU* instance = static_cast<IMU*>(pvParameters);
    
    instance->taskLoop();

    vTaskDelete(NULL);
}