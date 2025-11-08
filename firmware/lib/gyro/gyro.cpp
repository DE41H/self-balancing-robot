#include <gyro.hpp>

const TickType_t xLoopPeriod = (1000 / gyro::SAMPLE_FREQ_HZ) / portTICK_PERIOD_MS;

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
        TASK_STACK_SIZE,
        this,
        TASK_PRIORITY,
        &taskHandle,
        TASK_CORE_ID
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
    float sumX = 0, sumY = 0, sumZ = 0;
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        mpu.getEvent(&a, &g, &temp);
        sumX += g.gyro.x;
        sumY += g.gyro.y;
        sumZ += g.gyro.z;
        delay(CALIBRATION_DELAY_MS);
    }
    
    offset.x = sumX / CALIBRATION_SAMPLES;
    offset.y = sumY / CALIBRATION_SAMPLES;
    offset.z = sumZ / CALIBRATION_SAMPLES;
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
    Serial.println("Calibrating IMU...");
    calibrate();
    Serial.println("Finished Calibrating IMU");

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