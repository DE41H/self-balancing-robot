#include <gyro.hpp>

const TickType_t xLoopPeriod = (1000 / Gyro::SAMPLE_FREQ_HZ) / portTICK_PERIOD_MS;

Gyro::Gyro() {

}

bool Gyro::begin() {
    pitchQueue = xQueueCreate(1, sizeof(float));
    if (pitchQueue == NULL) {
        Serial.println("Failed to create pitch queue!");
        return false;
    }
    yawQueue = xQueueCreate(1, sizeof(float));
    if (yawQueue == NULL) {
        Serial.println("Failed to create yaw queue!");
        vQueueDelete(pitchQueue); 
        return false;
    }

    if (!setup()) {
        Serial.println("IMU Hardware Setup Failed!");
        vQueueDelete(pitchQueue);
        vQueueDelete(yawQueue);
        return false;
    }
    else {
        Serial.println("MPU-6050 Found!");
    }

    BaseType_t taskCreated = xTaskCreatePinnedToCore(
        taskTrampoline,
        "Gyro Task",
        TASK_STACK_SIZE,
        this,
        TASK_PRIORITY,
        &taskHandle,
        TASK_CORE_ID
    );
    if (taskCreated != pdPASS) {
        Serial.println("Failed to create gyro task!");
        vQueueDelete(pitchQueue);
        vQueueDelete(yawQueue);
        return false;
    }

    return true;
}

QueueHandle_t Gyro::getPitchQueue() const {
    return pitchQueue;
}

QueueHandle_t Gyro::getYawQueue() const {
    return yawQueue;
}

void Gyro::update() {
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

void Gyro::calibrate() {
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

bool Gyro::setup() {
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

void Gyro::taskLoop() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        update();
        xTaskDelayUntil(&xLastWakeTime, xLoopPeriod);
    }
}

void Gyro::taskTrampoline(void* pvParameters) {
    Gyro* instance = static_cast<Gyro*>(pvParameters);
    
    instance->taskLoop();

    vTaskDelete(NULL);
}