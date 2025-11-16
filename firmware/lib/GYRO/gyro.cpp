#include <gyro.hpp>

Gyro::Gyro():
_offset({0, 0, 0})
{

}

bool Gyro::begin() {
    _pitchQueue = xQueueCreate(1, sizeof(float));
    if (_pitchQueue == NULL) {
        Serial.println("Failed to create pitch queue!");
        return false;
    }
    _yawQueue = xQueueCreate(1, sizeof(float));
    if (_yawQueue == NULL) {
        Serial.println("Failed to create yaw queue!");
        vQueueDelete(_pitchQueue); 
        return false;
    }

    if (!setup()) {
        Serial.println("IMU Hardware Setup Failed!");
        vQueueDelete(_pitchQueue);
        vQueueDelete(_yawQueue);
        return false;
    }
    else {
        Serial.println("MPU-6050 Found!");
    }

    BaseType_t taskCreated = xTaskCreatePinnedToCore(
        taskTrampoline,
        "Gyro Task",
        Config::GYRO_TASK_STACK_SIZE,
        this,
        Config::GYRO_TASK_PRIORITY,
        &_taskHandle,
        Config::GYRO_TASK_CORE_ID
    );
    if (taskCreated != pdPASS) {
        Serial.println("Failed to create gyro task!");
        vQueueDelete(_pitchQueue);
        vQueueDelete(_yawQueue);
        return false;
    }

    return true;
}

void Gyro::update() {
    _mpu.getEvent(&_a, &_g, &_temp);
    _filter.updateIMU(
        _g.gyro.x - _offset.x,
        _g.gyro.y - _offset.y,
        _g.gyro.z - _offset.z,
        _a.acceleration.x,
        _a.acceleration.y,
        _a.acceleration.z
    );
    
    float currentPitch = _filter.getPitch();
    float currentYaw = _filter.getYaw();
    
    xQueueOverwrite(_pitchQueue, &currentPitch);
    xQueueOverwrite(_yawQueue, &currentYaw);
}

void Gyro::calibrate() {
    float sumX = 0, sumY = 0, sumZ = 0;
    
    for (int i = 0; i < Config::GYRO_CALIBRATION_SAMPLES; i++) {
        _mpu.getEvent(&_a, &_g, &_temp);
        sumX += _g.gyro.x;
        sumY += _g.gyro.y;
        sumZ += _g.gyro.z;
        vTaskDelay(pdMS_TO_TICKS(Config::GYRO_CALIBRATION_DELAY_MS));
    }
    
    _offset.x = sumX / Config::GYRO_CALIBRATION_SAMPLES;
    _offset.y = sumY / Config::GYRO_CALIBRATION_SAMPLES;
    _offset.z = sumZ / Config::GYRO_CALIBRATION_SAMPLES;
}

bool Gyro::setup() {
    Wire.begin(Config::SDA_PIN, Config::SCL_PIN);
    Wire.setClock(Config::WIRE_CLOCK);

    if (!_mpu.begin()) {
        return false;
    }

    _mpu.setAccelerometerRange(Config::ACCEL_RANGE);
    _mpu.setGyroRange(Config::GYRO_RANGE);
    _mpu.setFilterBandwidth(Config::FILTER_BAND);
    
    Serial.println("Calibrating IMU...");
    calibrate();
    Serial.println("Finished Calibrating IMU");

    _filter.begin(Config::SAMPLE_FREQ_HZ);

    return true;
}

void Gyro::taskLoop() {
    static const TickType_t xLoopPeriod = (1000 / Config::SAMPLE_FREQ_HZ) / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        update();
        xTaskDelayUntil(&xLastWakeTime, xLoopPeriod);
    }
}

void Gyro::taskTrampoline(void* pvParameters) {
    Gyro* instance = static_cast<Gyro*>(pvParameters);
    
    instance->taskLoop();

    vTaskDelete(NULL);
}