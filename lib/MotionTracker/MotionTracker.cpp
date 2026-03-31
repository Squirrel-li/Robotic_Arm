#define MOTION_TRACKER_CPP

#include <Arduino.h>
#include "MotionTracker.h"

MotionTracker::MotionTracker() : MotionTracker(0x6B)
{
    Serial.println("Default IMU initialized at 0x6B.");
}

MotionTracker::MotionTracker(uint8_t imuAddress)
 : _imuAddress(imuAddress),
   _angle({0.0f}),
   _offset({0.0f}),
   _lastGyro({0.0f}),
   _gyroQueue(true),
   _lastMicros(0),
   _isCalibrated(false)
{
    Serial.print("MotionTracker initialized with IMU at address 0x");
    Serial.println(imuAddress, HEX);
}

MotionTracker::~MotionTracker() {}

void MotionTracker::begin()
{
    const int csPin = 9; // SPI CS pin
    if (!_imu.begin_SPI(csPin)) {
        Serial.println("IMU initialization failed! Halting.");
        while (1) delay(1000);
    }
    _imu.setAccelDataRate(data_rate::LSM6DS_RATE_833_HZ);
    _imu.setGyroDataRate(data_rate::LSM6DS_RATE_833_HZ);
}

namespace {
    constexpr float kMicrosToSec = 1.0e-6f;
}

void MotionTracker::calibrateIMU(uint32_t durationMs)
{
    const int32_t targetSamples = 3000;
    const int32_t segmentSize = 10; // 每段累加 10 筆
    int32_t samples = 0;
    int32_t segmentCount = 0;
    AxisData gyroSum{0.0f};
    AxisData accSum{0.0f};

    Serial.println("Calibrating IMU... Please keep it still.");

    while (samples < targetSamples)
    {
        AxisData gyroSegment{0.0f};
        AxisData accSegment{0.0f};
        int32_t segmentSamples = 0;

        // 分段累加
        while (segmentSamples < segmentSize && samples < targetSamples)
        {
            AxisData gyro, acc;
            if (!_imu.gyroscopeAvailable() || !_imu.accelerationAvailable()) {
                Serial.println("Waiting for IMU data...");
                delay(100);
                continue;
            }
            _imu.readGyroscope(gyro.x, gyro.y, gyro.z);
            _imu.readAcceleration(acc.x, acc.y, acc.z);

            // 判斷是否靜止
            if (abs((acc.x * acc.x + acc.y * acc.y + acc.z * acc.z) - 1.0f) < 0.1f) {
                gyroSegment += gyro;
                accSegment  += acc;
                ++segmentSamples;
                ++samples;
            } else {
                // 若偵測到移動，重置累積
                Serial.println("Motion detected during calibration! Restarting sample collection...");
                samples = 0;
                gyroSum = AxisData{0.0f};
                accSum = AxisData{0.0f};
                segmentSamples = 0;
                segmentCount = 0;
                break;
            }
            delayMicroseconds(1200); // 約 833Hz 的取樣率
        }

        // 若本段累加有完成，才加到總和（分段取平均再加總）
        if (segmentSamples == segmentSize) {
            gyroSum += (gyroSegment * (1.0f / segmentSize));
            accSum  += (accSegment  * (1.0f / segmentSize));
            ++segmentCount;
        }
    }

    Serial.print("Samples collected: "); Serial.println(samples);
    Serial.println("Calibration complete.");

    const float invSegments = 1.0f / static_cast<float>(segmentCount);
    const AxisData accAvg = accSum * invSegments;
    _offset = gyroSum * invSegments;

    _angle.x = atan2(accAvg.y, accAvg.z) * RAD_TO_DEG;
    _angle.y = atan2(-accAvg.x, sqrt(accAvg.y * accAvg.y + accAvg.z * accAvg.z)) * RAD_TO_DEG;
    _angle.z = 0.0f;

    _lastMicros = micros();
    _isCalibrated = true;
}

void MotionTracker::update()
{
    if (!_isCalibrated) return;
    updateQueue();
    updateAngle();
}

void MotionTracker::updateQueue()
{
    if (!_isCalibrated) return;

    static AxisData currentData;
    static AxisData overwritten;

    // 讀取當前陀螺儀數據並扣除偏移量
    _imu.readGyroscope(currentData.x, currentData.y, currentData.z);
    currentData -= _offset;
    if (abs(currentData.x) < 0.05f) currentData.x = 0;
    if (abs(currentData.y) < 0.05f) currentData.y = 0;
    if (abs(currentData.z) < 0.05f) currentData.z = 0;

    if (_gyroQueue.pushOverwrite(currentData, &overwritten))
    {
        _gyroSum -= overwritten;
    }
    _gyroSum += currentData;

    if (inv_gyroQueueSize > 0.0f) {
        _lastGyro = _gyroSum * inv_gyroQueueSize;
    }
}

void MotionTracker::updateAngle()
{
    if (!_isCalibrated) return;

    uint32_t now = micros();
    if (_lastMicros == 0) {
        _lastMicros = now;
        return;
    }

    float dt = static_cast<float>(now - _lastMicros) * kMicrosToSec;
    _lastMicros = now;

    _angle += _lastGyro * dt;
}
