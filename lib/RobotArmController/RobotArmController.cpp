#define ROBOT_ARM_CONTROLLER_CPP

#include <Arduino.h>
#include "RobotArmController.h"

// #define ENABLE_DEBUG_OUTPUT

namespace {
    // 定點數優化常數
    constexpr int32_t K_ANGLE_TO_TICK_SCALED = static_cast<int32_t>(
        (static_cast<uint64_t>(servoMaxPulse - servoMinPulse) *
         static_cast<uint64_t>(pwmResolution) *
         static_cast<uint64_t>(pwmFreq) *
         1024ULL) / (180ULL * 1000000ULL)
    );
    constexpr int32_t K_MIN_PULSE_TICK = static_cast<int32_t>(servoMinPulse) * pwmResolution * pwmFreq / 1000000UL;
    constexpr uint16_t usToTickConst(uint16_t us) {
        return static_cast<uint16_t>(
            (static_cast<uint32_t>(us) * static_cast<uint32_t>(pwmFreq) * static_cast<uint32_t>(pwmResolution) + 500000UL)
            / 1000000UL
        );
    }
    constexpr uint32_t kTicksPerSecond =
        static_cast<uint32_t>(pwmResolution) * static_cast<uint32_t>(pwmFreq);
    constexpr float kTicksPerUs   = static_cast<float>(kTicksPerSecond) / 1000000.0f;
    constexpr int32_t kRoundUs = 500000; // for integer rounding
}

RobotArmController::RobotArmController() : RobotArmController(PCA9685_I2C_ADDRESS) {
    Serial.print("RobotArmController created with default I2C address 0x");
    Serial.println(PCA9685_I2C_ADDRESS, HEX);
}

RobotArmController::RobotArmController(const uint8_t i2cAddress) : _pwm(i2cAddress)
{
    #ifdef ENABLE_DEBUG_OUTPUT
    Serial.println("RobotArmController created");
    #endif

    for (uint8_t i = 0; i < static_cast<uint8_t>(RobotJoint::JOINT_COUNT); i++) {
        _jointTick[i] = jointConfigs[i].initTick;
    }
}

void RobotArmController::begin()
{
    #ifdef ENABLE_DEBUG_OUTPUT
    Serial.println("RobotArmController begin");
    #endif

    _pwm.begin();
    _pwm.setPWMFreq(pwmFreq);
    resetAll();
}

void RobotArmController::setAngle(RobotJoint joint, float angleDeg)
{
    setTick(joint, angleToTick(angleDeg));
}

void RobotArmController::setPulse(RobotJoint joint, uint16_t pulseUs)
{
    setTick(joint, pulseToTick(static_cast<float>(pulseUs)));
}

void RobotArmController::setTick(RobotJoint joint, int32_t off)
{
    const uint8_t idx = static_cast<uint8_t>(joint);
    const JointConfig& cfg = jointConfigs[idx];

    if (off < cfg.minTick) off = cfg.minTick;
    else if (off > cfg.maxTick) off = cfg.maxTick;

    uint16_t logicalOff = static_cast<uint16_t>(off);

    // 數值未變動則不發送 I2C 指令
    // if (logicalOff == _jointTick[idx]) return;

    uint16_t outputOff = logicalOff;
    if (cfg.reversed) {
        outputOff = cfg.minPlusMax - logicalOff;
    }

    _pwm.setPWM(cfg.channel, 0, outputOff);
    _jointTick[idx] = logicalOff;
}

void RobotArmController::rotateRelativeAngle(RobotJoint joint, float deltaAngleDeg)
{
    const uint8_t idx = static_cast<uint8_t>(joint);
    // 定點數運算
    int32_t deltaAngle = static_cast<int32_t>(deltaAngleDeg + (deltaAngleDeg >= 0 ? 0.5f : -0.5f));
    int32_t deltaTick = (deltaAngle * K_ANGLE_TO_TICK_SCALED) >> 10;
    setTick(joint, static_cast<int32_t>(_jointTick[idx]) + deltaTick);
}

void RobotArmController::rotateRelativePulse(RobotJoint joint, int16_t deltaPulse)
{
    const uint8_t idx = static_cast<uint8_t>(joint);

    // 直接 delta us -> delta tick（整數）
    const int32_t num = static_cast<int32_t>(deltaPulse) * static_cast<int32_t>(kTicksPerSecond);
    const int32_t deltaTick = (num >= 0) ? (num + kRoundUs) / 1000000 : (num - kRoundUs) / 1000000;

    setTick(joint, static_cast<int32_t>(_jointTick[idx]) + deltaTick);
}

void RobotArmController::setAllAngle(float angleDeg)
{
    int32_t tick = angleToTick(angleDeg); // 只算一次
    for (uint8_t i = 0; i < static_cast<uint8_t>(RobotJoint::JOINT_COUNT); i++) {
        setTick(static_cast<RobotJoint>(i), tick);
    }
}

void RobotArmController::setAllPulse(uint16_t pulse)
{
    int32_t tick = pulseToTick(static_cast<float>(pulse)); // 只算一次
    for (uint8_t i = 0; i < static_cast<uint8_t>(RobotJoint::JOINT_COUNT); i++) {
        setTick(static_cast<RobotJoint>(i), tick);
    }
}

void RobotArmController::resetAll()
{
    for (uint8_t i = 0; i < static_cast<uint8_t>(RobotJoint::JOINT_COUNT); i++) {
        setTick(static_cast<RobotJoint>(i), jointConfigs[i].initTick);
    }
}

// 全部使用 tick 限制，minPlusMax 預先計算
const RobotArmController::JointConfig RobotArmController::jointConfigs[static_cast<uint8_t>(RobotJoint::JOINT_COUNT)] = {
    {0, usToTickConst(servoMinPulse),       usToTickConst(servoMaxPulse),       static_cast<uint16_t>(usToTickConst(servoMinPulse) + usToTickConst(servoMaxPulse)),       usToTickConst(1500),                 false},
    {1, usToTickConst(servoMinPulse),       usToTickConst(servoMaxPulse),       static_cast<uint16_t>(usToTickConst(servoMinPulse) + usToTickConst(servoMaxPulse)),       usToTickConst(servoMinPulse + 100),  true },
    {2, usToTickConst(servoMinPulse),       usToTickConst(servoMaxPulse - 100), static_cast<uint16_t>(usToTickConst(servoMinPulse) + usToTickConst(servoMaxPulse - 100)), usToTickConst(servoMinPulse + 100),  false},
    {3, usToTickConst(servoMinPulse),       usToTickConst(servoMaxPulse),       static_cast<uint16_t>(usToTickConst(servoMinPulse) + usToTickConst(servoMaxPulse)),       usToTickConst(servoMinPulse + 200),  false},
    {4, usToTickConst(servoMinPulse),       usToTickConst(servoMaxPulse),       static_cast<uint16_t>(usToTickConst(servoMinPulse) + usToTickConst(servoMaxPulse)),       usToTickConst(1500),                 false},
    {5, usToTickConst(1150),                usToTickConst(2050),                static_cast<uint16_t>(usToTickConst(1150) + usToTickConst(2050)),                        usToTickConst(1500),                 false},
};

int32_t RobotArmController::angleToTick(float angleDeg)
{
    // 去浮點化，定點數運算
    int32_t angle = static_cast<int32_t>(angleDeg + (angleDeg >= 0 ? 0.5f : -0.5f));
    return ((angle * K_ANGLE_TO_TICK_SCALED) >> 10) + K_MIN_PULSE_TICK;
}

int32_t RobotArmController::pulseToTick(float pulseUs)
{
    return static_cast<int32_t>(pulseUs * kTicksPerUs + 0.5f);
}

float RobotArmController::tickToPulse(float tick)
{
    const float ticksPerSecond =
        static_cast<float>(static_cast<uint32_t>(pwmResolution) * static_cast<uint32_t>(pwmFreq));
    return tick * (1000000.0f / ticksPerSecond);
}

