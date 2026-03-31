#ifndef ROBOT_ARM_CONTROLLER_H
#define ROBOT_ARM_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>

constexpr int pwmFreq = 50;
constexpr int pwmResolution = 4096;
constexpr int servoMinPulse = 500;
constexpr int servoMaxPulse = 2500;

enum class RobotJoint : uint8_t 
{
  BASE = 0,
  SHOULDER,
  ELBOW,
  WRIST_PITCH,
  WRIST_ROLL,
  GRIPPER,
  JOINT_COUNT
};

class RobotArmController 
{
public:
    RobotArmController();
    RobotArmController(const uint8_t i2cAddress);
    void begin();
    void setAngle(RobotJoint joint, float angleDeg);
    void setPulse(RobotJoint joint, uint16_t pulse);
    void setTick(RobotJoint joint, int32_t off);
    void rotateRelativeAngle(RobotJoint joint, float deltaAngleDeg);
    void rotateRelativePulse(RobotJoint joint, int16_t deltaPulse);
    void setAllAngle(float angleDeg);
    void setAllPulse(uint16_t pulse);
    void resetAll();

private:
    struct JointConfig 
    {
        uint8_t channel;
        uint16_t minTick, maxTick;
        uint16_t minPlusMax; // 新增：預先儲存 minTick + maxTick
        uint16_t initTick;
        bool reversed;
    };

    uint16_t _jointTick[static_cast<uint8_t>(RobotJoint::JOINT_COUNT)];
    static const JointConfig jointConfigs[static_cast<uint8_t>(RobotJoint::JOINT_COUNT)];

    Adafruit_PWMServoDriver _pwm;

    int32_t angleToTick(float angleDeg);
    int32_t pulseToTick(float pulseUs);
    float tickToPulse(float tick);
};

#endif