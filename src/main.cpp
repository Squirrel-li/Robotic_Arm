#include <Arduino.h>
#include <SPI.h>
#include "main.h"


RobotArmController arm; // 機械臂物件
MotionTracker motionTracker; // 姿態追蹤物件

AxisData currentAngle;
AxisData gyroOffset;

uint32_t loopPriod = 5000; // us
uint32_t st, et, loopEnd;

void reflectGRIPPER();

void setup() {
    // pinMode(10, INPUT_PULLUP);
    // pinMode(11, INPUT_PULLUP);

    Serial.begin(2000000, SERIAL_8N2);
    Wire.begin();
    SPI.begin();
    arm.begin();
    motionTracker.begin();

    Wire.setClock(400000);
    delay(100); // 短暫延遲確保 IMU 穩定
    motionTracker.calibrateIMU();
}

void loop() 
{
    static uint32_t nextMotionUpdate = 0;
    static uint32_t nextArmUpdate = 0;
    static uint32_t nextFreqPrint = 0;
    static uint32_t motionExecTime = 0, armExecTime = 0;
    static uint32_t motionCount = 0, armCount = 0;

    uint32_t now = micros();

    // 833Hz: motionTracker 每 1200us 更新一次
    if ((int32_t)(now - nextMotionUpdate) >= 0) {
        uint32_t t0 = micros();
        nextMotionUpdate += 1200;
        motionTracker.update();
        currentAngle = motionTracker.getAngles();
        gyroOffset = motionTracker.getOffsets();
        motionExecTime += micros() - t0;
        motionCount++;
    }

    // 50Hz: arm 每 20000us 更新一次
    if ((int32_t)(now - nextArmUpdate) >= 0) {
        uint32_t t0 = micros();
        nextArmUpdate += 20000;

        // reflectGRIPPER()
        float pitch_div_3 = currentAngle.y / 3;
        arm.setAngle(RobotJoint::BASE, static_cast<float>(currentAngle.z));
        arm.setAngle(RobotJoint::WRIST_PITCH, static_cast<float>(pitch_div_3));
        arm.setAngle(RobotJoint::SHOULDER, static_cast<float>(pitch_div_3));
        arm.setAngle(RobotJoint::ELBOW, static_cast<float>(pitch_div_3));
        arm.setAngle(RobotJoint::WRIST_ROLL, static_cast<float>(currentAngle.x));

        armExecTime += micros() - t0;
        armCount++;
    }

    // 每 0.1 秒 (100ms) 印出一次頻率與執行時間
    if ((int32_t)(now - nextFreqPrint) >= 0) {
        // 改為每 50ms 執行一次分相逻辑，維持總體 10Hz (100ms) 的更新頻率
        const uint32_t phaseInterval = 50000; 
        nextFreqPrint += phaseInterval;
        
        static uint8_t printPhase = 0; // 用於切換輸出的相位

        if (printPhase == 0) {
            // 相位 0：只印出頻率與執行時間
            // 注意：這裡 motionCount 與 armCount 累計的是 50ms，故乘以 20 換算回 Hz
            Serial.print(F("[Arm] Freq: "));
            Serial.print(armCount * 10);
            Serial.print(F("Hz, Time: "));
            Serial.print(armCount ? (armExecTime / armCount) : 0);
            Serial.print(F("us | [MT] Freq: "));
            Serial.print(motionCount * 10);
            Serial.print(F("Hz, Time: "));
            Serial.print(motionCount ? (motionExecTime / motionCount) : 0);
            Serial.print(F("us, "));

            // 重置計數器（在第一階段完成後重置，確保下一階段印出的是新數據）
            motionExecTime = 0; armExecTime = 0;
            motionCount = 0; armCount = 0;
            printPhase = 1; 
        } 
        else {
            // 相位 1：印出角度數據與 Offset
            Serial.print(F(" -> x: ")); Serial.print(currentAngle.x);
            Serial.print(F(" , y: ")); Serial.print(currentAngle.y);
            Serial.print(F(" , z: ")); Serial.print(currentAngle.z);
            Serial.print(F("  | Off: ("));
            Serial.print(gyroOffset.x); Serial.print(F(","));
            Serial.print(gyroOffset.y); Serial.print(F(","));
            Serial.print(gyroOffset.z); 
            Serial.println(F(")"));
            
            printPhase = 0;
        }
    }
}

void reflectGRIPPER() 
{
    bool turnRight = digitalRead(11) == LOW;
    bool turnLeft = digitalRead(10) == LOW;

    if (turnRight && turnLeft) return;

    if (turnRight)
    {
        arm.rotateRelativePulse(RobotJoint::GRIPPER, (int16_t)10);
    }
    if (turnLeft)
    {
        arm.rotateRelativePulse(RobotJoint::GRIPPER, (int16_t)-10);
    }
}