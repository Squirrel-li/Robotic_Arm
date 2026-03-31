#ifndef MOTION_TRACKER_H
#define MOTION_TRACKER_H

#include <Arduino.h>
#include <Adafruit_LSM6DS3.h>
#include "../Queue/Queue.h"

#define gyroQueueSize 16
#define inv_gyroQueueSize (1.0f / gyroQueueSize)

struct AxisData {
    float x;
    float y;
    float z;

    // AxisData 與 AxisData
    AxisData operator+(const AxisData& rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    AxisData operator-(const AxisData& rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    AxisData operator*(const AxisData& rhs) const { return {x * rhs.x, y * rhs.y, z * rhs.z}; }
    AxisData operator/(const AxisData& rhs) const { return {x / rhs.x, y / rhs.y, z / rhs.z}; }

    AxisData& operator+=(const AxisData& rhs) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
    AxisData& operator-=(const AxisData& rhs) { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }
    AxisData& operator*=(const AxisData& rhs) { x *= rhs.x; y *= rhs.y; z *= rhs.z; return *this; }
    AxisData& operator/=(const AxisData& rhs) { x /= rhs.x; y /= rhs.y; z /= rhs.z; return *this; }

    // AxisData 與 scalar
    AxisData operator*(float s) const { return {x * s, y * s, z * s}; }
    AxisData operator/(float s) const { return {x / s, y / s, z / s}; }

    AxisData& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
    AxisData& operator/=(float s) { x /= s; y /= s; z /= s; return *this; }

    // 支援 scalar * AxisData
    friend AxisData operator*(float s, const AxisData& v) { return {v.x * s, v.y * s, v.z * s}; }
};

/**
 * @class MotionTracker
 * @brief 整合 LSM6DS3 的姿態追蹤類別，提供校準與角度積分功能。
 */

class MotionTracker {
public:

    /**
     * @brief 建構子
     * @param imuInstance 建立預設的 Adafruit_LSM6DS3 實體引用
     */
    MotionTracker();

    /**
     * @brief 建構子
     * @param imuInstance 傳入已初始化的 Adafruit_LSM6DS3 實體引用
     */
    MotionTracker(uint8_t imuAddress);

    /**
     * @brief 解構子
     * @param 無參數，釋放資源（如果有的話）
     */
    ~MotionTracker();

    /**
     * @brief 初始化 IMU 感測器，應在 setup() 中呼叫
     * @return 無返回值，初始化失敗會在序列監視器輸出錯誤訊息並停止程式
     */
    void begin();

    /**
     * @brief 執行陀螺儀靜態校準
     * @param durationMs 校準持續時間（毫秒），建議至少 3000ms
     */
    void calibrateIMU(uint32_t durationMs = 3000);

    /**
     * @brief 更新感測器數據並執行角度積分
     * 應在 loop()中高頻率呼叫
     */
    void update();

    /**
     * @brief 更新感測器數據並執行角度積分
     * 應在 loop或中斷中高頻率呼叫
     */
    void updateQueue();

    /**
     * @brief 更新感測器數據並執行角度積分
     * 應在 loop()中高頻率呼叫
     */
    void updateAngle();

    /**
     * @brief 更新感測器數據並執行角度積分
     * 應在 loop()中高頻率呼叫
     */
    void updateInterrupt();

    /**
     * @brief 輸出當前的姿態角度與校準偏移量，格式為 "Pitch: x, Roll: y, Yaw: z, Offsets: (x_offset, y_offset, z_offset)"
     * @return 無返回值，直接在序列監視器輸出調試資訊
     */
    void printDebug();

    /**
     * @brief 取得當前的姿態角度（單位：度）
     * @return 以 float 結構返回 pitch (x)、roll (y)、yaw (z) 角度
     */
    float getPitch() const { return _angle.x; }
    float getRoll()  const { return _angle.y; }
    float getYaw()   const { return _angle.z; }
    
    /**
     * @brief 取得當前的姿態角度（單位：度）
     * @return 以 float 結構返回 pitch (x)、roll (y)、yaw (z) 角度
     */
    AxisData getAngles() const { return _angle; }

    /**
     * @brief 取得陀螺儀的校準偏移量（單位：度/秒）
     * @return 以 float 結構返回 x、y、z 軸的偏移量
     */
    AxisData getOffsets() const { return _offset; }

    /**
     * @brief 重置積分角度與校準偏移量
     * @return 無返回值，重置後角度與偏移量將被清零
     */
    void resetAngles();

private:
    uint8_t _imuAddress;
    Adafruit_LSM6DS3 _imu;
    AxisData _angle;
    AxisData _offset;
    AxisData _lastGyro;
    Queue<AxisData, gyroQueueSize> _gyroQueue;
    AxisData _gyroSum;
    uint32_t _lastMicros;
    bool _isCalibrated;
};

#endif // MOTION_TRACKER_H