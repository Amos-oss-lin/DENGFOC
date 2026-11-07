#include <Arduino.h> 
#include "Wire.h"

class Sensor_AS5600
{
  public:
    Sensor_AS5600(int Mot_Num);
    void Sensor_init(TwoWire* _wire = &Wire);
    bool Sensor_update();
    float getAngle();
    float getVelocity();
    float getMechanicalAngle();
    double getSensorAngle();
    bool isReady() const;
    uint32_t getI2CErrors() const;
    uint32_t getI2CFailures() const;
  private:
    bool readSensorAngle(float& angle);
    int _Mot_Num;
    //AS5600 变量定义
    //int sensor_direction=1;       //编码器旋转方向定义
    float angle_prev=0; // 最后一次调用 getSensorAngle() 的输出结果，用于得到完整的圈数和速度
    long angle_prev_ts=0; // 上次调用 getAngle 的时间戳
    float vel_angle_prev=0; // 最后一次调用 getVelocity 时的角度
    long vel_angle_prev_ts=0; // 最后速度计算时间戳
    int32_t full_rotations=0; // 总圈数计数
    int32_t vel_full_rotations=0; //用于速度计算的先前完整旋转圈数
    float last_sensor_angle=0; // 最近一次成功读取的角度
    float last_velocity=0; // 最近一次速度，I2C短暂失败时保持
    bool sensor_ready=false; // 传感器是否可用
    uint16_t consecutive_failures=0; // 连续失败次数
    uint32_t last_retry_ms=0; // 故障后的重试时间
    uint32_t i2c_error_count=0; // I2C事务失败次数（含重试后成功的）
    uint32_t i2c_fail_count=0; // 完全读取失败的采样次数
    TwoWire* wire=nullptr;
};
