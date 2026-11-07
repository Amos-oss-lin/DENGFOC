#include "AS5600.h"
#include "Wire.h"
#include <Arduino.h> 

#define _2PI 6.28318530718f

// AS5600 角度寄存器
#define AS5600_ADDR        0x36
#define AS5600_ANGLE_REG   0x0C
#define AS5600_FAIL_LIMIT  5   // 连续5次采样失败则判定传感器掉线，停止闭环输出

// AS5600 相关
double Sensor_AS5600::getSensorAngle() {
  float angle;
  if (readSensorAngle(angle)) {
    last_sensor_angle = angle;
    sensor_ready = true;
    consecutive_failures = 0;
    return angle;
  }
  if (consecutive_failures < 0xFFFF) consecutive_failures++;
  if (consecutive_failures >= AS5600_FAIL_LIMIT) sensor_ready = false;
  return last_sensor_angle;
}

// 读取一次AS5600角度；失败时自动重试，返回false表示本次采样彻底失败
bool Sensor_AS5600::readSensorAngle(float& angle) {
  if (wire == nullptr) return false;

  // 只做两次快速尝试，且每次I2C超时1ms：
  // 60rad/s附近电机振动会让AS5600偶发NACK，旧代码每次重试可能阻塞10ms，
  // 控制周期被拉长后表现为“串口卡顿/速度乱摆”。宁可这一拍丢角度，也不能阻塞控制环。
  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    bool ok = false;

    // 先写寄存器地址（STOP），再单独发起读事务。
    // 这样避开ESP32 Wire库的 i2cWriteReadNonStop（重复起始）在电机PWM干扰下
    // 容易返回 Error -1 / 263 的问题；AS5600的寄存器指针在STOP后仍然保持。
    wire->beginTransmission(AS5600_ADDR);
    wire->write((uint8_t)AS5600_ANGLE_REG);
    if (wire->endTransmission(true) == 0) {
      ok = (wire->requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) == 2);
    }

    if (ok) {
      int hi = wire->read();
      int lo = wire->read();
      if (hi >= 0 && lo >= 0) {
        uint16_t raw = ((uint16_t)(hi & 0x0F) << 8) | (uint8_t)lo;
        angle = ((float)raw / 4096.0f) * _2PI;
        return true;
      }
    }

    // 清掉可能残留的数据，计数并重试
    while (wire->available()) wire->read();
    i2c_error_count++;
    delayMicroseconds(50);
  }

  i2c_fail_count++;
  return false;
}

//AS5600 相关

//=========角度处理相关=============
Sensor_AS5600::Sensor_AS5600(int Mot_Num) {
   _Mot_Num=Mot_Num;  //使得 Mot_Num 可以统一在该文件调用
}

void Sensor_AS5600::Sensor_init(TwoWire* _wire) {
    wire=_wire;
    wire->begin();   //已经显式启动的总线会保持原有引脚配置
    wire->setTimeOut(1);  //1ms；总线异常时绝不允许长时间阻塞控制环
    delay(500);

    float initial_angle;
    bool ok = false;
    // 上电时最多重试10次，排除AS5600上电/总线未稳定的偶发失败
    for (uint8_t i = 0; i < 10; i++) {
      if (readSensorAngle(initial_angle)) {
        ok = true;
        break;
      }
      delay(20);
    }
    if (!ok) {
      sensor_ready = false;
      Serial.printf("M%d AS5600初始化失败：I2C未读到角度数据\n", _Mot_Num);
      return;
    }

    last_sensor_angle = initial_angle;
    angle_prev = initial_angle;
    vel_angle_prev = initial_angle;
    angle_prev_ts = micros();
    vel_angle_prev_ts = angle_prev_ts;
    sensor_ready = true;
    consecutive_failures = 0;
}

bool Sensor_AS5600::Sensor_update() {
    // 总线故障后不要每个循环都高频重试，每20ms探测一次
    if (!sensor_ready) {
      uint32_t now = millis();
      if (now - last_retry_ms < 20) return false;
      last_retry_ms = now;
    }

    float val;
    if (!readSensorAngle(val)) {
      if (consecutive_failures < 0xFFFF) consecutive_failures++;
      if (consecutive_failures >= AS5600_FAIL_LIMIT) sensor_ready = false;
      return false;
    }

    consecutive_failures = 0;
    sensor_ready = true;

    uint32_t now = micros();
    float dt = (now - angle_prev_ts) * 1e-6f;
    if (dt <= 0.0f) dt = 1e-3f;

    // 角度跳变检查：AS5600 偶尔会在PWM干扰下读到一个“成功”但错误的角度。
    // 真实机械角速度不会超过约120rad/s，单次采样内角度变化超过 120*dt 就丢弃；
    // 否则会被当成一圈跳变，速度瞬间出现 ±200/585 这种尖峰，并让电流环打出冲击。
    float d_raw = val - angle_prev;
    float d_unwrapped = d_raw;
    if (d_unwrapped > (float)PI) d_unwrapped -= _2PI;
    else if (d_unwrapped < -(float)PI) d_unwrapped += _2PI;

    float max_step = 120.0f * dt;
    if (fabs(d_unwrapped) > max_step) {
      angle_glitch_count++;
      return false;   // 保持上一拍角度，等下一次正常采样
    }

    // 圈数检测
    if (d_raw > (float)PI) full_rotations -= 1;
    else if (d_raw < -(float)PI) full_rotations += 1;

    last_sensor_angle = val;
    angle_prev = val;
    angle_prev_ts = now;
    return true;
}

uint32_t Sensor_AS5600::getAngleGlitches() const {
    return angle_glitch_count;
}

bool Sensor_AS5600::isReady() const {
    return sensor_ready;
}

uint32_t Sensor_AS5600::getI2CErrors() const {
    return i2c_error_count;
}

uint32_t Sensor_AS5600::getI2CFailures() const {
    return i2c_fail_count;
}

float Sensor_AS5600::getMechanicalAngle() {
    return angle_prev;
}

float Sensor_AS5600::getAngle(){
    return (float)full_rotations * _2PI + angle_prev;
}

float Sensor_AS5600::getVelocity() {
    // 本次没有成功采样时不更新，保持上一次速度，避免I2C偶发失败造成速度掉0
    if (angle_prev_ts == vel_angle_prev_ts) {
      return last_velocity;
    }

    // 计算采样时间
    float Ts = (angle_prev_ts - vel_angle_prev_ts)*1e-6;
    // 快速修复奇怪的情况（微溢出）
    if(Ts <= 0) Ts = 1e-3f;
    // 速度计算
    float vel = ( (float)(full_rotations - vel_full_rotations)*_2PI + (angle_prev - vel_angle_prev) ) / Ts;    
    // 兜底：极端情况下（ts异常/残留毛刺）不允许速度尖峰进入速度环
    if (!isfinite(vel) || fabs(vel) > 120.0f) {
      vel = last_velocity;
    } else {
      last_velocity = vel;
    }
    // 保存变量以待将来使用
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return vel;
}
