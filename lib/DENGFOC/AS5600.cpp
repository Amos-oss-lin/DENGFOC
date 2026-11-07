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

  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    bool ok = false;

    if (attempt < 2) {
      // 先写寄存器地址（STOP），再单独发起读事务。
      // 这样避开ESP32 Wire库的 i2cWriteReadNonStop（重复起始）在电机PWM干扰下
      // 容易返回 Error -1 / 263 的问题；AS5600的寄存器指针在STOP后仍然保持。
      wire->beginTransmission(AS5600_ADDR);
      wire->write((uint8_t)AS5600_ANGLE_REG);
      if (wire->endTransmission(true) == 0) {
        ok = (wire->requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) == 2);
      }
    } else {
      // 最后一次兼容标准重复起始时序
      wire->beginTransmission(AS5600_ADDR);
      wire->write((uint8_t)AS5600_ANGLE_REG);
      if (wire->endTransmission(false) == 0) {
        ok = (wire->requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) == 2);
      }
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
    delayMicroseconds(100);
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
    wire->setTimeOut(10);  //避免总线异常时阻塞50ms
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
    last_sensor_angle = val;
    angle_prev_ts = micros();
    float d_angle = val - angle_prev;
    // 圈数检测
    if(abs(d_angle) > (0.8f*_2PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
    return true;
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
    // 保存变量以待将来使用
    last_velocity = vel;
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return vel;
}
