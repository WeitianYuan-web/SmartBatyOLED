#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Arduino.h>
#include "BMI270Read.h"

class KalmanFilter {
private:
  // 状态变量
  float pitch;      // 俯仰角
  float roll;       // 横滚角
  float yaw;        // 偏航角
  
  // 协方差矩阵
  float P[3][3];    // 状态估计协方差
  
  // 噪声参数
  float Q_angle;    // 过程噪声 - 角度
  float Q_gyro;     // 过程噪声 - 陀螺仪
  float R_angle;    // 测量噪声 - 角度
  
  // 时间相关
  unsigned long lastTime;
  float dt;         // 时间增量
  
  // 辅助函数
  void updateAngleFromAccel(SensorData &data, float &accel_pitch, float &accel_roll);
  
public:
  KalmanFilter(float q_angle = 0.001, float q_gyro = 0.003, float r_angle = 0.03);
  
  // 初始化滤波器
  void begin();
  
  // 更新滤波器
  void update(SensorData &data);
  
  // 获取估计的角度
  float getPitch() { return pitch; }
  float getRoll() { return roll; }
  float getYaw() { return yaw; }
};

#endif // KALMAN_FILTER_H 