#include "KalmanFilter.h"
#include <math.h>

KalmanFilter::KalmanFilter(float q_angle, float q_gyro, float r_angle) {
  // 初始化噪声参数
  Q_angle = q_angle;
  Q_gyro = q_gyro;
  R_angle = r_angle;
  
  // 初始化状态
  pitch = 0;
  roll = 0;
  yaw = 0;
  
  // 初始化协方差矩阵
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      P[i][j] = 0;
    }
  }
  P[0][0] = P[1][1] = P[2][2] = 1.0; // 对角线元素设为1
  
  lastTime = 0;
  dt = 0;
}

void KalmanFilter::begin() {
  lastTime = millis();
}

// 从加速度计数据计算角度
void KalmanFilter::updateAngleFromAccel(SensorData &data, float &accel_pitch, float &accel_roll) {
  // 计算总加速度
  float acc_total = sqrt(data.acc_x * data.acc_x + data.acc_y * data.acc_y + data.acc_z * data.acc_z);
  
  // 防止除以零
  if (acc_total < 0.1) {
    return;
  }
  
  // 计算俯仰角和横滚角
  accel_pitch = asin(data.acc_x / acc_total) * 180.0 / PI;
  accel_roll = asin(data.acc_y / acc_total) * 180.0 / PI;
  
  // 处理特殊情况
  if (data.acc_z < 0) {
    if (data.acc_x > 0) accel_pitch = 180 - accel_pitch;
    else accel_pitch = -180 - accel_pitch;
    
    accel_roll = 180 - accel_roll;
  }
}

void KalmanFilter::update(SensorData &data) {
  // 计算时间增量
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0; // 转换为秒
  lastTime = now;
  
  if (dt <= 0 || !data.valid) {
    return;
  }
  
  // 从加速度计计算角度
  float accel_pitch = 0, accel_roll = 0;
  updateAngleFromAccel(data, accel_pitch, accel_roll);
  
  // 预测步骤 - 使用陀螺仪数据更新角度
  float gyro_pitch = pitch + data.gyr_x * dt;
  float gyro_roll = roll + data.gyr_y * dt;
  float gyro_yaw = yaw + data.gyr_z * dt;
  
  // 更新协方差矩阵
  P[0][0] += dt * (dt * P[1][1] + Q_angle);
  P[1][1] += Q_gyro * dt;
  P[0][1] = P[1][0] = P[0][1] + dt * P[1][1];
  
  // 计算卡尔曼增益
  float K0 = P[0][0] / (P[0][0] + R_angle);
  float K1 = P[0][1] / (P[0][0] + R_angle);
  
  // 更新步骤 - 融合加速度计和陀螺仪数据
  pitch = gyro_pitch + K0 * (accel_pitch - gyro_pitch);
  roll = gyro_roll + K0 * (accel_roll - gyro_roll);
  yaw = gyro_yaw; // 偏航角只能通过陀螺仪积分或磁力计获取
  
  // 更新协方差矩阵
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  
  P[0][0] -= K0 * P00_temp;
  P[0][1] -= K0 * P01_temp;
  P[1][0] -= K1 * P00_temp;
  P[1][1] -= K1 * P01_temp;
  
  // 规范化偏航角到0-360度
  while (yaw < 0) yaw += 360;
  while (yaw >= 360) yaw -= 360;
} 