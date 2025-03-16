#ifndef BMI270_READ_H
#define BMI270_READ_H

#include <Arduino.h>
#include <Wire.h>
#include "bmi270/bmi270.h"

// 定义常量
#define GRAVITY_EARTH  (9.80665f)
#define BMI270_I2C_ADDR_PRIMARY 0x68
#define BMI270_I2C_ADDR_SECONDARY 0x69
#define BMI2_CHIP_ID_ADDR      UINT8_C(0x00)
#define BMI270_CHIP_ID         UINT8_C(0x24)

// 传感器数据结构
struct SensorData {
  float acc_x;
  float acc_y;
  float acc_z;
  float gyr_x;
  float gyr_y;
  float gyr_z;
  bool valid;
};

class BMI270Read {
private:
  struct bmi2_dev bmi;
  uint8_t dev_addr;
  int8_t sda_pin;
  int8_t scl_pin;
  bool initialized;
  bool skip_i2c_init;

  // I2C通信回调函数
  static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
  static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
  static void delay_us(uint32_t period, void *intf_ptr);

  // 内部辅助函数
  int8_t read_chip_id(uint8_t *chip_id);
  int8_t set_accel_gyro_config();
  float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);
  float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);
  void init_bmi_dev();

public:
  BMI270Read(int8_t sda = -1, int8_t scl = -1, uint8_t address = BMI270_I2C_ADDR_PRIMARY) :
    dev_addr(address), sda_pin(sda), scl_pin(scl), initialized(false), skip_i2c_init(false) {}
  ~BMI270Read();

  // 设置是否跳过I2C初始化
  void skipI2CInit(bool skip) { skip_i2c_init = skip; }

  // 初始化函数
  bool begin();
  
  // 扫描I2C总线
  int scanI2C();
  
  // 读取传感器数据
  SensorData readSensorData();
  
  // 获取初始化状态
  bool isInitialized() { return initialized; }
};

#endif // BMI270_READ_H