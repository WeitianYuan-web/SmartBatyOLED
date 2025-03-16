#include "BMI270Read.h"

// 析构函数
BMI270Read::~BMI270Read() {
  // 清理资源（如果需要）
}

// I2C读取回调
int8_t BMI270Read::i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
  uint8_t dev_addr = *(uint8_t*)intf_ptr;
  
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  if (Wire.endTransmission(false) != 0) {
    return 1; // 传输失败
  }
  
  uint8_t bytes_received = Wire.requestFrom(dev_addr, (uint8_t)len);
  if (bytes_received != len) {
    return 1; // 读取失败
  }
  
  for (uint32_t i = 0; i < len; i++) {
    reg_data[i] = Wire.read();
  }
  
  return 0; // 成功
}

// I2C写入回调
int8_t BMI270Read::i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
  uint8_t dev_addr = *(uint8_t*)intf_ptr;
  
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  
  for (uint32_t i = 0; i < len; i++) {
    Wire.write(reg_data[i]);
  }
  
  if (Wire.endTransmission() != 0) {
    return 1; // 写入失败
  }
  
  return 0; // 成功
}

// 延迟回调
void BMI270Read::delay_us(uint32_t period, void *intf_ptr) {
  delayMicroseconds(period);
}

// 初始化BMI270设备结构
void BMI270Read::init_bmi_dev() {
  // 设置接口参数
  bmi.intf = BMI2_I2C_INTF;
  bmi.read = i2c_read;
  bmi.write = i2c_write;
  bmi.delay_us = delay_us;
  bmi.intf_ptr = &dev_addr;
  bmi.read_write_len = 32; // 最大读写长度
  bmi.resolution = 16;     // 分辨率
}

// 读取芯片ID
int8_t BMI270Read::read_chip_id(uint8_t *chip_id) {
  Wire.beginTransmission(dev_addr);
  Wire.write(BMI2_CHIP_ID_ADDR);
  if (Wire.endTransmission(false) != 0) {
    return 1;
  }
  
  if (Wire.requestFrom(dev_addr, (uint8_t)1) != 1) {
    return 1;
  }
  
  *chip_id = Wire.read();
  return 0;
}

// 配置加速度计和陀螺仪
int8_t BMI270Read::set_accel_gyro_config() {
  int8_t rslt;
  struct bmi2_sens_config config[2];
  
  // 配置加速度计和陀螺仪
  config[0].type = BMI2_ACCEL;
  config[1].type = BMI2_GYRO;
  
  // 获取默认配置
  rslt = bmi2_get_sensor_config(config, 2, &bmi);
  if (rslt != BMI2_OK) return rslt;
  
  // 映射数据就绪中断到INT1引脚
  rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &bmi);
  if (rslt != BMI2_OK) return rslt;
  
  // 配置加速度计
  config[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;      // 100Hz
  config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;      // ±2G范围
  config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;     // 平均4个样本
  config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE; // 高性能模式
  
  // 配置陀螺仪
  config[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;        // 100Hz
  config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;      // ±2000dps范围
  config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;       // 正常模式
  config[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE; // 低功耗模式
  config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE; // 高性能模式
  
  // 设置配置
  rslt = bmi2_set_sensor_config(config, 2, &bmi);
  
  return rslt;
}

// LSB转换为m/s²
float BMI270Read::lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width) {
  float half_scale = (float)((1 << bit_width) / 2.0f);
  return (GRAVITY_EARTH * val * g_range) / half_scale;
}

// LSB转换为°/s
float BMI270Read::lsb_to_dps(int16_t val, float dps, uint8_t bit_width) {
  float half_scale = (float)((1 << bit_width) / 2.0f);
  return (dps / half_scale) * val;
}

// 初始化BMI270
bool BMI270Read::begin() {
  // 初始化I2C（如果需要）
  if (!skip_i2c_init) {
    if (sda_pin >= 0 && scl_pin >= 0) {
      Wire.begin(sda_pin, scl_pin);
    } else {
      Wire.begin();
    }
    Wire.setClock(400000); // I2C时钟速率
  }
  
  // 初始化BMI270设备结构
  init_bmi_dev();
  
  // 读取芯片ID
  uint8_t chip_id = 0;
  int8_t rslt = read_chip_id(&chip_id);
  
  if (rslt != 0 || chip_id != BMI270_CHIP_ID) {
    // 尝试另一个地址
    dev_addr = (dev_addr == BMI270_I2C_ADDR_PRIMARY) ? 
                BMI270_I2C_ADDR_SECONDARY : BMI270_I2C_ADDR_PRIMARY;
    rslt = read_chip_id(&chip_id);
    
    if (rslt != 0 || chip_id != BMI270_CHIP_ID) {
      return false; // 初始化失败
    }
  }
  
  // 初始化BMI270
  rslt = bmi270_init(&bmi);
  if (rslt != BMI2_OK) {
    return false;
  }
  
  // 配置加速度计和陀螺仪
  rslt = set_accel_gyro_config();
  if (rslt != BMI2_OK) {
    return false;
  }
  
  // 启用传感器
  uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };
  rslt = bmi2_sensor_enable(sensor_list, 2, &bmi);
  if (rslt != BMI2_OK) {
    return false;
  }
  
  initialized = true;
  return true;
}

// 扫描I2C总线
int BMI270Read::scanI2C() {
  byte error, address;
  int deviceCount = 0;
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      deviceCount++;
    }
  }
  
  return deviceCount;
}

// 读取传感器数据
SensorData BMI270Read::readSensorData() {
  SensorData data = {0, 0, 0, 0, 0, 0, false};
  
  if (!initialized) {
    return data;
  }
  
  struct bmi2_sens_data sensor_data = { { 0 } };
  int8_t rslt = bmi2_get_sensor_data(&sensor_data, &bmi);
  
  if (rslt == BMI2_OK && (sensor_data.status & BMI2_DRDY_ACC) && (sensor_data.status & BMI2_DRDY_GYR)) {
    // 转换加速度数据（2G范围）
    data.acc_x = lsb_to_mps2(sensor_data.acc.x, 2.0f, bmi.resolution);
    data.acc_y = lsb_to_mps2(sensor_data.acc.y, 2.0f, bmi.resolution);
    data.acc_z = lsb_to_mps2(sensor_data.acc.z, 2.0f, bmi.resolution);
    
    // 转换陀螺仪数据（2000dps范围）
    data.gyr_x = lsb_to_dps(sensor_data.gyr.x, 2000.0f, bmi.resolution);
    data.gyr_y = lsb_to_dps(sensor_data.gyr.y, 2000.0f, bmi.resolution);
    data.gyr_z = lsb_to_dps(sensor_data.gyr.z, 2000.0f, bmi.resolution);
    
    data.valid = true;
  }
  
  return data;
} 