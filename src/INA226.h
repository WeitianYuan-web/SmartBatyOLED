#ifndef INA226_H
#define INA226_H

#include <Arduino.h>
#include <Wire.h>

// INA226寄存器地址
#define INA226_CONFIG_REG      0x00  // 配置寄存器
#define INA226_SHUNT_VOLT_REG  0x01  // 分流电压寄存器
#define INA226_BUS_VOLT_REG    0x02  // 总线电压寄存器
#define INA226_POWER_REG       0x03  // 功率寄存器
#define INA226_CURRENT_REG     0x04  // 电流寄存器
#define INA226_CALIB_REG       0x05  // 校准寄存器

// 滤波参数
#define INA226_FILTER_SIZE 5      // 滤波窗口大小
#define INA226_ALPHA 0.8          // 指数滤波平滑因子 (0-1)

class INA226 {
public:
    // 构造函数
    INA226(uint8_t addr = 0x40, float shuntResistor = 0.005);
    
    // 初始化
    bool begin(float maxExpectedCurrent = 6.0);
    
    // 读取函数
    float readBusVoltage();       // 读取总线电压 (V)
    float readShuntVoltage();     // 读取分流电压 (V)
    float readCurrent();          // 读取电流 (A)
    float readPower();            // 读取功率 (W)
    
    // 带滤波的读取函数
    float readBusVoltageFiltered();  // 读取总线电压 (V)，带滤波
    float readCurrentFiltered();     // 读取电流 (A)，带滤波
    float readPowerFiltered();       // 读取功率 (W)，带滤波
    
    // 设置偏移补偿
    void setCurrentOffset(float offset) { currentOffset = offset; }
    void setPowerOffset(float offset) { powerOffset = offset; }
    
    // 设置滤波参数
    void setFilterAlpha(float alpha) { filterAlpha = alpha; }

private:
    // 寄存器操作
    void writeRegister(uint8_t reg, uint16_t value);
    uint16_t readRegister(uint8_t reg);
    
    // 指数加权移动平均滤波函数
    float exponentialFilter(float newValue, float lastValue);
    
    // 设备地址
    uint8_t deviceAddress;
    
    // 分流电阻值
    float shuntResistor;
    
    // 转换参数
    float currentLSB;
    float powerLSB;
    
    // 偏移补偿
    float currentOffset;
    float powerOffset;
    
    // 滤波参数
    float filterAlpha;
    
    // 上一次的滤波值
    float lastBusVoltage;
    float lastCurrent;
    float lastPower;
};

#endif 