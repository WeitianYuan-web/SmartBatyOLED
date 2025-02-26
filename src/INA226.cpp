#include "INA226.h"

// 构造函数
INA226::INA226(uint8_t addr, float shuntRes) {
    deviceAddress = addr;
    shuntResistor = shuntRes;
    currentOffset = 0;
    powerOffset = 0;
    filterAlpha = INA226_ALPHA;
    lastBusVoltage = 0;
    lastCurrent = 0;
    lastPower = 0;
}

// 初始化
bool INA226::begin(float maxExpectedCurrent) {
    // 计算电流LSB (最大电流/2^15)
    currentLSB = maxExpectedCurrent / 32768;
    
    // 计算功率LSB (25倍电流LSB)
    powerLSB = currentLSB * 25;
    
    // 计算校准寄存器值
    uint16_t calibrationValue = (uint16_t)((0.00512) / (currentLSB * shuntResistor));
    
    // 写入校准值
    writeRegister(INA226_CALIB_REG, calibrationValue);
    
    // 配置INA226
    // 设置配置寄存器：
    // - 16次平均采样
    // - 1.1ms转换时间
    // - 连续测量分流和总线电压
    uint16_t config = 0x4127;
    writeRegister(INA226_CONFIG_REG, config);
    
    // 检查通信是否正常
    uint16_t configRead = readRegister(INA226_CONFIG_REG);
    return (configRead == config);
}

// 读取总线电压 (V)
float INA226::readBusVoltage() {
    uint16_t value = readRegister(INA226_BUS_VOLT_REG);
    return value * 0.00125; // LSB = 1.25mV
}

// 读取分流电压 (V)
float INA226::readShuntVoltage() {
    int16_t value = readRegister(INA226_SHUNT_VOLT_REG);
    return value * 0.0000025; // LSB = 2.5uV
}

// 读取电流 (A)
float INA226::readCurrent() {
    int16_t value = readRegister(INA226_CURRENT_REG);
    return value * currentLSB - currentOffset; // 减去偏移值
}

// 读取功率 (W)
float INA226::readPower() {
    uint16_t value = readRegister(INA226_POWER_REG);
    return value * powerLSB - powerOffset; // 减去偏移值
}

// 指数加权移动平均滤波函数
float INA226::exponentialFilter(float newValue, float lastValue) {
    return filterAlpha * newValue + (1 - filterAlpha) * lastValue;
}

// 读取总线电压 (V)，带滤波
float INA226::readBusVoltageFiltered() {
    float rawValue = readBusVoltage();
    lastBusVoltage = exponentialFilter(rawValue, lastBusVoltage);
    return lastBusVoltage;
}

// 读取电流 (A)，带滤波
float INA226::readCurrentFiltered() {
    float rawValue = readCurrent();
    lastCurrent = exponentialFilter(rawValue, lastCurrent);
    return lastCurrent;
}

// 读取功率 (W)，带滤波
float INA226::readPowerFiltered() {
    float rawValue = readPower();
    lastPower = exponentialFilter(rawValue, lastPower);
    return lastPower;
}

// 写入寄存器
void INA226::writeRegister(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(reg);
    Wire.write((value >> 8) & 0xFF);  // 高字节
    Wire.write(value & 0xFF);         // 低字节
    Wire.endTransmission();
}

// 读取寄存器
uint16_t INA226::readRegister(uint8_t reg) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom((uint8_t)deviceAddress, (uint8_t)2);
    uint16_t value = Wire.read() << 8;  // 高字节
    value |= Wire.read();               // 低字节
    return value;
} 