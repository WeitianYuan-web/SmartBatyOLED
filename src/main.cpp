#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>

#define EN 10
// INA226相关定义
#define INA226_ADDR 0x40

// INA226寄存器地址
#define CONFIG_REG      0x00  // 配置寄存器
#define SHUNT_VOLT_REG  0x01  // 分流电压寄存器
#define BUS_VOLT_REG    0x02  // 总线电压寄存器
#define POWER_REG       0x03  // 功率寄存器
#define CURRENT_REG     0x04  // 电流寄存器
#define CALIB_REG       0x05  // 校准寄存器

// INA226全局变量
float shuntResistor = 0.005; // 分流电阻值，单位：欧姆（5毫欧）
float currentLSB;            // 电流LSB值
float powerLSB;              // 功率LSB值

// INA226函数声明
void initINA226();
float readBusVoltage();
float readShuntVoltage();
float readCurrent();
float readPower();
void writeRegister(uint8_t reg, uint16_t value);
uint16_t readRegister(uint8_t reg);

// OLED显示器类
class OLEDDisplay {
private:
  U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2;
  float batteryVoltage;
  int batteryCells;
  float current;
  String deviceID;
  String deviceIP;
  bool wifiConnected;
  int chipTemperature;
  
  // 互斥锁，用于保护显示数据
  SemaphoreHandle_t displayMutex;
  
  // 计算电池电量百分比
  int8_t calculateBatteryPercentage(float voltage, int cells) {
    float nominalVoltage, maxVoltage;
    
    switch(cells) {
      case 1:  // 1S
        nominalVoltage = 3.7f;  // 标称电压（0%）
        maxVoltage = 4.2f;      // 充满电压（100%）
        break;
      case 2:  // 2S
        nominalVoltage = 7.4f;
        maxVoltage = 8.4f;
        break;
      case 3:  // 3S
        nominalVoltage = 11.1f;
        maxVoltage = 12.6f;
        break;
      case 4:  // 4S
        nominalVoltage = 14.8f;
        maxVoltage = 16.8f;
        break;
      default:
        return 0;
    }

    // 计算百分比（相对于标称电压）
    float percentage = ((voltage - nominalVoltage) / (maxVoltage - nominalVoltage)) * 100.0f;
    
    // 限制百分比范围为-100到100
    if (percentage < -100) percentage = -100;
    if (percentage > 100) percentage = 100;

    return (int8_t)percentage;
  }

  // 绘制电池图标和S数
  void drawBattery(uint8_t x, uint8_t y, float voltage, int cells) {
    int8_t percentage = calculateBatteryPercentage(voltage, cells);
    
    // 电池外框
    u8g2.drawFrame(x, y, 12, 6);
    // 电池正极
    u8g2.drawBox(x + 12, y + 2, 2, 2);
    
    // 只在电量为正值时显示填充
    if (percentage > 0) {
      uint8_t width = (percentage * 10) / 100;
      if (width > 0) {
        u8g2.drawBox(x + 1, y + 1, width, 4);
      }
    }
    
    // 当电量为负值时闪烁显示
    if (percentage < 0 && (millis() / 500) % 2) {
      u8g2.drawBox(x + 1, y + 1, 2, 4);
    }

    // 在电池图标下方显示电池类型
    u8g2.setFont(u8g2_font_5x7_tf);  // 使用小号字体
    u8g2.setCursor(x + 2, y + 13);    // 位置在电池图标下方
    u8g2.print(cells);
    u8g2.print("S");
  }

  // 绘制闪电图标
  void drawLightning(uint8_t x, uint8_t y) {
    u8g2.drawLine(x, y, x + 3, y + 3);
    u8g2.drawLine(x + 3, y + 3, x, y + 6);
    u8g2.drawPixel(x + 1, y + 3);
  }

  // 绘制WiFi图标
  void drawWiFi(uint8_t x, uint8_t y) {
    // 根据连接状态决定是否闪烁
    if (wifiConnected || (millis() / 500) % 2) {
      u8g2.drawCircle(x, y, 6, U8G2_DRAW_UPPER_RIGHT);
      u8g2.drawCircle(x, y, 4, U8G2_DRAW_UPPER_RIGHT);
      u8g2.drawCircle(x, y, 2, U8G2_DRAW_UPPER_RIGHT);
    }
  }

  // 计算功率
  float calculatePower(float voltage, float current) {
    return voltage * current;
  }

  // 绘制温度图标
  void drawTemperature(uint8_t x, uint8_t y) {
    // 简单的温度计图标
    u8g2.drawCircle(x + 2, y + 5, 2);  // 温度计底部圆形
    u8g2.drawVLine(x + 2, y, 4);       // 温度计柱体
  }

public:
  OLEDDisplay() : u8g2(U8G2_R0), 
                  batteryVoltage(0.0),
                  batteryCells(0),
                  current(0.0),
                  deviceID(""),
                  deviceIP("0.0.0.0"),
                  wifiConnected(false),
                  chipTemperature(0) {
    displayMutex = xSemaphoreCreateMutex();
  }

  // 初始化显示器
  bool begin() {
    if (!u8g2.begin()) {
      return false;
    }
    u8g2.enableUTF8Print();
    return true;
  }

  // 线程安全的数据更新方法
  void updateData(float voltage, int cells, float amp, const String& id, const String& ip, bool wifiStatus, float temp) {
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      batteryVoltage = voltage;
      batteryCells = cells;
      current = amp;
      deviceID = id;
      deviceIP = ip;
      wifiConnected = wifiStatus;
      chipTemperature = temp;
      xSemaphoreGive(displayMutex);
    }
  }

  // 线程安全的显示刷新方法
  void refresh() {
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      u8g2.clearBuffer();
      
      // 绘制边框
      u8g2.drawFrame(0, 0, 128, 32);
      u8g2.drawHLine(0, 19, 128);
      u8g2.drawVLine(50, 0, 19);  // 第一个分隔线
      u8g2.drawVLine(85, 0, 19);  // 第二个分隔线
      u8g2.drawVLine(103, 19, 13);  // 第二个分隔线

      // 左侧电压显示
      drawBattery(2, 2, batteryVoltage, batteryCells);
      u8g2.setFont(u8g2_font_7x13B_tr);
      u8g2.setCursor(15, 14);
      u8g2.print(batteryVoltage, 1);
      u8g2.setFont(u8g2_font_5x7_tf);
      u8g2.setCursor(44, 14);
      u8g2.print("V");

      // 中间功率显示
      float power = calculatePower(batteryVoltage, current);
      u8g2.setFont(u8g2_font_6x10_tf);  // 使用更小的字体显示功率
      u8g2.setCursor(52, 14);
      u8g2.print(power, 1);
      u8g2.setFont(u8g2_font_5x7_tf);
      u8g2.setCursor(78, 14);
      u8g2.print("W");

      // 右侧电流显示
      drawLightning(87, 6);
      u8g2.setFont(u8g2_font_7x13B_tr);
      u8g2.setCursor(95, 14);  // 右移电流显示位置
      u8g2.print(current, 1);
      u8g2.setFont(u8g2_font_5x7_tf);
      u8g2.setCursor(120, 14);
      u8g2.print("A");

      // 底部信息
      u8g2.setFont(u8g2_font_5x8_tf);
      drawWiFi(4, 28);
      u8g2.setCursor(12, 30);
      u8g2.print(deviceID);
      u8g2.print(" ");
      u8g2.print(deviceIP);
      
      // 在底部信息区域添加温度显示
      //drawTemperature(90, 22);
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.setCursor(105, 30);
      u8g2.print(chipTemperature, 1);
      u8g2.print("C");
      
      u8g2.sendBuffer();
      xSemaphoreGive(displayMutex);
    }
  }

  // 更新WiFi状态和IP
  void updateWiFiStatus(bool connected, const String& ip) {
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      wifiConnected = connected;
      if (connected) {
        deviceIP = ip;
      }
      xSemaphoreGive(displayMutex);
    }
  }
};

// 创建全局显示器实例
OLEDDisplay display;

// 显示任务
void displayTask(void *parameter) {
  while (1) {
    display.refresh();
    vTaskDelay(pdMS_TO_TICKS(100));  // 每100ms刷新一次显示
  }
}

// 全局变量
String deviceID = "A01";  // 默认设备ID
const char* ssid = "CMCC-JM3A";  // 默认WiFi SSID
const char* password = "18771407258";  // 默认WiFi密码
bool enState = false;  // EN引脚状态



// 处理用户命令
void processCommand(const String& command) {
    if (command.startsWith("en:")) {
        String value = command.substring(3);
        value.trim();
        
        if (value == "on") {
            enState = true;
            digitalWrite(EN, HIGH);
            Serial.println("EN引脚已打开");
        } else if (value == "off") {
            enState = false;
            digitalWrite(EN, LOW);
            Serial.println("EN引脚已关闭");
        } else {
            Serial.println("无效的EN命令，使用 on 或 off");
        }
    }
    else if (command.startsWith("id:")) {
        String newID = command.substring(3);
        newID.trim();
        
        if (newID.length() > 0) {
            deviceID = newID;
            Serial.print("设备ID已更新为: ");
            Serial.println(deviceID);
        } else {
            Serial.println("无效的ID");
        }
    }
    else if (command.startsWith("wifi:")) {
        String wifiConfig = command.substring(5);
        int commaPos = wifiConfig.indexOf(',');
        
        if (commaPos > 0) {
            String newSSID = wifiConfig.substring(0, commaPos);
            String newPassword = wifiConfig.substring(commaPos + 1);
            
            newSSID.trim();
            newPassword.trim();
            
            if (newSSID.length() > 0 && newPassword.length() > 0) {
                // 更新WiFi设置
                ssid = strdup(newSSID.c_str());
                password = strdup(newPassword.c_str());
                
                Serial.println("WiFi设置已更新，重新连接中...");
                WiFi.disconnect();
                WiFi.begin(ssid, password);
            } else {
                Serial.println("无效的WiFi设置");
            }
        } else {
            Serial.println("无效的WiFi命令格式，使用 wifi:ssid,password");
        }
    }
    else if (command == "status") {
        Serial.println("当前状态:");
        Serial.print("  设备ID: ");
        Serial.println(deviceID);
        Serial.print("  WiFi SSID: ");
        Serial.println(ssid);
        Serial.print("  EN状态: ");
        Serial.println(enState ? "ON" : "OFF");
        Serial.print("  WiFi状态: ");
        Serial.println(WiFi.status() == WL_CONNECTED ? "已连接" : "未连接");
        if (WiFi.status() == WL_CONNECTED) {
            Serial.print("  IP地址: ");
            Serial.println(WiFi.localIP().toString());
        }
        Serial.print("  电压: ");
        Serial.print(readBusVoltage(), 2);
        Serial.println(" V");
        Serial.print("  电流: ");
        Serial.print(readCurrent(), 3);
        Serial.println(" A");
        Serial.print("  功率: ");
        Serial.print(readPower(), 2);
        Serial.println(" W");
        Serial.print("  温度: ");
        Serial.print(temperatureRead(), 1);
        Serial.println(" °C");
    }
    else {
        Serial.println("未知命令");
    }
}

// 修改BatteryMonitor类以使用全局deviceID
class BatteryMonitor {
private:
    String currentIP = "0.0.0.0";
    bool isWiFiConnected = false;
    
    // 获取芯片温度
    float getChipTemperature() {
        // 实际读取ESP32内部温度传感器
        return temperatureRead();
    }

public:
    struct SensorData {
        float voltage;
        float current;
        int cells;
        String deviceID;
        String deviceIP;
        bool wifiConnected;
        int temperature;
    };

    // 更新WiFi状态和IP
    void updateWiFiStatus(bool connected, const String& ip) {
        isWiFiConnected = connected;
        if (connected) {
            currentIP = ip;
        }
    }

    // 获取实际数据
    SensorData getData() {
        SensorData data;
        
        // 从INA226读取实际电压和电流
        data.voltage = readBusVoltage();
        data.current = readCurrent();
        
        // 根据电压自动判断电池节数
        if (data.voltage < 4.5) {
            data.cells = 1;  // 1S
        } else if (data.voltage < 9.0) {
            data.cells = 2;  // 2S
        } else if (data.voltage < 13.0) {
            data.cells = 3;  // 3S
        } else {
            data.cells = 4;  // 4S
        }
        
        // 使用全局deviceID
        data.deviceID = deviceID;
        data.deviceIP = currentIP;
        data.wifiConnected = isWiFiConnected;

        // 添加温度数据
        data.temperature = getChipTemperature();

        return data;
    }
};

// 创建全局实例
BatteryMonitor batteryMonitor;

// 用户控制任务
void userControlTask(void *parameter) {
    String inputBuffer = "";
    
    Serial.println("用户控制任务启动");
    Serial.println("可用命令:");
    Serial.println("  en:on - 打开EN引脚");
    Serial.println("  en:off - 关闭EN引脚");
    Serial.println("  id:xxx - 设置设备ID为xxx");
    Serial.println("  wifi:ssid,password - 设置WiFi");
    Serial.println("  status - 显示当前状态");
    
    pinMode(EN, OUTPUT);
    digitalWrite(EN, enState ? HIGH : LOW);
    
    while (1) {
        if (Serial.available()) {
            char c = Serial.read();
            
            // 回显字符
            if (c >= 32 && c <= 126) {  // 可打印字符
                Serial.print(c);
            }
            
            if (c == '\n' || c == '\r') {
                if (inputBuffer.length() > 0) {
                    Serial.println();
                    processCommand(inputBuffer);
                    inputBuffer = "";
                }
            } else {
                inputBuffer += c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // 短暂延时避免占用过多CPU
    }
}

// 修改数据更新任务
void updateTask(void *parameter) {
    while (1) {
        BatteryMonitor::SensorData data = batteryMonitor.getData();
        
        display.updateData(
            data.voltage,
            data.cells,
            data.current,
            data.deviceID,
            data.deviceIP,
            data.wifiConnected,
            data.temperature
        );
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}



// WiFi事件处理
void WiFiEvent(WiFiEvent_t event) {
    switch(event) {
        case SYSTEM_EVENT_STA_GOT_IP:
            batteryMonitor.updateWiFiStatus(true, WiFi.localIP().toString());
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            batteryMonitor.updateWiFiStatus(false, "Disconnected");
            WiFi.reconnect();
            break;
        case SYSTEM_EVENT_STA_START:
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            break;
        default:
            break;
    }
}

// WiFi连接任务
void wifiTask(void *parameter) {
    // 注册WiFi事件处理函数
    WiFi.onEvent(WiFiEvent);
    
    // 设置WiFi模式
    WiFi.mode(WIFI_STA);
    
    // 开始连接WiFi
    WiFi.begin(ssid, password);
    
    while (1) {
        if (WiFi.status() != WL_CONNECTED) {
            // 如果断开连接，尝试重连
            WiFi.reconnect();
            batteryMonitor.updateWiFiStatus(false, "Connecting...");
        } else {
            // 确保IP地址始终是最新的
            batteryMonitor.updateWiFiStatus(true, WiFi.localIP().toString());
        }
        vTaskDelay(pdMS_TO_TICKS(5000));  // 每5秒检查一次连接状态
    }
}

void setup() {
    Serial.begin(115200);
    
    // 初始化EN引脚
    pinMode(EN, OUTPUT);
    digitalWrite(EN, enState ? HIGH : LOW);
    
    // 初始化INA226
    initINA226();
    
    if (!display.begin()) {
        Serial.println("OLED初始化失败!");
        while (1);  // 如果OLED初始化失败，停止运行
    }

    // 创建用户控制任务 - 在核心0上运行
    xTaskCreatePinnedToCore(
        userControlTask,
        "UserControlTask",
        4096,
        NULL,
        1,
        NULL,
        0           // 在核心0上运行
    );

    // 创建WiFi任务 - 在核心0上运行
    xTaskCreatePinnedToCore(
        wifiTask,
        "WiFiTask",
        8192,        // 8K堆栈
        NULL,
        1,
        NULL,
        0           // 在核心0上运行
    );

    // 创建显示任务 - 在核心0上运行
    xTaskCreatePinnedToCore(
        displayTask,
        "DisplayTask",
        4096,
        NULL,
        2,
        NULL,
        0           // 在核心0上运行
    );

    // 创建数据更新任务 - 在核心0上运行
    xTaskCreatePinnedToCore(
        updateTask,
        "UpdateTask",
        4096,
        NULL,
        1,
        NULL,
        0           // 在核心0上运行
    );
}

void loop() {
  // 在FreeRTOS中，loop()可以为空
  vTaskDelete(NULL);  // 删除setup/loop任务
}

// INA226函数实现
void initINA226() {
    // 设置校准值
    // 最大预期电流，设置为6A
    float maxExpectedCurrent = 6.0;
    
    // 计算电流LSB (最大电流/2^15)
    currentLSB = maxExpectedCurrent / 32768;
    
    // 计算功率LSB (25倍电流LSB)
    powerLSB = currentLSB * 25;
    
    // 计算校准寄存器值
    uint16_t calibrationValue = (uint16_t)((0.00512) / (currentLSB * shuntResistor));
    
    // 写入校准值
    writeRegister(CALIB_REG, calibrationValue);
    
    // 配置INA226
    // 设置配置寄存器：
    // - 16次平均采样
    // - 1.1ms转换时间
    // - 连续测量分流和总线电压
    uint16_t config = 0x4127;
    writeRegister(CONFIG_REG, config);
    
    Serial.print("校准值: ");
    Serial.println(calibrationValue);
    Serial.print("电流LSB: ");
    Serial.print(currentLSB * 1000000, 3);
    Serial.println(" uA/bit");
}

// 读取总线电压 (V)
float readBusVoltage() {
    uint16_t value = readRegister(BUS_VOLT_REG);
    return value * 0.00125; // LSB = 1.25mV
}

// 读取分流电压 (V)
float readShuntVoltage() {
    int16_t value = readRegister(SHUNT_VOLT_REG);
    return value * 0.0000025; // LSB = 2.5uV
}

// 读取电流 (A)
float readCurrent() {
    int16_t value = readRegister(CURRENT_REG);
    return value * currentLSB;
}

// 读取功率 (W)
float readPower() {
    uint16_t value = readRegister(POWER_REG);
    return value * powerLSB;
}

// 写入寄存器
void writeRegister(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(INA226_ADDR);
    Wire.write(reg);
    Wire.write((value >> 8) & 0xFF);  // 高字节
    Wire.write(value & 0xFF);         // 低字节
    Wire.endTransmission();
}

// 读取寄存器
uint16_t readRegister(uint8_t reg) {
    Wire.beginTransmission(INA226_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(INA226_ADDR, 2);
    uint16_t value = Wire.read() << 8;  // 高字节
    value |= Wire.read();               // 低字节
    return value;
}