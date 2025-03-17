#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WebSocketsClient.h> // 添加WebSocket客户端库
#include <ArduinoJson.h>      // 添加JSON库
#include "INA226.h"  // 添加INA226头文件

// 添加宏定义控制BMI270功能
#define ENABLE_BMI270 0  // 设置为1启用BMI270功能，设置为0禁用

#if ENABLE_BMI270
#include "BMI270Read.h"
#include "KalmanFilter.h"
#endif

#define EN_GPIO_NUM 10 // 确认ESP32-C3的GPIO10可用

// INA226相关定义
#define INA226_ADDR 0x40

// INA226寄存器地址
#define CONFIG_REG      0x00  // 配置寄存器
#define SHUNT_VOLT_REG  0x01  // 分流电压寄存器
#define BUS_VOLT_REG    0x02  // 总线电压寄存器
#define POWER_REG       0x03  // 功率寄存器
#define CURRENT_REG     0x04  // 电流寄存器
#define CALIB_REG       0x05  // 校准寄存器
// 定义引脚
#define SDA_PIN 8
#define SCL_PIN 9

// 全局变量
String deviceID = "A01";  // 默认设备ID
//const char* ssid = "Xiaomi_B1F1";  // 默认WiFi SSID
const char* ssid = "Magic5";  // 默认WiFi SSID
const char* password = "yuqing051017";  // 默认WiFi密码
bool  enState = true;  // EN引脚状态


// WebSocket相关配置
const char* wsHost = "192.168.28.44"; // WebSocket服务器地址，需要配置
uint16_t wsPort = 8080;              // WebSocket服务器端口
const char* wsPath = "/";            // WebSocket路径
const char* API_KEY = "sensor_device_key_123"; // 认证密钥
const char* BATTERY_ID = "smartbaty-001";    // 电池ID
unsigned long lastWsSendTime = 0;    // 上次发送数据的时间
const int WS_SEND_INTERVAL = 200;   // 发送间隔(毫秒)
bool isWsConnected = false;          // WebSocket连接状态
bool isWsAuthenticated = false;      // WebSocket认证状态
unsigned long lastPingTime = 0;      // 上次发送ping的时间
const int PING_INTERVAL = 30000;     // ping发送间隔(毫秒，30秒)

// 打印相关变量
unsigned long lastDataPrintTime = 0;    // 上次打印电池数据的时间
unsigned long lastCommandPrintTime = 0; // 上次打印命令列表的时间
const int DATA_PRINT_INTERVAL = 10000;  // 打印数据间隔(毫秒，10秒)
const int COMMAND_PRINT_INTERVAL = 10000; // 打印命令间隔(毫秒，10秒)
bool firstDataSent = true;              // 首次发送数据标志
char printBuffer[30] = {0};             // 打印缓冲区
int printCounter = 0;                   // 打印计数器

// 电源命令管理变量
bool hasPendingPowerCommand = false;  // 是否有未处理的电源命令
bool pendingPowerState = false;       // 未处理的电源状态
unsigned long powerCommandTime = 0;   // 计划执行时间

// 创建WebSocket客户端
WebSocketsClient webSocket;

// 函数前向声明
void sendAuthRequest();
void sendBatteryData();
void sendPowerStateResponse(bool powerState);
String getISOTimestamp();
void printCommandList();  // 新增函数声明

// 创建BMI270读取对象和卡尔曼滤波器
#if ENABLE_BMI270
BMI270Read bmi270(SDA_PIN, SCL_PIN);
KalmanFilter kalman;
#endif

// INA226全局变量
float shuntResistor = 0.005; // 分流电阻值，单位：欧姆（5毫欧）
float currentLSB;            // 电流LSB值
float powerLSB;              // 功率LSB值

// 创建INA226实例
INA226 ina226(0x40, 0.005);  // 地址0x40，分流电阻0.005欧姆

// OLED显示器类
class OLEDDisplay {
private:
  U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2;
  float batteryVoltage;
  int batteryCells;
  float current;
  float batteryPower;
  String deviceID;
  String deviceIP;
  bool wifiConnected;
  int chipTemperature;
  
  // 添加BMI270数据成员
  #if ENABLE_BMI270
  float pitch;
  float roll;
  float yaw;
  #endif
  
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
                  batteryPower(0.0),
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
  void updateData(float voltage, int cells, float amp, float power, const String& id, const String& ip, bool wifiStatus, float temp) {
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      batteryVoltage = voltage;
      batteryCells = cells;
      current = amp;
      batteryPower = power;
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
      
      // 左侧电压显示 - 无论EN状态如何都显示
      drawBattery(2, 2, batteryVoltage, batteryCells);
      u8g2.setFont(u8g2_font_7x13B_tr);
      u8g2.setCursor(15, 14);
      u8g2.print(batteryVoltage, 1);
      u8g2.setFont(u8g2_font_5x7_tf);
      u8g2.setCursor(44, 14);
      u8g2.print("V");

      if (enState) {
        // 正常显示（EN开启状态）
        u8g2.drawVLine(50, 0, 19);  // 第一个分隔线
        u8g2.drawVLine(85, 0, 19);  // 第二个分隔线
        u8g2.drawVLine(103, 19, 13);  // 底部分隔线
        
        // 中间功率显示
        u8g2.setFont(u8g2_font_6x10_tf);  // 使用更小的字体显示功率
        u8g2.setCursor(52, 14);
        // 当功率小于0.1时显示0
        float displayPower = (batteryPower < 0.1) ? 0.0 : batteryPower;
        u8g2.print(displayPower, 1);
        u8g2.setFont(u8g2_font_5x7_tf);
        u8g2.setCursor(78, 14);
        u8g2.print("W");

        // 右侧电流显示
        drawLightning(87, 6);
        u8g2.setFont(u8g2_font_7x13B_tr);
        u8g2.setCursor(95, 14);  // 右移电流显示位置
        
        // 当电流绝对值小于0.1时显示0
        float displayCurrent = (abs(current) < 0.1) ? 0.0 : current;
        
        // 根据电流大小调整显示格式
        if (abs(displayCurrent) < 1.0 ) {
          // 小于1A时，显示为".XX"格式
          char currentStr[10];
          int decimalPart = abs(displayCurrent) * 100;
          sprintf(currentStr, ".%02d", decimalPart);
          u8g2.print(currentStr);
        } else {
          // 大于等于1A或等于0时，正常显示
          u8g2.print(displayCurrent, 1);
        }
        
        u8g2.setFont(u8g2_font_5x7_tf);
        u8g2.setCursor(121, 14);
        u8g2.print("A");
      } else {
        // 特殊显示（EN关闭状态）
        // 用一个明显的标识表示电源已关闭
        u8g2.setFont(u8g2_font_7x13B_tr);
        u8g2.setCursor(58, 14);
        u8g2.print("5VEN:OFF");
      }

      // 底部信息 - 无论EN状态如何都显示
      u8g2.setFont(u8g2_font_5x8_tf);
      drawWiFi(4, 28);
      u8g2.setCursor(12, 30);
      u8g2.print(deviceID);
      u8g2.print(" ");
      u8g2.print(deviceIP);
      
      // 在底部信息区域添加温度显示
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

  // 添加更新姿态数据的方法
  #if ENABLE_BMI270
  void updateAttitude(float newPitch, float newRoll, float newYaw) {
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      pitch = newPitch;
      roll = newRoll;
      yaw = newYaw;
      xSemaphoreGive(displayMutex);
    }
  }
  #endif
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


// 处理用户命令
void processCommand(const String& command) {
    if (command.startsWith("en:")) {
        String value = command.substring(3);
        value.trim();
        
        if (value == "on") {
            enState = true;
            Serial.println("EN状态已设为开启");
        } else if (value == "off") {
            enState = false;
            Serial.println("EN状态已设为关闭");
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
    else if (command.startsWith("ws:")) {
        String wsConfig = command.substring(3);
        int colonPos = wsConfig.indexOf(':');
        
        if (colonPos > 0) {
            String newHost = wsConfig.substring(0, colonPos);
            String newPort = wsConfig.substring(colonPos + 1);
            
            newHost.trim();
            newPort.trim();
            
            if (newHost.length() > 0 && newPort.length() > 0) {
                // 更新WebSocket设置
                wsHost = strdup(newHost.c_str());
                wsPort = newPort.toInt();
                
                Serial.println("WebSocket设置已更新，重新连接中...");
                webSocket.disconnect();
                webSocket.begin(wsHost, wsPort, wsPath);
            } else {
                Serial.println("无效的WebSocket设置");
            }
        } else {
            Serial.println("无效的WebSocket命令格式，使用 ws:host:port");
        }
    }
    else if (command.startsWith("wskey:")) {
        String newKey = command.substring(6);
        newKey.trim();
        
        if (newKey.length() > 0) {
            // 更新WebSocket API密钥
            API_KEY = strdup(newKey.c_str());
            
            Serial.println("WebSocket API密钥已更新，请重新连接认证");
            isWsAuthenticated = false;
            
            // 如果已连接，重新发送认证请求
            if (isWsConnected) {
                sendAuthRequest();
            }
        } else {
            Serial.println("无效的API密钥");
        }
    }
    else if (command.startsWith("battery_id:")) {
        String newID = command.substring(11);
        newID.trim();
        
        if (newID.length() > 0) {
            // 更新电池ID
            BATTERY_ID = strdup(newID.c_str());
            
            Serial.print("电池ID已更新为: ");
            Serial.println(BATTERY_ID);
            
            // 如果已认证，需要重新认证
            if (isWsAuthenticated) {
                isWsAuthenticated = false;
                if (isWsConnected) {
                    sendAuthRequest();
                }
            }
        } else {
            Serial.println("无效的电池ID");
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
        Serial.print(ina226.readBusVoltage(), 2);  // 使用INA226类方法
        Serial.println(" V");
        Serial.print("  电流: ");
        Serial.print(ina226.readCurrent(), 3);  // 使用INA226类方法
        Serial.println(" A");
        Serial.print("  功率: ");
        Serial.print(ina226.readPower(), 2);  // 使用INA226类方法
        Serial.println(" W");
        Serial.print("  温度: ");
        Serial.print(temperatureRead(), 1);
        Serial.println(" °C");
        
        // 添加WebSocket状态信息
        Serial.print("  WebSocket服务器: ");
        Serial.print(wsHost);
        Serial.print(":");
        Serial.println(wsPort);
        Serial.print("  WebSocket状态: ");
        Serial.println(isWsConnected ? "已连接" : "未连接");
        Serial.print("  WebSocket认证: ");
        Serial.println(isWsAuthenticated ? "已认证" : "未认证");
        Serial.print("  电池ID: ");
        Serial.println(BATTERY_ID);
        
        #if ENABLE_BMI270
        Serial.println("  BMI270: 已启用");
        #else
        Serial.println("  BMI270: 未启用");
        #endif
    }
    else {
        Serial.println("未知命令");
    }
}

// 修改BatteryMonitor类以使用INA226类
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
        float power;
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
        data.voltage = ina226.readBusVoltageFiltered();  // 使用带滤波的方法
        data.current = ina226.readCurrentFiltered();     // 使用带滤波的方法
        data.power = ina226.readPowerFiltered();        // 使用带滤波的方法
        
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

// 处理电源控制命令
void handlePowerCommand(bool powerState) {
    Serial.print("处理电源控制命令: ");
    Serial.println(powerState ? "开启" : "关闭");
    
    // 只更新电源状态标志位，不直接控制引脚
    enState = powerState;
    
    // 发送状态响应
    sendPowerStateResponse(true);
    
    // 打印调试信息
    Serial.print("EN状态: ");
    Serial.println(enState ? "开启" : "关闭");
}

// WebSocket事件处理函数
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.println("WebSocket断开连接");
            isWsConnected = false;
            isWsAuthenticated = false;
            break;
        case WStype_CONNECTED:
            {
                Serial.print("WebSocket已连接到服务器: ");
                Serial.println((char*)payload);
                isWsConnected = true;
                
                // 发送认证请求
                sendAuthRequest();
            }
            break;
        case WStype_TEXT:
            {
                Serial.print("收到WebSocket消息: ");
                Serial.println((char*)payload);
                
                // 解析JSON响应
                DynamicJsonDocument doc(1024);
                DeserializationError error = deserializeJson(doc, payload, length);
                
                if(error) {
                    Serial.print("JSON解析失败: ");
                    Serial.println(error.c_str());
                    return;
                }
                
                // 检查是否是认证响应
                if(doc.containsKey("type") && doc["type"] == "auth_response") {
                    if(doc["success"]) {
                        Serial.println("WebSocket认证成功!");
                        isWsAuthenticated = true;
                    } else {
                        Serial.print("WebSocket认证失败: ");
                        Serial.println(doc["message"].as<String>());
                        isWsAuthenticated = false;
                    }
                }
                
                // 增强的电源命令处理
                else if(doc.containsKey("type") && doc["type"] == "power_command") {
                    // 首先检查是否有电源状态字段
                    if(doc.containsKey("power_state")) {
                        bool powerState = doc["power_state"].as<bool>();
                        
                        // 检查是否有终端ID字段
                        if(doc.containsKey("terminal_id")) {
                            String targetId = doc["terminal_id"].as<String>();
                            Serial.print("目标设备ID: ");
                            Serial.println(targetId);
                            Serial.print("当前设备ID: ");
                            Serial.println(BATTERY_ID);
                            
                            // 如果ID匹配才处理
                            if(targetId.equals(BATTERY_ID)) {
                                handlePowerCommand(powerState);
                            } else {
                                Serial.println("终端ID不匹配，忽略命令");
                            }
                        } else {
                            // 没有终端ID，直接处理命令
                            Serial.println("收到无终端ID的电源命令，默认处理");
                            handlePowerCommand(powerState);
                        }
                    } else {
                        Serial.println("电源命令缺少power_state字段");
                    }
                }
                
                // 检查是否是ping
                else if(doc.containsKey("type") && doc["type"] == "ping") {
                    // 响应ping
                    DynamicJsonDocument pingResp(256);
                    pingResp["type"] = "pong";
                    pingResp["timestamp"] = getISOTimestamp();
                    pingResp["device_id"] = BATTERY_ID;
                    
                    String pongStr;
                    serializeJson(pingResp, pongStr);
                    webSocket.sendTXT(pongStr);
                    Serial.println("收到ping，已回复pong");
                }
            }
            break;
        case WStype_BIN:
            Serial.print("收到二进制数据，长度: ");
            Serial.println(length);
            break;
        case WStype_ERROR:
            Serial.println("WebSocket错误");
            break;
        default:
            break;
    }
}

// 发送认证请求
void sendAuthRequest() {
    if(!isWsConnected) {
        Serial.println("未连接到WebSocket服务器，无法发送认证请求");
        return;
    }
    
    Serial.println("发送WebSocket认证请求...");
    
    // 创建JSON对象
    DynamicJsonDocument doc(1024);
    doc["type"] = "auth";
    doc["apiKey"] = API_KEY;
    doc["isDevice"] = true;
    doc["deviceId"] = BATTERY_ID;
    
    // 添加设备信息
    JsonObject deviceInfo = doc.createNestedObject("deviceInfo");
    deviceInfo["deviceType"] = "battery";
    deviceInfo["version"] = "1.0.0";
    deviceInfo["manufacturer"] = "SmartBattery";
    
    // 获取当前运行时间(毫秒)
    unsigned long uptime = millis();
    deviceInfo["uptime"] = uptime;
    
    // 序列化JSON
    String jsonStr;
    serializeJson(doc, jsonStr);
    
    // 发送消息
    webSocket.sendTXT(jsonStr);
    Serial.println("已发送认证请求: " + jsonStr);
}

// 修正时间戳格式
String getISOTimestamp() {
    time_t now;
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
        return "2023-01-01T00:00:00Z";
    }
    char isoTime[25];
    strftime(isoTime, sizeof(isoTime), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    return String(isoTime);
}

// 发送电池数据
void sendBatteryData() {
    if(!isWsConnected || !isWsAuthenticated) {
        return; // 未连接或未认证，不发送数据
    }
    
    // 获取电池数据
    BatteryMonitor::SensorData data = batteryMonitor.getData();
    
    // 创建JSON对象
    DynamicJsonDocument doc(1024);
    
    // 修改后的JSON结构（数值类型保持为浮点数）
    doc["device_type"] = "battery";
    doc["terminal_id"] = BATTERY_ID;
    doc["device_ip"] = WiFi.localIP().toString();
    doc["timestamp"] = getISOTimestamp();
    doc["interval"] = WS_SEND_INTERVAL;

    // 电池参数（全部使用数值类型）
    doc["cells"] = data.cells;
    doc["voltage"] = data.voltage;  // 直接使用浮点数
    // 计算电量百分比（与模拟器一致）
    float cellVoltage = data.voltage / data.cells;
    int batteryPercent = 0;
    if(cellVoltage >= 3.0 && cellVoltage <= 4.2) {
        batteryPercent = round((cellVoltage - 3.0) / 1.2 * 100);
    }
    doc["battery_percent"] = batteryPercent;
    doc["power"] = data.power;      // 直接使用浮点数
    doc["temperature"] = data.temperature;  // 直接使用浮点数
    doc["is_charging"] = data.current > 0;
    doc["power_state"] = enState;  // 使用保存的状态变量，而不是直接读取引脚

    // 删除多余的字段，确保与模拟器格式一致
    doc["rssi"] = WiFi.RSSI();       // 保留信号强度作为额外信息
    
    // 序列化JSON
    String jsonStr;
    serializeJson(doc, jsonStr);
    
    // 发送数据
    webSocket.sendTXT(jsonStr);
    
    // 优化打印输出
    unsigned long currentTime = millis();
    if (firstDataSent) {
        Serial.print("已发送电池数据 ");
        firstDataSent = false;
    }
    
    // 更新打印计数器和缓冲区
    printCounter = (printCounter + 1) % 4;
    switch (printCounter) {
        case 0: strcpy(printBuffer, "\\"); break;
        case 1: strcpy(printBuffer, "|"); break;
        case 2: strcpy(printBuffer, "/"); break;
        case 3: strcpy(printBuffer, "-"); break;
    }
    Serial.print(printBuffer);
    Serial.print("\b"); // 回退一格
    
    // 每隔指定时间完整打印一次数据
    if (currentTime - lastDataPrintTime >= DATA_PRINT_INTERVAL) {
        Serial.println(); // 换行
        Serial.println("电池数据详情: ");
        Serial.print("  电压: "); Serial.print(data.voltage); Serial.println("V");
        Serial.print("  电流: "); Serial.print(data.current); Serial.println("A");
        Serial.print("  功率: "); Serial.print(data.power); Serial.println("W");
        Serial.print("  温度: "); Serial.print(data.temperature); Serial.println("°C");
        Serial.print("  电源状态: "); Serial.println(enState ? "开启" : "关闭");
        Serial.println("已发送电池数据");
        
        // 重置首次发送标志，这样下一个10秒周期会重新开始动画
        firstDataSent = true;
        lastDataPrintTime = currentTime;
    }
}

// 增强电源状态响应
void sendPowerStateResponse(bool powerState) {
    if(!isWsConnected || !isWsAuthenticated) {
        Serial.println("未连接或未认证，无法发送电源状态响应");
        return;
    }
    
    // 创建JSON对象
    DynamicJsonDocument doc(1024);
    
    // 简化响应结构，与模拟器保持一致
    doc["type"] = "power_state_response";
    doc["terminal_id"] = BATTERY_ID;
    doc["power_state"] = powerState;
    doc["success"] = true;
    doc["timestamp"] = getISOTimestamp();
    
    // 添加额外的有用信息，但格式更简洁
    doc["message"] = powerState ? "Power on success" : "Power off success";
    doc["actual_state"] = enState;  // 使用状态变量而不是读取引脚
    
    // 序列化JSON
    String jsonStr;
    serializeJson(doc, jsonStr);
    
    // 发送数据
    webSocket.sendTXT(jsonStr);
    Serial.print("已发送电源状态响应: ");
    Serial.println(powerState ? "开启" : "关闭");
    
    // 打印调试信息
    Serial.print("发送电源响应：");
    serializeJson(doc, Serial);
    Serial.println();
}

// 发送ping消息
void sendPing() {
    if(!isWsConnected || !isWsAuthenticated) {
        return; // 未连接或未认证，不发送ping
    }
    
    // 创建JSON对象
    DynamicJsonDocument doc(256);
    doc["type"] = "ping";
    doc["device_id"] = BATTERY_ID;
    doc["timestamp"] = getISOTimestamp();
    
    // 序列化JSON
    String jsonStr;
    serializeJson(doc, jsonStr);
    
    // 发送ping
    webSocket.sendTXT(jsonStr);
    
    // 简化打印
    Serial.print("PING: ");
    Serial.println(doc["timestamp"].as<String>());
}

// WebSocket连接任务
void wsClientTask(void *parameter) {
    // 初始化WebSocket
    webSocket.begin(wsHost, wsPort, wsPath);
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000); // 5秒重连一次
    
    while (1) {
        // 保持WebSocket连接
        webSocket.loop();
        
        unsigned long currentTime = millis();
        
        // 定时发送电池数据
        if(isWsConnected && isWsAuthenticated && 
           (currentTime - lastWsSendTime >= WS_SEND_INTERVAL)) {
            // 如果间隔超过数据打印间隔，重置首次发送标志
            if(currentTime - lastDataPrintTime >= DATA_PRINT_INTERVAL) {
                firstDataSent = true;
            }
            
            sendBatteryData();
            lastWsSendTime = currentTime;
        }
        
        // 定时发送ping保持连接活跃
        if(isWsConnected && isWsAuthenticated && 
           (currentTime - lastPingTime >= PING_INTERVAL)) {
            sendPing();
            lastPingTime = currentTime;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 短暂延时避免占用过多CPU
    }
}

// 用户控制任务
void userControlTask(void *parameter) {
    String inputBuffer = "";
    
    // 初始打印命令列表
    Serial.println("用户控制任务启动");
    printCommandList();
    lastCommandPrintTime = millis();
    
    while (1) {
        unsigned long currentTime = millis();
        
        // 每隔指定时间打印一次命令列表
        if (currentTime - lastCommandPrintTime >= COMMAND_PRINT_INTERVAL) {
            printCommandList();
            lastCommandPrintTime = currentTime;
        }
        
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

// 打印命令列表的辅助函数
void printCommandList() {
    Serial.println("\n可用命令:");
    Serial.println("  en:on - 打开EN引脚");
    Serial.println("  en:off - 关闭EN引脚");
    Serial.println("  id:xxx - 设置设备ID为xxx");
    Serial.println("  wifi:ssid,password - 设置WiFi");
    Serial.println("  ws:host:port - 设置WebSocket服务器");
    Serial.println("  wskey:apikey - 设置WebSocket API密钥");
    Serial.println("  battery_id:xxx - 设置电池ID为xxx");
    Serial.println("  status - 显示当前状态");
}

// 修改数据更新任务
void updateTask(void *parameter) {
    while (1) {
        // 控制EN引脚
        digitalWrite(EN_GPIO_NUM, enState ? HIGH : LOW);
        
        BatteryMonitor::SensorData data = batteryMonitor.getData();
        
        display.updateData(
            data.voltage,
            data.cells,
            data.current,
            data.power,
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

// 修改BMI270任务
#if ENABLE_BMI270
void bmi270Task(void *parameter) {
    // 检查BMI270是否已经初始化成功
    if (!bmi270.isInitialized()) {
        Serial.println("BMI270未初始化，任务终止");
        vTaskDelete(NULL);
        return;
    }
    
    while (1) {
        // 读取传感器数据
        SensorData data = bmi270.readSensorData();
        
        if (data.valid) {
            // 更新卡尔曼滤波器
            kalman.update(data);
            
            // 获取估计的角度
            float pitch = kalman.getPitch();
            float roll = kalman.getRoll();
            float yaw = kalman.getYaw();
            
            // 更新显示
            display.updateAttitude(pitch, roll, yaw);
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz更新率
    }
}
#endif

void setup() {
    Serial.begin(115200);
    
    // 统一初始化I2C
    if (!Wire.begin()) {
        Serial.println("I2C初始化失败!");
        while (1);
    }
    Serial.println("I2C初始化成功");

    configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    
    #if ENABLE_BMI270
    // 设置BMI270跳过I2C初始化
    bmi270.skipI2CInit(true);
    
    // 初始化BMI270
    if (!bmi270.begin()) {
        Serial.println("BMI270初始化失败！请检查连接和地址。");
    } else {
        Serial.println("BMI270初始化成功");
        // 初始化卡尔曼滤波器
        kalman.begin();
    }
    #endif
    
    // 初始化EN引脚
    pinMode(EN_GPIO_NUM, OUTPUT);
    digitalWrite(EN_GPIO_NUM, enState ? HIGH : LOW);

    // 初始化INA226
    if (ina226.begin(6.0))
    {
        Serial.println("INA226初始化成功");
    }
    else
    {
        Serial.println("INA226初始化失败");
    }

    // 设置偏移补偿
    ina226.setCurrentOffset(-0.012);
    ina226.setPowerOffset(0);

    // 设置滤波参数（可选）
    ina226.setFilterAlpha(0.8);

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
        0
    );

    // 创建WiFi任务 - 在核心0上运行
    xTaskCreatePinnedToCore(
        wifiTask,
        "WiFiTask",
        8192,
        NULL,
        1,
        NULL,
        0
    );

    // 创建显示任务 - 在核心0上运行
    xTaskCreatePinnedToCore(
        displayTask,
        "DisplayTask",
        4096,
        NULL,
        2,
        NULL,
        0
    );

    // 创建数据更新任务 - 在核心0上运行
    xTaskCreatePinnedToCore(
        updateTask,
        "UpdateTask",
        4096,
        NULL,
        1,
        NULL,
        0
    );

    // 创建BMI270数据读取任务 - 在核心0上运行
    #if ENABLE_BMI270
    xTaskCreatePinnedToCore(
        bmi270Task,
        "BMI270Task",
        4096,
        NULL,
        1,
        NULL,
        0
    );
    #endif

    // 创建WebSocket连接任务 - 在核心0上运行
    xTaskCreatePinnedToCore(
        wsClientTask,
        "WebSocketTask",
        4096,
        NULL,
        1,
        NULL,
        0
    );
}

void loop() {
  // 在FreeRTOS中，loop()可以为空
  vTaskDelete(NULL);  // 删除setup/loop任务
}