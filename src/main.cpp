#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WebSocketsClient.h> // 添加WebSocket客户端库
#include <ArduinoJson.h>      // 添加JSON库
#include "INA226.h"  // 添加INA226头文件
#include <Preferences.h>  // 添加Preferences库用于保存配置

// 添加宏定义控制BMI270功能
#define ENABLE_BMI270 0  // 设置为1启用BMI270功能，设置为0禁用

#if ENABLE_BMI270
#include "BMI270Read.h"
#include "KalmanFilter.h"
#endif

#define EN_GPIO_NUM 10 // 旧10 新3

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
String deviceID = "B01";  // 默认设备ID
String batteryID = "smartbaty-001";  // 默认电池ID
String wsHostStr = "192.168.31.128";  // WebSocket服务器地址，转为字符串类型
uint16_t wsPort = 8080;              // WebSocket服务器端口
String wsPathStr = "/";              // WebSocket路径
String apiKeyStr = "sensor_device_key_123"; // 认证密钥
String ssidStr = "Xiaomi_B1F1";  // 默认WiFi SSID
String passwordStr = "Ywt6837057";  // 默认WiFi密码
bool  enState = true;  // EN引脚状态

// 创建Preferences实例
Preferences preferences;

// WebSocket相关配置
// 这些变量只是字符串的临时指针，不需要全局存储
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

// 修改CONFIG_HELP，添加导入导出命令
const char* CONFIG_HELP = 
    "配置命令格式: config:<参数>=<值>\n"
    "可配置参数:\n"
    "  device_id - 设备ID\n"
    "  battery_id - 电池ID\n"
    "  ws_host - WebSocket服务器地址\n"
    "  ws_port - WebSocket服务器端口\n"
    "  api_key - WebSocket认证密钥\n"
    "  wifi_ssid - WiFi名称\n"
    "  wifi_pass - WiFi密码\n"
    "  en_state - 电源状态(on/off)\n"
    "  save - 保存所有配置\n"
    "  reset - 重置所有配置\n"
    "  export - 导出配置为JSON\n"
    "  import=<JSON字符串> - 导入配置\n"
    "  status - 显示当前设置和状态\n"
    "  help - 显示此帮助\n"
    "示例: config:battery_id=battery-001";

// 修改打印命令列表的辅助函数
void printCommandList() {
    Serial.println("\n可用命令:");
    Serial.println("  config:<参数>=<值> - 配置系统参数 (输入config:help查看详细帮助)");
    Serial.println("  config:status - 显示当前状态");
    Serial.println("  en:on - 打开电源");
    Serial.println("  en:off - 关闭电源");
}

// 在processCommand函数中添加导入导出功能
void processCommand(const String& command) {
    if (command.startsWith("config:")) {
        String configParam = command.substring(7);
        
        // 处理配置帮助
        if (configParam == "help") {
            Serial.println(CONFIG_HELP);
            return;
        }
        
        // 处理状态显示
        if (configParam == "status") {
            Serial.println("当前状态:");
            Serial.print("  设备ID: ");
            Serial.println(deviceID);
            Serial.print("  电池ID: ");
            Serial.println(batteryID);
            Serial.print("  WiFi名称: ");
            Serial.println(ssidStr);
            Serial.print("  WiFi密码: ");
            Serial.println("********");
            Serial.print("  WebSocket服务器: ");
            Serial.print(wsHostStr);
            Serial.print(":");
            Serial.println(wsPort);
            Serial.print("  API密钥: ");
            Serial.println(apiKeyStr);
            Serial.print("  电源状态: ");
            Serial.println(enState ? "开启" : "关闭");
            Serial.print("  WiFi状态: ");
            Serial.println(WiFi.status() == WL_CONNECTED ? "已连接" : "未连接");
            if (WiFi.status() == WL_CONNECTED) {
                Serial.print("  IP地址: ");
                Serial.println(WiFi.localIP().toString());
            }
            Serial.print("  电压: ");
            Serial.print(ina226.readBusVoltage(), 2);
            Serial.println(" V");
            Serial.print("  电流: ");
            Serial.print(ina226.readCurrent(), 3);
            Serial.println(" A");
            Serial.print("  功率: ");
            Serial.print(ina226.readPower(), 2);
            Serial.println(" W");
            Serial.print("  温度: ");
            Serial.print(temperatureRead(), 1);
            Serial.println(" °C");
            
            // 添加WebSocket状态信息
            Serial.print("  WebSocket状态: ");
            Serial.println(isWsConnected ? "已连接" : "未连接");
            Serial.print("  WebSocket认证: ");
            Serial.println(isWsAuthenticated ? "已认证" : "未认证");
            
            #if ENABLE_BMI270
            Serial.println("  BMI270: 已启用");
            #else
            Serial.println("  BMI270: 未启用");
            #endif
            return;
        }
        
        // 处理导出配置
        if (configParam == "export") {
            DynamicJsonDocument doc(1024);
            
            // 添加所有配置项
            doc["device_id"] = deviceID;
            doc["battery_id"] = batteryID;
            doc["ws_host"] = wsHostStr;
            doc["ws_port"] = wsPort;
            doc["api_key"] = apiKeyStr;
            doc["wifi_ssid"] = ssidStr;
            doc["wifi_pass"] = passwordStr;
            doc["en_state"] = enState;
            
            // 序列化JSON
            String jsonStr;
            serializeJson(doc, jsonStr);
            
            // 输出配置
            Serial.println("导出的配置:");
            Serial.println(jsonStr);
            return;
        }
        
        // 处理导入配置
        if (configParam.startsWith("import=")) {
            String jsonStr = configParam.substring(7);
            
            // 解析JSON
            DynamicJsonDocument doc(1024);
            DeserializationError error = deserializeJson(doc, jsonStr);
            
            if (error) {
                Serial.print("JSON解析失败: ");
                Serial.println(error.c_str());
                return;
            }
            
            // 导入配置项
            bool hasChanges = false;
            
            if (doc.containsKey("device_id")) {
                deviceID = doc["device_id"].as<String>();
                preferences.putString("deviceID", deviceID);
                hasChanges = true;
            }
            
            if (doc.containsKey("battery_id")) {
                batteryID = doc["battery_id"].as<String>();
                preferences.putString("batteryID", batteryID);
                hasChanges = true;
            }
            
            if (doc.containsKey("ws_host")) {
                wsHostStr = doc["ws_host"].as<String>();
                preferences.putString("wsHost", wsHostStr);
                hasChanges = true;
            }
            
            if (doc.containsKey("ws_port")) {
                wsPort = doc["ws_port"].as<uint16_t>();
                preferences.putUShort("wsPort", wsPort);
                hasChanges = true;
            }
            
            if (doc.containsKey("api_key")) {
                apiKeyStr = doc["api_key"].as<String>();
                preferences.putString("apiKey", apiKeyStr);
                hasChanges = true;
            }
            
            if (doc.containsKey("wifi_ssid")) {
                ssidStr = doc["wifi_ssid"].as<String>();
                preferences.putString("wifiSsid", ssidStr);
                hasChanges = true;
            }
            
            if (doc.containsKey("wifi_pass")) {
                passwordStr = doc["wifi_pass"].as<String>();
                preferences.putString("wifiPass", passwordStr);
                hasChanges = true;
            }
            
            if (doc.containsKey("en_state")) {
                enState = doc["en_state"].as<bool>();
                preferences.putBool("enState", enState);
                hasChanges = true;
            }
            
            if (hasChanges) {
                Serial.println("配置已导入并保存");
                
                // 如果WiFi设置更改，重新连接
                if (doc.containsKey("wifi_ssid") || doc.containsKey("wifi_pass")) {
                    Serial.println("WiFi设置已更新，重新连接中...");
                    WiFi.disconnect();
                    WiFi.begin(ssidStr.c_str(), passwordStr.c_str());
                }
                
                // 如果WebSocket设置更改，重新连接
                if (doc.containsKey("ws_host") || doc.containsKey("ws_port")) {
                    Serial.println("WebSocket设置已更新，重新连接中...");
                    webSocket.disconnect();
                    webSocket.begin(wsHostStr.c_str(), wsPort, wsPathStr.c_str());
                }
                
                // 如果认证信息更改，重新认证
                if (doc.containsKey("api_key") || doc.containsKey("battery_id")) {
                    isWsAuthenticated = false;
                    if (isWsConnected) {
                        sendAuthRequest();
                    }
                }
            } else {
                Serial.println("无有效配置项导入");
            }
            
            return;
        }
        
        // 查找等号分隔参数名和值
        int equalsPos = configParam.indexOf('=');
        if (equalsPos <= 0) {
            Serial.println("配置格式错误，使用 config:<参数>=<值>");
            Serial.println("输入 config:help 查看帮助");
            return;
        }
        
        String paramName = configParam.substring(0, equalsPos);
        String paramValue = configParam.substring(equalsPos + 1);
        
        paramName.trim();
        paramValue.trim();
        
        // 处理各种参数
        if (paramName == "device_id") {
            if (paramValue.length() > 0) {
                deviceID = paramValue;
                // 保存到preferences
                preferences.putString("deviceID", deviceID);
                Serial.print("设备ID已更新为: ");
                Serial.println(deviceID);
            } else {
                Serial.println("无效的设备ID");
            }
        }
        else if (paramName == "battery_id") {
            if (paramValue.length() > 0) {
                // 更新电池ID
                batteryID = paramValue;
                // 保存到preferences
                preferences.putString("batteryID", batteryID);
                
                Serial.print("电池ID已更新为: ");
                Serial.println(batteryID);
                
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
        else if (paramName == "ws_host") {
            if (paramValue.length() > 0) {
                wsHostStr = paramValue;
                // 保存到preferences
                preferences.putString("wsHost", wsHostStr);
                
                Serial.print("WebSocket服务器地址已更新为: ");
                Serial.println(wsHostStr);
                
                // 重新连接WebSocket
                webSocket.disconnect();
                webSocket.begin(wsHostStr.c_str(), wsPort, wsPathStr.c_str());
            } else {
                Serial.println("无效的服务器地址");
            }
        }
        else if (paramName == "ws_port") {
            int port = paramValue.toInt();
            if (port > 0 && port < 65536) {
                wsPort = port;
                // 保存到preferences
                preferences.putUShort("wsPort", port);
                
                Serial.print("WebSocket服务器端口已更新为: ");
                Serial.println(wsPort);
                
                // 重新连接WebSocket
                webSocket.disconnect();
                webSocket.begin(wsHostStr.c_str(), wsPort, wsPathStr.c_str());
            } else {
                Serial.println("无效的端口号");
            }
        }
        else if (paramName == "api_key") {
            if (paramValue.length() > 0) {
                apiKeyStr = paramValue;
                // 保存到preferences
                preferences.putString("apiKey", apiKeyStr);
                
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
        else if (paramName == "wifi_ssid") {
            if (paramValue.length() > 0) {
                ssidStr = paramValue;
                // 保存到preferences
                preferences.putString("wifiSsid", ssidStr);
                
                Serial.print("WiFi名称已更新为: ");
                Serial.println(ssidStr);
                Serial.println("注意：需要同时设置wifi_pass才会重新连接");
            } else {
                Serial.println("无效的WiFi名称");
            }
        }
        else if (paramName == "wifi_pass") {
            if (paramValue.length() > 0) {
                passwordStr = paramValue;
                // 保存到preferences
                preferences.putString("wifiPass", passwordStr);
                
                Serial.println("WiFi密码已更新");
                
                // 重新连接WiFi
                Serial.println("WiFi设置已更新，重新连接中...");
                WiFi.disconnect();
                WiFi.begin(ssidStr.c_str(), passwordStr.c_str());
            } else {
                Serial.println("无效的WiFi密码");
            }
        }
        else if (paramName == "en_state") {
            if (paramValue == "on" || paramValue == "true" || paramValue == "1") {
                enState = true;
                // 保存到preferences
                preferences.putBool("enState", true);
                Serial.println("电源状态已设为开启");
            } 
            else if (paramValue == "off" || paramValue == "false" || paramValue == "0") {
                enState = false;
                // 保存到preferences
                preferences.putBool("enState", false);
                Serial.println("电源状态已设为关闭");
            }
            else {
                Serial.println("无效的电源状态值，使用 on/off, true/false 或 1/0");
            }
        }
        else if (paramName == "save") {
            // 立即保存所有当前设置
            preferences.putString("deviceID", deviceID);
            preferences.putString("batteryID", batteryID);
            preferences.putString("wsHost", wsHostStr);
            preferences.putUShort("wsPort", wsPort);
            preferences.putString("apiKey", apiKeyStr);
            preferences.putString("wifiSsid", ssidStr);
            preferences.putString("wifiPass", passwordStr);
            preferences.putBool("enState", enState);
            Serial.println("所有配置已保存");
        }
        else if (paramName == "reset") {
            // 重置所有配置
            preferences.clear();
            Serial.println("所有配置已重置为默认值");
            Serial.println("请重启设备以应用默认配置");
        }
        else {
            Serial.print("未知的配置参数: ");
            Serial.println(paramName);
            Serial.println("输入 config:help 查看帮助");
        }
    }
    else if (command.startsWith("en:")) {
        String value = command.substring(3);
        value.trim();
        
        if (value == "on") {
            enState = true;
            // 保存到preferences
            preferences.putBool("enState", true);
            Serial.println("电源状态已设为开启");
        } else if (value == "off") {
            enState = false;
            // 保存到preferences
            preferences.putBool("enState", false);
            Serial.println("电源状态已设为关闭");
        } else {
            Serial.println("无效的电源命令，使用 on 或 off");
        }
    }
    else if (command == "status") {
        // 兼容旧的status命令，重定向到config:status
        processCommand("config:status");
    }
    else {
        Serial.println("未知命令，输入config:help查看帮助");
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
    
    // 修正当前检测，消除小噪声
    float correctCurrent(float rawCurrent) {
        // 忽略非常小的电流值，认为是噪声
        if (fabs(rawCurrent) < 0.05) {
            return 0.0;
        }
        return rawCurrent;
    }
    
    // 修正电压检测
    float correctVoltage(float rawVoltage) {
        // 如果电压低于某个阈值，可能是传感器噪声
        if (rawVoltage < 1.0) {
            return 0.0;
        }
        return rawVoltage;
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
        
        // 读取原始数据并进行校正
        float rawVoltage = ina226.readBusVoltageFiltered();
        float rawCurrent = ina226.readCurrentFiltered();
        float rawPower = ina226.readPowerFiltered();
        
        // 应用校正并填充数据
        data.voltage = correctVoltage(rawVoltage);
        data.current = correctCurrent(rawCurrent);
        data.power = rawPower;  // 使用读取的功率值，也可以根据校正后的电压电流重新计算
        
        // 每10秒打印一次详细的电流电压情况
        static unsigned long lastDebugTime = 0;
        unsigned long now = millis();
        if (now - lastDebugTime > 10000) {
            Serial.println("INA226详细读数:");
            Serial.print("  原始电压: "); 
            Serial.print(rawVoltage, 3); 
            Serial.print(" V, 校正后: "); 
            Serial.print(data.voltage, 3);
            Serial.println(" V");
            
            Serial.print("  原始电流: "); 
            Serial.print(rawCurrent, 3); 
            Serial.print(" A, 校正后: "); 
            Serial.print(data.current, 3);
            Serial.println(" A");
            
            Serial.print("  原始功率: "); 
            Serial.print(rawPower, 3); 
            Serial.println(" W");
            
            lastDebugTime = now;
        }
        
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
                            Serial.println(batteryID);
                            
                            // 如果ID匹配才处理
                            if(targetId.equals(batteryID)) {
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
                    pingResp["device_id"] = batteryID;
                    
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
    
    // 获取最新的指针引用
    const char* batteryIdPtr = batteryID.c_str();
    const char* apiKeyPtr = apiKeyStr.c_str();
    
    // 创建JSON对象
    DynamicJsonDocument doc(1024);
    doc["type"] = "auth";
    doc["apiKey"] = apiKeyPtr;
    doc["isDevice"] = true;
    doc["deviceId"] = batteryIdPtr;
    
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
    
    // 获取最新的指针引用
    const char* batteryIdPtr = batteryID.c_str();
    
    // 创建JSON对象
    DynamicJsonDocument doc(1024);
    
    // 修改后的JSON结构（数值类型保持为浮点数）
    doc["device_type"] = "battery";
    doc["terminal_id"] = batteryIdPtr;
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
    
    // 获取最新的指针引用
    const char* batteryIdPtr = batteryID.c_str();
    
    // 创建JSON对象
    DynamicJsonDocument doc(1024);
    
    // 简化响应结构，与模拟器保持一致
    doc["type"] = "power_state_response";
    doc["terminal_id"] = batteryIdPtr;
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
    
    // 获取最新的指针引用
    const char* batteryIdPtr = batteryID.c_str();
    
    // 创建JSON对象
    DynamicJsonDocument doc(256);
    doc["type"] = "ping";
    doc["device_id"] = batteryIdPtr;
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
    const char* wsHostPtr = wsHostStr.c_str();
    const char* wsPathPtr = wsPathStr.c_str();
    webSocket.begin(wsHostPtr, wsPort, wsPathPtr);
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000); // 5秒重连一次
    
    while (1) {
        // 更新字符串指针，确保访问最新内容
        const char* batteryIdPtr = batteryID.c_str();
        const char* apiKeyPtr = apiKeyStr.c_str();
        
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
    WiFi.begin(ssidStr.c_str(), passwordStr.c_str());
    
    while (1) {
        if (WiFi.status() != WL_CONNECTED) {
            // 如果断开连接，尝试重连
            WiFi.reconnect();
            batteryMonitor.updateWiFiStatus(false, "连接中...");
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
    
    // 打开preferences命名空间
    preferences.begin("smartbaty", false);
    
    // 读取保存的配置项 (如果不存在则使用默认值)
    deviceID = preferences.getString("deviceID", deviceID);
    String savedBatteryID = preferences.getString("batteryID", batteryID);
    String savedWsHost = preferences.getString("wsHost", wsHostStr);
    uint16_t savedWsPort = preferences.getUShort("wsPort", wsPort);
    String savedApiKey = preferences.getString("apiKey", apiKeyStr);
    String savedSsid = preferences.getString("wifiSsid", ssidStr);
    String savedPassword = preferences.getString("wifiPass", passwordStr);
    enState = preferences.getBool("enState", enState);
    
    // 更新全局变量
    if (savedBatteryID.length() > 0) batteryID = savedBatteryID;
    if (savedWsHost.length() > 0) wsHostStr = savedWsHost;
    if (savedWsPort > 0) wsPort = savedWsPort;
    if (savedApiKey.length() > 0) apiKeyStr = savedApiKey;
    if (savedSsid.length() > 0) ssidStr = savedSsid;
    if (savedPassword.length() > 0) passwordStr = savedPassword;
    
    Serial.println("已加载保存的配置");
    Serial.print("设备ID: ");
    Serial.println(deviceID);
    Serial.print("电池ID: ");
    Serial.println(batteryID);
    
    // 统一初始化I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000); // 降低I2C时钟速率到100kHz，增加通信稳定性
    
    Serial.println("I2C初始化成功");
    Serial.print("使用引脚 - SDA: ");
    Serial.print(SDA_PIN);
    Serial.print(", SCL: ");
    Serial.println(SCL_PIN);

    // 扫描I2C总线，查找连接的设备
    Serial.println("扫描I2C总线...");
    uint8_t deviceCount = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.print("发现I2C设备，地址: 0x");
            if (addr < 16) Serial.print("0");
            Serial.println(addr, HEX);
            
            if (addr == INA226_ADDR) {
                Serial.println("  -> 这是INA226的地址");
            }
            deviceCount++;
        }
    }
    
    if (deviceCount == 0) {
        Serial.println("没有发现I2C设备，请检查连接!");
    } else {
        Serial.print("共发现 ");
        Serial.print(deviceCount);
        Serial.println(" 个I2C设备");
    }

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
    Serial.println("开始初始化INA226...");
    
    // 增加延时让I2C稳定
    delay(500);  // 增加到500ms
    
    // 先检查INA226是否可访问
    Wire.beginTransmission(INA226_ADDR);
    byte errorCode = Wire.endTransmission();
    
    if (errorCode != 0) {
        Serial.print("无法访问INA226，错误码: ");
        Serial.println(errorCode);
        Serial.println("错误说明:");
        switch(errorCode) {
            case 1: Serial.println("  数据过长"); break;
            case 2: Serial.println("  传输中收到NACK"); break;
            case 3: Serial.println("  接收数据时收到NACK"); break;
            case 4: Serial.println("  其他错误"); break;
            default: Serial.println("  未知错误"); break;
        }
        Serial.println("尝试重新初始化...");
        Wire.begin(SDA_PIN, SCL_PIN);
        Wire.setClock(50000);  // 降低至50kHz
        delay(1000);  // 再延时1秒
    }
    
    // 尝试三次初始化INA226
    bool ina226Initialized = false;
    for (int attempt = 0; attempt < 3 && !ina226Initialized; attempt++) {
        Serial.print("INA226初始化尝试 #");
        Serial.println(attempt + 1);
        
        if (ina226.begin(10.0)) {  // 最大电流设为10A，提高测量精度
            ina226Initialized = true;
            Serial.println("INA226初始化成功");
            
            // 设置偏移补偿，需根据实际测量调整
            ina226.setCurrentOffset(0.0);  // 先设为0进行测试
            ina226.setPowerOffset(0.0);
            
            // 设置滤波系数
            ina226.setFilterAlpha(0.3);  // 平衡响应速度和稳定性
            
            // 打印测试读数
            Serial.println("INA226初始读数:");
            Serial.print("  总线电压: ");
            Serial.print(ina226.readBusVoltage(), 3);
            Serial.println(" V");
            Serial.print("  分流电压: ");
            Serial.print(ina226.readShuntVoltage() * 1000, 3);
            Serial.println(" mV");
            Serial.print("  电流: ");
            Serial.print(ina226.readCurrent(), 3);
            Serial.println(" A");
            Serial.print("  功率: ");
            Serial.print(ina226.readPower(), 3);
            Serial.println(" W");
        } else {
            Serial.println("INA226初始化失败");
            Serial.println("等待500ms后重试...");
            Wire.begin(SDA_PIN, SCL_PIN);  // 重新初始化I2C
            Wire.setClock(50000 - (attempt * 10000));  // 每次尝试降低速度
            delay(500);
        }
    }
    
    if (!ina226Initialized) {
        Serial.println("INA226初始化最终失败 - 检查接线和I2C地址");
        Serial.println("程序继续执行，但电流和电压测量可能不准确");
        Serial.print("I2C地址: 0x");
        Serial.println(INA226_ADDR, HEX);
        Serial.print("SDA引脚: ");
        Serial.println(SDA_PIN);
        Serial.print("SCL引脚: ");
        Serial.println(SCL_PIN);
        Serial.println("建议检查:");
        Serial.println("1. 检查INA226电源是否接通");
        Serial.println("2. 确认SDA/SCL引脚连接正确");
        Serial.println("3. 检查是否有上拉电阻(建议4.7kΩ)");
        Serial.println("4. 尝试减少I2C总线长度或干扰");
    }

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