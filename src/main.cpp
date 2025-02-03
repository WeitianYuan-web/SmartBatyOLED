#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>

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

public:
  OLEDDisplay() : u8g2(U8G2_R0), 
                  batteryVoltage(0.0),
                  batteryCells(0),
                  current(0.0),
                  deviceID(""),
                  deviceIP("0.0.0.0"),
                  wifiConnected(false) {
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
  void updateData(float voltage, int cells, float amp, const String& id, const String& ip, bool wifiStatus) {
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      batteryVoltage = voltage;
      batteryCells = cells;
      current = amp;
      deviceID = id;
      deviceIP = ip;
      wifiConnected = wifiStatus;
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

// 模拟数据生成类
class SimulatedData {
private:
    float time = 0.0;
    const float PI2 = 2.0 * PI;
    String currentIP = "0.0.0.0";
    bool isWiFiConnected = false;
    
    // 在指定范围内生成正弦波形数据
    float generateSineWave(float min, float max, float frequency, float phase = 0.0) {
        float middle = (max + min) / 2.0;
        float amplitude = (max - min) / 2.0;
        return middle + amplitude * sin(PI2 * frequency * time + phase);
    }

public:
    struct SensorData {
        float voltage;
        float current;
        int cells;
        String deviceID;
        String deviceIP;
        bool wifiConnected;
    };

    // 更新WiFi状态和IP
    void updateWiFiStatus(bool connected, const String& ip) {
        isWiFiConnected = connected;
        if (connected) {
            currentIP = ip;
        }
    }

    // 获取模拟数据
    SensorData getData() {
        SensorData data;
        
        // 先确定电池节数（1-4S循环变化）
        int cells = ((int)(time * 0.1) % 4) + 1;
        
        // 根据电池节数设置电压范围
        float nominalVoltage, maxVoltage;
        switch(cells) {
            case 1:
                nominalVoltage = 3.7f; maxVoltage = 4.2f;
                break;
            case 2:
                nominalVoltage = 7.4f; maxVoltage = 8.4f;
                break;
            case 3:
                nominalVoltage = 11.1f; maxVoltage = 12.6f;
                break;
            case 4:
                nominalVoltage = 14.8f; maxVoltage = 16.8f;
                break;
        }
        
        // 生成对应范围的电压值（在标称电压附近波动）
        data.voltage = generateSineWave(
            nominalVoltage - 0.5f,  // 最低电压比标称电压低0.5V
            maxVoltage,             // 最高电压为充满电压
            0.02
        );
        data.cells = cells;
        
        // 电流保持不变
        data.current = generateSineWave(0.0, 6.0, 0.05, PI/4);
        
        data.deviceID = "ID:A001";
        data.deviceIP = currentIP;
        data.wifiConnected = isWiFiConnected;

        time += 0.1;
        return data;
    }
};

// 创建全局实例
SimulatedData simulator;

// 修改数据更新任务
void updateTask(void *parameter) {
    while (1) {
        // 获取模拟数据
        SimulatedData::SensorData data = simulator.getData();
        
        // 更新显示
        display.updateData(
            data.voltage,
            data.cells,
            data.current,
            data.deviceID,
            data.deviceIP,
            data.wifiConnected
        );
        
        vTaskDelay(pdMS_TO_TICKS(100));  // 每100ms更新一次数据
    }
}

// WiFi配置
const char* ssid = "CMCC-JM3A";
const char* password = "18771407258";

// WiFi事件处理
void WiFiEvent(WiFiEvent_t event) {
    switch(event) {
        case SYSTEM_EVENT_STA_GOT_IP:
            simulator.updateWiFiStatus(true, WiFi.localIP().toString());
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            simulator.updateWiFiStatus(false, "Disconnected");
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
            simulator.updateWiFiStatus(false, "Connecting...");
        } else {
            // 确保IP地址始终是最新的
            simulator.updateWiFiStatus(true, WiFi.localIP().toString());
        }
        vTaskDelay(pdMS_TO_TICKS(5000));  // 每5秒检查一次连接状态
    }
}

void setup() {
    if (!display.begin()) {
        while (1);  // 如果OLED初始化失败，停止运行
    }

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