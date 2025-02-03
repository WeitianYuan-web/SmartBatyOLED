#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>

// OLED显示器类
class OLEDDisplay {
private:
  U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2;
  float batteryVoltage;
  int batteryCells;
  float current;
  String deviceID;
  String deviceIP;
  
  // 互斥锁，用于保护显示数据
  SemaphoreHandle_t displayMutex;
  
  // 绘制电池图标
  void drawBattery(uint8_t x, uint8_t y, uint8_t percentage) {
    u8g2.drawFrame(x, y, 12, 6);
    u8g2.drawBox(x + 12, y + 2, 2, 2);
    uint8_t width = (percentage * 10) / 100;
    if (width > 0) {
      u8g2.drawBox(x + 1, y + 1, width, 4);
    }
  }

  // 绘制闪电图标
  void drawLightning(uint8_t x, uint8_t y) {
    u8g2.drawLine(x, y, x + 3, y + 3);
    u8g2.drawLine(x + 3, y + 3, x, y + 6);
    u8g2.drawPixel(x + 1, y + 3);
  }

  // 绘制WiFi图标
  void drawWiFi(uint8_t x, uint8_t y) {
    u8g2.drawCircle(x, y, 6, U8G2_DRAW_UPPER_RIGHT);
    u8g2.drawCircle(x, y, 4, U8G2_DRAW_UPPER_RIGHT);
    u8g2.drawCircle(x, y, 2, U8G2_DRAW_UPPER_RIGHT);
  }

public:
  OLEDDisplay() : u8g2(U8G2_R0), 
                  batteryVoltage(0.0),
                  batteryCells(0),
                  current(0.0),
                  deviceID(""),
                  deviceIP("") {
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
  void updateData(float voltage, int cells, float amp, const String& id, const String& ip) {
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      batteryVoltage = voltage;
      batteryCells = cells;
      current = amp;
      deviceID = id;
      deviceIP = ip;
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
      u8g2.drawVLine(64, 0, 19);

      // 左侧电压显示
      drawBattery(2, 2, 80);
      u8g2.setFont(u8g2_font_logisoso16_tr);
      u8g2.setCursor(16, 16);
      u8g2.print(batteryVoltage, 1);
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(54, 16);
      u8g2.print("V");

      // 右侧电流显示
      drawLightning(68, 6);
      u8g2.setFont(u8g2_font_logisoso16_tr);
      u8g2.setCursor(76, 16);
      u8g2.print(current, 1);
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(114, 16);
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
    float time = 0.1;
    const float PI2 = 2.0 * PI;
    
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
    };

    // 获取模拟数据
    SensorData getData() {
        SensorData data;
        
        // 电压波动：基准值 + 正弦波动
        // 3-18V 范围内缓慢变化
        data.voltage = generateSineWave(3.0, 18.0, 0.02);
        
        // 电流波动：基准值 + 较快的正弦波动
        // 0-6A 范围内快速变化
        data.current = generateSineWave(0.0, 6.0, 0.05, PI/4);
        
        // 根据电压确定电池节数
        if (data.voltage < 4.5) data.cells = 1;        // 1S
        else if (data.voltage < 9.0) data.cells = 2;   // 2S
        else if (data.voltage < 13.5) data.cells = 3;  // 3S
        else data.cells = 4;                           // 4S

        // 固定值
        data.deviceID = "ID:A001";
        data.deviceIP = "192.168.1.100";

        // 更新时间
        time += 0.1;  // 时间步进
        
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
            data.deviceIP
        );
        
        vTaskDelay(pdMS_TO_TICKS(100));  // 每100ms更新一次数据
    }
}

void setup() {
  Serial.begin(115200);
  
  if (!display.begin()) {
    Serial.println("OLED显示器初始化失败!");
    while (1);
  }

  // 创建显示任务
  xTaskCreate(
    displayTask,          // 任务函数
    "DisplayTask",        // 任务名称
    4096,                // 堆栈大小
    NULL,                // 任务参数
    2,                   // 优先级
    NULL                 // 任务句柄
  );

  // 创建数据更新任务
  xTaskCreate(
    updateTask,          // 任务函数
    "UpdateTask",        // 任务名称
    4096,               // 堆栈大小
    NULL,               // 任务参数
    1,                  // 优先级
    NULL                // 任务句柄
  );
}

void loop() {
  // 在FreeRTOS中，loop()可以为空
  vTaskDelete(NULL);  // 删除setup/loop任务
}