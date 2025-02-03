#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

// 创建U8G2显示对象，使用I2C通信
// 如果显示不正常，可以尝试将 "U8G2_R0" 改为 "U8G2_R2" 来旋转显示
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0);

// 模拟数据，实际使用时替换为真实数据
float batteryVoltage = 14.8;
int batteryCells = 4;  // 1-4S
float current = 2.5;
const char* deviceID = "ID:A001";
const char* deviceIP = "192.168.1.100";

// 绘制电池图标
void drawBattery(uint8_t x, uint8_t y, uint8_t percentage) {
  // 电池外框
  u8g2.drawFrame(x, y, 12, 6);
  // 电池正极
  u8g2.drawBox(x + 12, y + 2, 2, 2);
  // 电量填充
  uint8_t width = (percentage * 10) / 100;
  if (width > 0) {
    u8g2.drawBox(x + 1, y + 1, width, 4);
  }
}

// 绘制闪电图标（电流符号）
void drawLightning(uint8_t x, uint8_t y) {
  u8g2.drawLine(x, y, x + 3, y + 3);     // 上部
  u8g2.drawLine(x + 3, y + 3, x, y + 6); // 下部
  u8g2.drawPixel(x + 1, y + 3);          // 加粗中间部分
}

// 绘制WiFi图标
void drawWiFi(uint8_t x, uint8_t y) {
  u8g2.drawCircle(x, y, 6, U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawCircle(x, y, 4, U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawCircle(x, y, 2, U8G2_DRAW_UPPER_RIGHT);
}

void setup() {
  // 初始化OLED
  if (!u8g2.begin()) {
    Serial.begin(115200);
    Serial.println("OLED显示器初始化失败!");
    while (1);  // 如果初始化失败则停止运行
  }
  u8g2.enableUTF8Print();  // 启用UTF8支持，可以显示中文
  
  // 设置字体
  u8g2.setFont(u8g2_font_6x10_tf);  // 使用小号字体以显示更多信息
}

void loop() {
  // 清空显示缓冲区
  u8g2.clearBuffer();
  
  // 绘制边框
  u8g2.drawFrame(0, 0, 128, 32);
  u8g2.drawHLine(0, 19, 128);
  u8g2.drawVLine(64, 0, 19);  // 中间分隔线

  // 左侧电压显示
  drawBattery(2, 2, 80);  // 电池图标
  u8g2.setFont(u8g2_font_logisoso16_tr);
  u8g2.setCursor(16, 16);
  u8g2.print(batteryVoltage, 1);
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setCursor(54, 16);
  u8g2.print("V");

  // 右侧电流显示
  drawLightning(68, 6);    // 闪电图标
  u8g2.setFont(u8g2_font_logisoso16_tr);
  u8g2.setCursor(76, 16);
  u8g2.print(current, 1);
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setCursor(114, 16);
  u8g2.print("A");

  // 底部信息
  u8g2.setFont(u8g2_font_5x8_tf);
  drawWiFi(4, 28);        // WiFi图标
  u8g2.setCursor(12, 30);
  u8g2.print(deviceID);
  u8g2.print(" ");
  u8g2.print(deviceIP);

  // 将缓冲区内容发送到显示器
  u8g2.sendBuffer();
  delay(500);  // 每500ms更新一次显示
}