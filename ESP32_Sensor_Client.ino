/*
 * ESP32 WebSocket传感器与电池客户端
 * 此代码用于将ESP32连接到WebSocket服务器并发送传感器和电池数据
 * 支持分别发送陀螺仪和电池数据，可通过串口命令配置
 */

#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>

// 网络设置
const char* ssid = "您的WiFi名称";     // 替换为您的WiFi名称
const char* password = "您的WiFi密码";  // 替换为您的WiFi密码

// WebSocket服务器设置
const char* websocket_server_host = "192.168.1.100"; // 替换为您的服务器IP
const uint16_t websocket_server_port = 8080;

// 设备设置
const char* API_KEY = "sensor_device_key_123"; // 与服务器配置中的密钥匹配
const char* DEVICE_ID = "esp32-sensor-001";    // 设备唯一ID
const char* BATTERY_ID = "battery-esp32-001";  // 电池终端ID

// 数据发送间隔设置(毫秒)
const int SENSOR_DATA_INTERVAL = 2000;  // 传感器数据发送间隔
const int BATTERY_DATA_INTERVAL = 1000; // 电池数据发送间隔

// 功能启用配置
bool enableSensorData = true;  // 是否发送传感器数据
bool enableBatteryData = true; // 是否发送电池数据
bool enableGyroData = true;    // 是否发送陀螺仪数据

// 电池模拟参数
int batteryCells = 6;                // 电池节数
float currentPercent = 75.0;         // 电池电量(百分比)
bool isCharging = false;             // 是否正在充电
bool powerState = true;              // 电源状态
float batteryTemperature = 25.0;     // 电池温度
float power = 10.0;                  // 功率(W)
float basePower = 5.0;               // 基础功率值(W)
float dischargeRate = 0.05;          // 放电速率(%/秒)
float chargeRate = 0.1;              // 充电速率(%/秒)

// 环境传感器模拟数据
float envTemperature = 22.5;         // 环境温度(°C)
float humidity = 55.0;               // 湿度(%)
float pressure = 1013.2;             // 气压(hPa)

// 陀螺仪模拟数据
float gyroX = 0.0;                   // X轴角度(°)
float gyroY = 0.0;                   // Y轴角度(°)
float gyroZ = 0.0;                   // Z轴角度(°)
int motionMode = 0;                  // 运动模式
float motionSpeed = 1.0;             // 运动速度

// 全局变量
bool isConnected = false;            // WebSocket连接状态
bool isAuthenticated = false;        // 认证状态
unsigned long lastSensorSendTime = 0;    // 上次发送传感器数据的时间
unsigned long lastBatterySendTime = 0;   // 上次发送电池数据的时间
unsigned long lastGyroUpdateTime = 0;    // 上次更新陀螺仪数据的时间
int reconnectAttempts = 0;           // 重连尝试次数
const int MAX_RECONNECT_ATTEMPTS = 10;   // 最大重连尝试次数

// 使用websockets命名空间
using namespace websockets;

// 创建WebSocket客户端
WebsocketsClient client;

void setup() {
  // 初始化串口，波特率115200
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n正在启动ESP32传感器与电池客户端...");
  
  // 连接到WiFi
  connectToWiFi();
  
  // 设置WebSocket事件处理程序
  setupWebSocket();
  
  // 连接到WebSocket服务器
  connectToServer();
  
  // 随机初始化一些参数
  randomizeParameters();
  
  // 打印帮助信息
  printHelp();
}

void loop() {
  // 处理串口命令
  handleSerialCommands();
  
  // 保持WebSocket连接和处理消息
  if(client.available()) {
    client.poll();
  }
  
  // 检查连接状态
  if(isConnected && isAuthenticated) {
    unsigned long currentTime = millis();
    
    // 发送传感器数据
    if(enableSensorData && (currentTime - lastSensorSendTime >= SENSOR_DATA_INTERVAL)) {
      updateSensorData();
      sendSensorData();
      lastSensorSendTime = currentTime;
    }
    
    // 发送电池数据
    if(enableBatteryData && (currentTime - lastBatterySendTime >= BATTERY_DATA_INTERVAL)) {
      updateBatteryState();
      sendBatteryData();
      lastBatterySendTime = currentTime;
    }
    
    // 更新陀螺仪数据(更频繁地更新以保持平滑)
    if(currentTime - lastGyroUpdateTime >= 100) {  // 每100ms更新一次
      updateGyroData();
      lastGyroUpdateTime = currentTime;
    }
  }
  else if(!isConnected && (millis() > lastSensorSendTime + 5000)) {
    // 如果未连接且距离上次尝试超过5秒，尝试重新连接
    Serial.println("未连接到服务器，尝试重新连接...");
    connectToServer();
    lastSensorSendTime = millis();
  }
  
  // 小延迟以减轻CPU负担
  delay(10);
}

// 随机初始化参数
void randomizeParameters() {
  batteryCells = 4 + random(4);       // 4-7节电池
  currentPercent = 30 + random(70);   // 30-100%电量
  batteryTemperature = 20 + random(15); // 20-35°C
  basePower = 5 + random(10);         // 5-15W基础功率
  power = basePower;
  
  envTemperature = 18 + random(15);   // 18-33°C
  humidity = 40 + random(40);         // 40-80%
  pressure = 1000 + random(30);       // 1000-1030hPa
  
  motionMode = random(4);             // 0-3种运动模式
  motionSpeed = 0.5 + random(100) / 50.0; // 0.5-2.5速度
  
  Serial.println("参数已随机初始化");
}

// 处理串口命令
void handleSerialCommands() {
  if(Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if(command == "help") {
      printHelp();
    }
    else if(command == "sensor on") {
      enableSensorData = true;
      Serial.println("传感器数据发送已启用");
    }
    else if(command == "sensor off") {
      enableSensorData = false;
      Serial.println("传感器数据发送已禁用");
    }
    else if(command == "battery on") {
      enableBatteryData = true;
      Serial.println("电池数据发送已启用");
    }
    else if(command == "battery off") {
      enableBatteryData = false;
      Serial.println("电池数据发送已禁用");
    }
    else if(command == "gyro on") {
      enableGyroData = true;
      Serial.println("陀螺仪数据发送已启用");
    }
    else if(command == "gyro off") {
      enableGyroData = false;
      Serial.println("陀螺仪数据发送已禁用");
    }
    else if(command == "status") {
      printStatus();
    }
    else if(command == "charge on") {
      isCharging = true;
      Serial.println("电池充电已开启");
    }
    else if(command == "charge off") {
      isCharging = false;
      Serial.println("电池充电已关闭");
    }
    else if(command == "power on") {
      powerState = true;
      Serial.println("电源已开启");
    }
    else if(command == "power off") {
      powerState = false;
      Serial.println("电源已关闭");
    }
    else if(command == "mode 0" || command == "mode 1" || 
            command == "mode 2" || command == "mode 3") {
      motionMode = command.substring(5).toInt();
      Serial.print("运动模式已设置为: ");
      Serial.println(motionMode);
    }
    else if(command == "random") {
      randomizeParameters();
    }
    else {
      Serial.println("未知命令。输入 'help' 获取命令列表。");
    }
  }
}

// 打印帮助信息
void printHelp() {
  Serial.println("\n可用命令:");
  Serial.println("help         - 显示此帮助信息");
  Serial.println("status       - 显示当前状态");
  Serial.println("sensor on    - 启用传感器数据发送");
  Serial.println("sensor off   - 禁用传感器数据发送");
  Serial.println("battery on   - 启用电池数据发送");
  Serial.println("battery off  - 禁用电池数据发送");
  Serial.println("gyro on      - 启用陀螺仪数据发送");
  Serial.println("gyro off     - 禁用陀螺仪数据发送");
  Serial.println("charge on    - 开启电池充电");
  Serial.println("charge off   - 关闭电池充电");
  Serial.println("power on     - 开启电源");
  Serial.println("power off    - 关闭电源");
  Serial.println("mode 0-3     - 设置陀螺仪运动模式(0-3)");
  Serial.println("random       - 随机重置所有参数");
  Serial.println();
}

// 打印当前状态
void printStatus() {
  Serial.println("\n当前状态:");
  Serial.print("连接状态: ");
  Serial.println(isConnected ? "已连接" : "未连接");
  Serial.print("认证状态: ");
  Serial.println(isAuthenticated ? "已认证" : "未认证");
  Serial.print("传感器数据: ");
  Serial.println(enableSensorData ? "启用" : "禁用");
  Serial.print("电池数据: ");
  Serial.println(enableBatteryData ? "启用" : "禁用");
  Serial.print("陀螺仪数据: ");
  Serial.println(enableGyroData ? "启用" : "禁用");
  
  Serial.println("\n电池参数:");
  Serial.print("电池电量: ");
  Serial.print(currentPercent);
  Serial.println("%");
  Serial.print("充电状态: ");
  Serial.println(isCharging ? "充电中" : "未充电");
  Serial.print("电源状态: ");
  Serial.println(powerState ? "开启" : "关闭");
  Serial.print("功率: ");
  Serial.print(power);
  Serial.println("W");
  
  Serial.println("\n传感器参数:");
  Serial.print("环境温度: ");
  Serial.print(envTemperature);
  Serial.println("°C");
  Serial.print("湿度: ");
  Serial.print(humidity);
  Serial.println("%");
  Serial.print("气压: ");
  Serial.print(pressure);
  Serial.println("hPa");
  
  Serial.println("\n陀螺仪参数:");
  Serial.print("角度(X,Y,Z): ");
  Serial.print(gyroX);
  Serial.print("°, ");
  Serial.print(gyroY);
  Serial.print("°, ");
  Serial.print(gyroZ);
  Serial.println("°");
  Serial.print("运动模式: ");
  Serial.println(motionMode);
  Serial.println();
}

// 连接到WiFi
void connectToWiFi() {
  Serial.print("连接到WiFi");
  WiFi.begin(ssid, password);
  
  // 等待连接建立
  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < 20) {
    delay(500);
    Serial.print(".");
    attempt++;
  }
  
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi连接成功!");
    Serial.print("IP地址: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi连接失败. 请检查凭据或重启设备!");
  }
}

// 设置WebSocket事件处理
void setupWebSocket() {
  // 注册消息回调
  client.onMessage([&](WebsocketsMessage message) {
    Serial.print("收到消息: ");
    Serial.println(message.data());
    
    // 解析JSON响应
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, message.data());
    
    if(error) {
      Serial.print("JSON解析失败: ");
      Serial.println(error.c_str());
      return;
    }
    
    // 检查是否是认证响应
    if(doc.containsKey("type") && doc["type"] == "auth_response") {
      if(doc["success"]) {
        Serial.println("认证成功!");
        isAuthenticated = true;
        reconnectAttempts = 0; // 重置重连尝试次数
      } else {
        Serial.print("认证失败: ");
        Serial.println(doc["message"].as<String>());
        isAuthenticated = false;
      }
    }
    
    // 检查是否是电源控制命令
    if(doc.containsKey("type") && doc["type"] == "power_command") {
      if(doc.containsKey("power_state") && doc.containsKey("terminal_id")) {
        String targetId = doc["terminal_id"].as<String>();
        if(targetId == BATTERY_ID) {
          powerState = doc["power_state"].as<bool>();
          Serial.print("收到电源控制命令: ");
          Serial.println(powerState ? "开启" : "关闭");
          
          // 发送电源状态响应
          sendPowerStateResponse(true);
        }
      }
    }
  });

  // 注册事件处理程序
  client.onEvent([&](WebsocketsEvent event, String data) {
    if(event == WebsocketsEvent::ConnectionOpened) {
      Serial.println("WebSocket连接已打开");
      isConnected = true;
      
      // 发送认证请求
      sendAuthRequest();
    } 
    else if(event == WebsocketsEvent::ConnectionClosed) {
      Serial.println("WebSocket连接已关闭");
      isConnected = false;
      isAuthenticated = false;
    } 
    else if(event == WebsocketsEvent::GotPing) {
      Serial.println("收到Ping");
      client.pong();
    }
  });
}

// 连接到WebSocket服务器
void connectToServer() {
  if(reconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
    Serial.println("已达到最大重连尝试次数，请重启设备");
    return;
  }
  
  String serverUrl = "ws://";
  serverUrl += websocket_server_host;
  serverUrl += ":";
  serverUrl += websocket_server_port;
  
  Serial.print("正在连接到服务器: ");
  Serial.println(serverUrl);
  
  bool connected = client.connect(websocket_server_host, websocket_server_port, "/");
  
  if(connected) {
    Serial.println("连接成功!");
    isConnected = true;
    // 在onEvent中会自动发送认证请求
  } else {
    Serial.println("连接失败!");
    isConnected = false;
    reconnectAttempts++;
    
    Serial.print("重连尝试: ");
    Serial.print(reconnectAttempts);
    Serial.print("/");
    Serial.println(MAX_RECONNECT_ATTEMPTS);
  }
}

// 发送认证请求
void sendAuthRequest() {
  Serial.println("发送认证请求...");
  
  // 创建JSON对象
  DynamicJsonDocument doc(1024);
  doc["type"] = "auth";
  doc["apiKey"] = API_KEY;
  doc["isDevice"] = true;
  doc["deviceId"] = DEVICE_ID;
  
  // 添加设备信息
  JsonObject deviceInfo = doc.createNestedObject("deviceInfo");
  deviceInfo["model"] = "ESP32";
  deviceInfo["chipId"] = String((uint32_t)ESP.getEfuseMac(), HEX);
  deviceInfo["macAddress"] = WiFi.macAddress();
  deviceInfo["ipAddress"] = WiFi.localIP().toString();
  deviceInfo["rssi"] = WiFi.RSSI(); // 信号强度
  deviceInfo["firmware"] = "1.0.0";
  deviceInfo["timestamp"] = getISOTimestamp();
  
  // 序列化JSON
  String jsonStr;
  serializeJson(doc, jsonStr);
  
  // 发送消息
  client.send(jsonStr);
}

// 更新环境传感器数据(模拟)
void updateSensorData() {
  // 添加微小随机波动
  envTemperature += (random(20) - 10) / 100.0;
  humidity += (random(20) - 10) / 100.0;
  pressure += (random(20) - 10) / 100.0;
  
  // 保持数值在合理范围内
  envTemperature = constrain(envTemperature, 10, 40);
  humidity = constrain(humidity, 20, 95);
  pressure = constrain(pressure, 980, 1040);
}

// 更新陀螺仪数据(模拟)
void updateGyroData() {
  // 根据运动模式更新陀螺仪数据
  float dt = (millis() - lastGyroUpdateTime) / 1000.0; // 时间增量(秒)
  static float time = 0;
  time += dt;
  
  switch(motionMode) {
    case 0: // 简单旋转模式
      gyroX = 15 * sin(time * motionSpeed);
      gyroY = 15 * cos(time * motionSpeed);
      gyroZ = 5 * sin(time * motionSpeed * 2);
      break;
      
    case 1: // 摇摆模式
      gyroX = 10 * sin(time * motionSpeed);
      gyroY = 5 * sin(time * motionSpeed * 0.5);
      gyroZ = 20 * sin(time * motionSpeed * 0.25);
      break;
      
    case 2: // 旋转模式
      gyroX = 5;
      gyroY = 5;
      gyroZ = 45 * sin(time * motionSpeed * 0.5);
      break;
      
    case 3: // 随机波动模式
      gyroX += (random(20) - 10) / 2.0;
      gyroY += (random(20) - 10) / 2.0;
      gyroZ += (random(20) - 10) / 2.0;
      
      // 防止无限漂移
      gyroX *= 0.95;
      gyroY *= 0.95;
      gyroZ *= 0.95;
      break;
  }
  
  // 限制角度范围在 -180° 到 180°
  gyroX = constrain(gyroX, -180, 180);
  gyroY = constrain(gyroY, -180, 180);
  gyroZ = constrain(gyroZ, -180, 180);
}

// 更新电池状态(模拟)
void updateBatteryState() {
  if(!powerState) {
    // 电源关闭时不更新
    return;
  }
  
  float dt = (millis() - lastBatterySendTime) / 1000.0; // 时间增量(秒)
  
  // 更新电池电量
  if(isCharging) {
    currentPercent += chargeRate * dt;
    if(currentPercent > 100) {
      currentPercent = 100;
      isCharging = false; // 充满后停止充电
      Serial.println("电池已充满");
    }
  } else {
    currentPercent -= dischargeRate * dt;
    if(currentPercent < 15) {
      isCharging = true; // 电量低开始充电
      Serial.println("电池电量低，开始充电");
    }
  }
  
  // 确保电量在0-100范围内
  currentPercent = constrain(currentPercent, 0, 100);
  
  // 更新温度（随机波动）
  batteryTemperature += (random(10) - 5) * 0.05;
  batteryTemperature = constrain(batteryTemperature, 10, 45);
  
  // 更新功率（随机波动）
  if(isCharging) {
    // 充电时功率较高，接近最大值30W
    power = 20 + random(90) / 10.0; // 20-29W
  } else {
    // 放电时功率较低，基于基础功率波动
    float loadFactor = 0.7 + random(60) / 100.0; // 0.7-1.3的负载因子
    power = basePower * loadFactor; // 基础功率的70%-130%
    
    // 随机产生一个功率波峰
    if(random(100) < 5) { // 5%的概率产生波峰
      power = 15 + random(150) / 10.0; // 15-30W
      Serial.print("功率波峰: ");
      Serial.print(power);
      Serial.println("W");
    }
  }
  
  // 保留一位小数
  power = round(power * 10) / 10.0;
}

// 发送电池数据
void sendBatteryData() {
  if(!isConnected || !isAuthenticated) {
    Serial.println("未连接或未认证，无法发送电池数据");
    return;
  }
  
  // 创建JSON对象
  DynamicJsonDocument doc(1024);
  
  // 添加电池终端基本信息
  doc["device_type"] = "battery";
  doc["terminal_id"] = BATTERY_ID;
  doc["device_ip"] = WiFi.localIP().toString();
  doc["interval"] = BATTERY_DATA_INTERVAL;
  doc["timestamp"] = getISOTimestamp();
  
  // 添加电池参数
  doc["cells"] = batteryCells;
  
  // 计算电池电压
  float cellVoltage = 3.0 + (currentPercent / 100.0) * 1.2; // 3.0V-4.2V
  float totalVoltage = cellVoltage * batteryCells;
  doc["voltage"] = String(totalVoltage, 2);
  
  doc["battery_percent"] = round(currentPercent);
  doc["power"] = String(power, 1);
  doc["temperature"] = String(batteryTemperature, 1);
  doc["is_charging"] = isCharging;
  doc["power_state"] = powerState;
  
  // 序列化JSON
  String jsonStr;
  serializeJson(doc, jsonStr);
  
  // 发送数据
  client.send(jsonStr);
  Serial.println("已发送电池数据");
}

// 发送环境传感器数据
void sendSensorData() {
  if(!isConnected || !isAuthenticated) {
    Serial.println("未连接或未认证，无法发送传感器数据");
    return;
  }
  
  // 创建JSON对象
  DynamicJsonDocument doc(1024);
  
  // 添加设备信息
  doc["device_ip"] = WiFi.localIP().toString();
  doc["interval"] = SENSOR_DATA_INTERVAL;
  doc["timestamp"] = getISOTimestamp();
  doc["terminal_id"] = DEVICE_ID;
  
  // 环境数据
  doc["temperature"] = String(envTemperature, 1) + "°C";
  doc["humidity"] = String(humidity, 1) + "%";
  doc["pressure"] = String(pressure, 1) + "hPa";
  
  // 添加陀螺仪数据（如果启用）
  if(enableGyroData) {
    doc["gyro_x"] = String(gyroX, 2) + "°";
    doc["gyro_y"] = String(gyroY, 2) + "°";
    doc["gyro_z"] = String(gyroZ, 2) + "°";
  }
  
  // 序列化JSON
  String jsonStr;
  serializeJson(doc, jsonStr);
  
  // 发送数据
  client.send(jsonStr);
  Serial.println("已发送传感器数据");
}

// 发送电源状态响应
void sendPowerStateResponse(bool success) {
  if(!isConnected || !isAuthenticated) {
    Serial.println("未连接或未认证，无法发送电源状态响应");
    return;
  }
  
  // 创建JSON对象
  DynamicJsonDocument doc(1024);
  
  doc["type"] = "power_state_response";
  doc["terminal_id"] = BATTERY_ID;
  doc["power_state"] = powerState;
  doc["success"] = success;
  doc["timestamp"] = getISOTimestamp();
  
  // 序列化JSON
  String jsonStr;
  serializeJson(doc, jsonStr);
  
  // 发送数据
  client.send(jsonStr);
  Serial.print("已发送电源状态响应: ");
  Serial.println(powerState ? "开启" : "关闭");
}

// 获取ISO格式的时间戳(简化版，没有实际的时间同步)
String getISOTimestamp() {
  // 实际使用需通过NTP同步时间
  // 这里仅返回近似运行时间作为演示
  unsigned long now = millis();
  unsigned long seconds = now / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  
  seconds %= 60;
  minutes %= 60;
  hours %= 24;
  
  char timestamp[30];
  sprintf(timestamp, "2023-05-15T%02d:%02d:%02dZ", hours, minutes, seconds);
  return String(timestamp);
} 