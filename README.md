SmartBaty电池监控系统
项目简介
SmartBatOLED是一款智能电池监控和降压转换终端,用于实时监测电池和提供5V和3.3V，可为树莓派等供电。系统通过OLED显示器直观展示电池信息，同时支持WiFi连接将数据传输至服务器。
实时监测：监控电池电压、电流、功率和温度
OLED显示：直观显示电池参数和设备状态
远程数据传输：通过WebSocket协议实时上传数据
远程控制：支持远程开关电源
自动适配：自动识别1-4S锂电池
可配置性：丰富的参数配置选项
数据过滤：采用滤波算法提高测量精度
配置保存：断电保存各项设置
丰富的调试信息：完整的日志和故障诊断
硬件要求
ESP32C3开发板
INA226电流传感器模块
SSD1306 OLED显示屏(128x32)
4.7kΩ上拉电阻(用于I2C总线)
可选：BMI270姿态传感器
接线指南
| 组件 | 连接方式 |
|------|----------|
| INA226 | SDA → GPIO8, SCL → GPIO9, VCC → 3.3V, GND → GND |
| OLED显示屏 | SDA → GPIO8, SCL → GPIO9, VCC → 3.3V, GND → GND |
| 电源控制 | EN → GPIO10 |
安装指南
焊接ESP32C3到PCB背面
使用Arduino IDE或PlatformIO编译并上传代码
通过串口监视器(波特率115200)查看启动日志
根据提示配置WiFi和其他参数
使用说明
串口命令
设备支持通过串口发送以下命令:
配置命令格式: config:<参数>=<值>

可用命令:
  config:<参数>=<值> - 配置系统参数 (输入config:help查看详细帮助)
  config:status - 显示当前状态
  en:on - 打开电源
  en:off - 关闭电源

  可配置参数

  可配置参数:
  device_id - 设备ID
  battery_id - 电池ID
  ws_host - WebSocket服务器地址
  ws_port - WebSocket服务器端口
  api_key - WebSocket认证密钥
  wifi_ssid - WiFi名称
  wifi_pass - WiFi密码
  en_state - 电源状态(on/off)
  save - 保存所有配置
  reset - 重置所有配置
  export - 导出配置为JSON
  import=<JSON字符串> - 导入配置
  status - 显示当前设置和状态
  help - 显示此帮助

  OLED显示说明
OLED屏幕分为三个区域:
左上：电池电压和电池类型(1S-4S)
中上：功率显示(W)
右上：电流显示(A)
底部：WiFi状态、设备ID、IP地址和温度
故障排除
如果遇到I2C通信错误"Wire.cpp:513] requestFrom(): i2cRead returned Error"，请尝试:
检查硬件连接：
确认SDA(GPIO8)和SCL(GPIO9)引脚连接正确
检查INA226电源是否接通
添加4.7kΩ上拉电阻到SDA和SCL线
减少干扰：
缩短I2C连接线长度
远离电机、继电器等干扰源
使用屏蔽线缆
验证地址：
确认INA226的地址是否为0x40
使用I2C扫描功能查看总线上的设备
WebSocket协议
系统使用WebSocket协议与服务器通信，发送格式为JSON，包含电池的所有监测数据。服务器可以发送控制命令，例如开关电源。详细协议格式可查阅代码中的相关部分。
