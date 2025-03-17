/**
 * 多电池终端模拟器
 * 用于测试多个电池终端连接场景
 */

const WebSocket = require('ws');

// 配置信息
const CONFIG = {
    SERVER_URL: 'ws://192.168.31.128:8080',  // WebSocket服务器地址
    API_KEY: 'test_key_456',   // 设备API密钥
    TERMINAL_COUNT: 2,                  // 要模拟的电池终端数量
    DATA_INTERVAL: 200,                // 数据发送间隔(毫秒)
    MAX_RECONNECT_ATTEMPTS: 10,         // 最大重连尝试次数
    BASE_RECONNECT_DELAY: 3000          // 基本重连延迟(毫秒)
};

// 电池终端类 - 每个实例代表一个电池终端
class BatteryTerminal {
    constructor(id) {
        this.id = id;
        this.socket = null;
        this.reconnectAttempts = 0;
        this.isAuthenticated = false;
        this.intervalId = null;
        
        // 随机初始配置
        this.cells = 4 + Math.floor(Math.random() * 4); // 4S - 7S
        this.baseVoltage = 3.7;
        this.maxVoltage = 4.2;
        this.minVoltage = 3.0;
        this.currentPercent = 30 + Math.floor(Math.random() * 70); // 30% - 100%
        this.dischargeRate = 0.02 + Math.random() * 0.08; // 放电速率(%/秒)
        this.chargeRate = 0.05 + Math.random() * 0.1; // 充电速率(%/秒)
        this.isCharging = this.currentPercent < 30; // 电量低时开始充电
        this.temperature = 20 + Math.random() * 15; // 20°C - 35°C
        
        // 功率值改为最大接近30W
        this.basePower = 5 + Math.random() * 10; // 基础功率5-15W
        this.power = this.basePower; // 初始功率
        
        // 电源状态默认为开启
        this.powerState = true;
        
        console.log(`初始化电池终端 ${this.id}:`, {
            cells: this.cells,
            initialPercent: this.currentPercent,
            isCharging: this.isCharging,
            powerState: this.powerState,
            basePower: this.basePower
        });
    }
    
    // 连接到WebSocket服务器
    connect() {
        if (this.socket) {
            clearInterval(this.intervalId);
            this.socket.close();
        }
        
        console.log(`[终端${this.id}] 正在连接到服务器: ${CONFIG.SERVER_URL}`);
        this.socket = new WebSocket(CONFIG.SERVER_URL);
        
        // 连接成功时
        this.socket.onopen = () => {
            console.log(`[终端${this.id}] WebSocket连接已建立`);
            this.authenticate();
        };
        
        // 接收到消息时
        this.socket.onmessage = (event) => {
            try {
                const response = JSON.parse(event.data);
                console.log(`[终端${this.id}] 收到服务器消息:`, response);
                
                // 处理认证响应
                if (response.type === 'auth_response') {
                    this.handleAuthResponse(response);
                }
                
                // 处理电源控制命令
                if (response.type === 'power_command' && response.power_state !== undefined) {
                    this.handlePowerCommand(response);
                }
            } catch (e) {
                console.error(`[终端${this.id}] 数据解析错误:`, e.message);
            }
        };
        
        // 处理错误
        this.socket.onerror = (error) => {
            console.error(`[终端${this.id}] WebSocket错误:`, error.message || '未知错误');
        };
        
        // 连接关闭时
        this.socket.onclose = (event) => {
            console.log(`[终端${this.id}] WebSocket连接已关闭 (${event.code}: ${event.reason || '未知原因'})`);
            this.isAuthenticated = false;
            clearInterval(this.intervalId);
            
            // 重连逻辑
            if (this.reconnectAttempts < CONFIG.MAX_RECONNECT_ATTEMPTS) {
                const delay = CONFIG.BASE_RECONNECT_DELAY * Math.pow(1.5, this.reconnectAttempts);
                this.reconnectAttempts++;
                
                console.log(`[终端${this.id}] 尝试重新连接 (${this.reconnectAttempts}/${CONFIG.MAX_RECONNECT_ATTEMPTS})，等待 ${Math.round(delay/1000)} 秒...`);
                
                setTimeout(() => {
                    console.log(`[终端${this.id}] 尝试重新连接 (${this.reconnectAttempts}/${CONFIG.MAX_RECONNECT_ATTEMPTS})...`);
                    this.connect();
                }, delay);
            } else {
                console.log(`[终端${this.id}] 已达到最大重连尝试次数`);
            }
        };
    }
    
    // 处理电源控制命令
    handlePowerCommand(command) {
        const newPowerState = command.power_state;
        console.log(`[终端${this.id}] 收到电源控制命令: ${newPowerState ? '开启' : '关闭'}`);
        
        // 模拟延迟处理
        setTimeout(() => {
            // 更新电源状态
            this.powerState = newPowerState;
            
            // 发送状态响应
            this.sendPowerStateResponse(true);
            
            // 如果关闭电源，停止发送数据
            if (!newPowerState && this.intervalId) {
                console.log(`[终端${this.id}] 电源已关闭，停止发送数据`);
                clearInterval(this.intervalId);
                this.intervalId = null;
            }
            
            // 如果开启电源，重新开始发送数据
            if (newPowerState && !this.intervalId && this.isAuthenticated) {
                console.log(`[终端${this.id}] 电源已开启，开始发送数据`);
                this.startSendingData();
            }
        }, 500 + Math.random() * 1000); // 随机延迟500-1500ms
    }
    
    // 发送电源状态响应
    sendPowerStateResponse(success, message) {
        if (!this.socket || this.socket.readyState !== WebSocket.OPEN) {
            console.warn(`[终端${this.id}] 无法发送电源状态响应: WebSocket未连接`);
            return;
        }
        
        const response = {
            type: 'power_state_response',
            terminal_id: `battery-${this.id}`,
            power_state: this.powerState,
            success: success,
            timestamp: new Date().toISOString()
        };
        
        if (message) {
            response.message = message;
        }
        
        this.socket.send(JSON.stringify(response));
        console.log(`[终端${this.id}] 已发送电源状态响应: ${this.powerState ? '开启' : '关闭'}`);
    }
    
    // 发送认证请求
    authenticate() {
        if (this.socket && this.socket.readyState === WebSocket.OPEN) {
            console.log(`[终端${this.id}] 发送认证请求...`);
            this.socket.send(JSON.stringify({
                type: 'auth',
                apiKey: CONFIG.API_KEY,
                isDevice: true,
                deviceId: `battery-${this.id}`,
                deviceInfo: {
                    model: 'Battery Terminal v1.0',
                    deviceType: 'battery',
                    timestamp: new Date().toISOString()
                }
            }));
        } else {
            console.error(`[终端${this.id}] 无法发送认证请求: WebSocket未连接`);
        }
    }
    
    // 处理认证响应
    handleAuthResponse(response) {
        if (response.success) {
            console.log(`[终端${this.id}] 认证成功!`);
            this.isAuthenticated = true;
            this.reconnectAttempts = 0;
            
            // 开始发送模拟数据（如果电源开启）
            if (this.powerState) {
                this.startSendingData();
            } else {
                console.log(`[终端${this.id}] 电源已关闭，不发送数据`);
            }
        } else {
            console.error(`[终端${this.id}] 认证失败:`, response.message);
        }
    }
    
    // 开始定期发送模拟数据
    startSendingData() {
        if (this.intervalId) {
            clearInterval(this.intervalId);
        }
        
        console.log(`[终端${this.id}] 开始发送模拟电池数据，间隔 ${CONFIG.DATA_INTERVAL}ms`);
        
        // 立即发送一次数据
        this.sendBatteryData();
        
        // 设置定期发送
        this.intervalId = setInterval(() => this.sendBatteryData(), CONFIG.DATA_INTERVAL);
    }
    
    // 发送模拟电池数据
    sendBatteryData() {
        if (!this.socket || this.socket.readyState !== WebSocket.OPEN || !this.isAuthenticated) {
            console.warn(`[终端${this.id}] 无法发送数据: 连接未就绪或未认证`);
            return;
        }
        
        // 如果电源关闭，不发送数据
        if (!this.powerState) {
            console.log(`[终端${this.id}] 电源已关闭，跳过数据发送`);
            return;
        }
        
        // 更新模拟电池状态
        this.updateBatteryState();
        
        // 生成模拟数据
        const data = this.generateBatteryData();
        
        // 发送数据
        this.socket.send(JSON.stringify(data));
        console.log(`[终端${this.id}] 已发送电池数据: 电量=${data.battery_percent}%, 电压=${data.voltage}V`);
    }
    
    // 更新电池状态
    updateBatteryState() {
        // 计算时间间隔
        const timeDelta = CONFIG.DATA_INTERVAL / 1000; // 转换为秒
        
        // 更新电量
        if (this.isCharging) {
            this.currentPercent += this.chargeRate * timeDelta;
            if (this.currentPercent > 100) {
                this.currentPercent = 100;
                this.isCharging = false; // 充满后停止充电
                console.log(`[终端${this.id}] 电池已充满`);
            }
        } else {
            this.currentPercent -= this.dischargeRate * timeDelta;
            if (this.currentPercent < 15) {
                this.isCharging = true; // 电量低时开始充电
                console.log(`[终端${this.id}] 电池电量低，开始充电`);
            }
        }
        
        // 确保电量在0-100范围内
        this.currentPercent = Math.max(0, Math.min(100, this.currentPercent));
        
        // 更新温度（随机波动）
        this.temperature += (Math.random() - 0.5) * 0.5;
        this.temperature = Math.round(this.temperature * 10) / 10;
        
        // 更新功率（随机波动）
        if (this.isCharging) {
            // 充电时功率较高，接近最大值30W
            this.power = 20 + Math.random() * 9; // 20-29W
        } else {
            // 放电时功率较低，基于基础功率波动
            const loadFactor = 0.7 + Math.random() * 0.6; // 0.7-1.3的负载因子
            this.power = this.basePower * loadFactor; // 基础功率的70%-130%
            
            // 随机产生一个功率波峰
            if (Math.random() < 0.05) { // 5%的概率产生波峰
                this.power = 15 + Math.random() * 15; // 15-30W的波峰
                console.log(`[终端${this.id}] 功率波峰: ${this.power.toFixed(1)}W`);
            }
        }
        this.power = Math.round(this.power * 10) / 10;
    }
    
    // 生成模拟电池数据
    generateBatteryData() {
        // 获取当前时间
        const now = new Date();
        
        // 计算电池电压
        const voltageRange = this.maxVoltage - this.minVoltage;
        const cellVoltage = this.minVoltage + (voltageRange * this.currentPercent / 100);
        const totalVoltage = cellVoltage * this.cells;
        
        // 随机IP地址
        const ip = `192.168.1.${100 + parseInt(this.id)}`;
        
        return {
            device_type: 'battery',
            terminal_id: `battery-${this.id}`,
            device_ip: ip,
            timestamp: now.toISOString(),
            interval: CONFIG.DATA_INTERVAL,
            
            // 电池参数
            cells: this.cells,
            voltage: totalVoltage.toFixed(2),
            battery_percent: Math.round(this.currentPercent),
            power: this.power.toFixed(1),
            temperature: this.temperature.toFixed(1),
            is_charging: this.isCharging,
            power_state: this.powerState // 添加电源状态字段
        };
    }
    
    // 关闭连接
    close() {
        if (this.socket) {
            console.log(`[终端${this.id}] 关闭连接`);
            clearInterval(this.intervalId);
            this.socket.close();
        }
    }
}

// 创建并管理多个电池终端
class BatterySimulator {
    constructor(count) {
        this.terminals = [];
        this.count = count;
        
        // 创建指定数量的电池终端
        for (let i = 1; i <= count; i++) {
            this.terminals.push(new BatteryTerminal(i));
        }
        
        console.log(`已创建 ${count} 个电池终端模拟器`);
    }
    
    // 启动所有终端
    start() {
        console.log(`启动 ${this.count} 个电池终端...`);
        this.terminals.forEach(terminal => terminal.connect());
    }
    
    // 停止所有终端
    stop() {
        console.log('停止所有电池终端...');
        this.terminals.forEach(terminal => terminal.close());
    }
}

// 启动模拟器
const simulator = new BatterySimulator(CONFIG.TERMINAL_COUNT);
simulator.start();

// 处理程序退出
process.on('SIGINT', () => {
    console.log('接收到退出信号，正在关闭所有连接...');
    simulator.stop();
    setTimeout(() => process.exit(0), 1000);
});

console.log(`电池终端模拟器已启动，模拟 ${CONFIG.TERMINAL_COUNT} 个终端`); 