const WebSocket = require('ws');
const crypto = require('crypto');

// 简单的API密钥验证（实际生产环境应使用更安全的方法）
const VALID_API_KEYS = [
    'sensor_device_key_123',  // 示例密钥
    'test_key_456'            // 测试密钥
];

// 配置选项
const CONFIG = {
    PORT: 8080,
    HEARTBEAT_INTERVAL: 10000,    // 心跳间隔缩短到10秒
    CONNECTION_TIMEOUT: 30000,    // 连接超时时间30秒
    MAX_CLIENTS: 50               // 最大客户端连接数
};

// 创建WebSocket服务器，监听端口
const wss = new WebSocket.Server({ 
    port: CONFIG.PORT,
    maxPayload: 1024 * 50,      // 限制最大消息大小为50KB
    clientTracking: true,       // 跟踪客户端连接
    perMessageDeflate: true     // 启用消息压缩
});

// 存储所有连接的客户端
let clients = new Set();
let deviceClients = new Map(); // 存储设备客户端，键为设备ID
let batteryClients = new Map(); // 存储电池终端客户端，键为终端ID

// 清理断开的连接
function heartbeat() {
    this.isAlive = true;
}

// 数据验证
function validateSensorData(data) {
    // 检查数据格式
    if (!data || typeof data !== 'object') {
        return false;
    }
    
    // 检查必要字段
    const requiredFields = ['device_ip', 'interval'];
    for (const field of requiredFields) {
        if (data[field] === undefined) {
            return false;
        }
    }
    
    // 检查是否是电池数据
    if (data.device_type === 'battery') {
        // 电池数据需要包含以下字段
        const batteryFields = ['voltage', 'temperature', 'cells', 'power'];
        let hasBatteryData = false;
        
        for (const field of batteryFields) {
            if (data[field] !== undefined) {
                hasBatteryData = true;
                // 检查值是否为有效数字
                if (isNaN(parseFloat(data[field]))) {
                    console.warn(`电池数据字段 ${field} 无效: ${data[field]}`);
                    return false;
                }
            }
        }
        
        return hasBatteryData;
    }
    
    // 如果不是电池数据，按照传统传感器数据检查
    // 至少有一个传感器数据
    const sensorFields = ['temperature', 'humidity', 'pressure', 'gyro_x', 'gyro_y', 'gyro_z'];
    let hasSensorData = false;
    
    for (const field of sensorFields) {
        if (data[field] !== undefined) {
            hasSensorData = true;
            // 检查值是否为有效数字
            if (isNaN(parseFloat(data[field]))) {
                console.warn(`传感器数据字段 ${field} 无效: ${data[field]}`);
                return false;
            }
        }
    }
    
    return hasSensorData;
}

// 验证API密钥
function isValidApiKey(key) {
    return VALID_API_KEYS.includes(key);
}

// 广播数据到所有网页客户端
function broadcastToWebClients(data, excludeClient) {
    try {
        if (!validateSensorData(data)) {
            console.warn('数据验证失败，不广播无效数据');
            return false;
        }
        
        let broadcastCount = 0;
        clients.forEach((client) => {
            if (client !== excludeClient && client.readyState === WebSocket.OPEN && !client.isDevice) {
                client.send(JSON.stringify(data));
                broadcastCount++;
            }
        });
        
        console.log(`已将数据广播给 ${broadcastCount} 个客户端`);
        return true;
    } catch (e) {
        console.error('广播数据时出错：', e);
        return false;
    }
}

// 广播设备连接状态到所有网页客户端
function broadcastDeviceStatus(connected) {
    const statusData = {
        type: 'device_status',
        connected: connected,
        timestamp: new Date().toISOString()
    };
    
    let broadcastCount = 0;
    clients.forEach((client) => {
        if (client.readyState === WebSocket.OPEN && !client.isDevice && client.isAuthenticated) {
            client.send(JSON.stringify(statusData));
            broadcastCount++;
        }
    });
    
    console.log(`已将设备状态(${connected ? '已连接' : '已断开'})广播给 ${broadcastCount} 个客户端`);
}

// 广播电池终端连接状态到所有网页客户端
function broadcastBatteryStatus(connected, terminalId) {
    const statusData = {
        type: 'battery_status',
        connected: connected,
        terminal_id: terminalId,
        timestamp: new Date().toISOString()
    };
    
    let broadcastCount = 0;
    clients.forEach((client) => {
        if (client.readyState === WebSocket.OPEN && !client.isDevice && client.isAuthenticated) {
            client.send(JSON.stringify(statusData));
            broadcastCount++;
        }
    });
    
    console.log(`已将电池终端状态(ID:${terminalId}, ${connected ? '已连接' : '已断开'})广播给 ${broadcastCount} 个客户端`);
}

// 向客户端发送当前所有电池终端列表
function sendBatteryTerminalList(clientWs) {
    // 准备终端列表数据
    const terminals = [];
    
    // 遍历所有电池终端
    batteryClients.forEach((ws, deviceId) => {
        const terminal = {
            id: deviceId,
            connected: true
        };
        terminals.push(terminal);
    });
    
    // 发送终端列表
    const listData = {
        type: 'battery_list',
        terminals: terminals,
        count: terminals.length,
        timestamp: new Date().toISOString()
    };
    
    // 如果指定了客户端，只发送给它，否则广播给所有客户端
    if (clientWs && clientWs.readyState === WebSocket.OPEN) {
        clientWs.send(JSON.stringify(listData));
        console.log(`已发送电池终端列表(${terminals.length}个)给单个客户端`);
    } else {
        let count = 0;
        clients.forEach((client) => {
            if (client.readyState === WebSocket.OPEN && !client.isDevice && client.isAuthenticated) {
                client.send(JSON.stringify(listData));
                count++;
            }
        });
        console.log(`已广播电池终端列表(${terminals.length}个)给 ${count} 个客户端`);
    }
}

// 检查是否有设备连接
function hasConnectedDevices() {
    return [...clients].some(client => client.isDevice && client.isAuthenticated && !client.isBatteryTerminal);
}

// 检查是否有电池终端连接
function hasConnectedBatteryTerminals() {
    return [...clients].some(client => client.isDevice && client.isAuthenticated && client.isBatteryTerminal);
}

// 更新设备状态并广播
function updateDeviceStatus() {
    const hasDevices = hasConnectedDevices();
    broadcastDeviceStatus(hasDevices);
}

// 更新电池终端状态并广播
function updateBatteryStatus() {
    const hasBatteryTerminals = hasConnectedBatteryTerminals();
    broadcastBatteryStatus(hasBatteryTerminals);
}

// 处理电源控制命令并转发到相应的电池终端
function handlePowerCommand(ws, data) {
    // 验证命令格式
    if (!data.terminal_id || data.power_state === undefined) {
        sendErrorResponse(ws, 'power_command格式无效');
        return;
    }
    
    const terminalId = data.terminal_id;
    const powerState = Boolean(data.power_state);
    
    console.log(`收到电源控制命令: 终端=${terminalId}, 电源=${powerState ? '开' : '关'}`);
    
    // 检查目标终端是否连接
    if (!batteryClients.has(terminalId)) {
        sendPowerStateResponse(ws, terminalId, false, '目标电池终端未连接');
        return;
    }
    
    // 获取终端客户端连接
    const terminalWs = batteryClients.get(terminalId);
    
    // 检查终端连接状态
    if (terminalWs.readyState !== WebSocket.OPEN) {
        sendPowerStateResponse(ws, terminalId, false, '目标电池终端连接已断开');
        return;
    }
    
    // 向电池终端转发命令
    try {
        const commandMsg = {
            type: 'power_command',
            power_state: powerState,
            timestamp: new Date().toISOString()
        };
        
        terminalWs.send(JSON.stringify(commandMsg));
        
        console.log(`已转发电源控制命令到终端 ${terminalId}`);
        
        // 发送临时状态响应（实际状态将由终端回报）
        sendPowerStateResponse(ws, terminalId, true, null, powerState);
    } catch (err) {
        console.error(`转发电源控制命令失败:`, err);
        sendPowerStateResponse(ws, terminalId, false, '服务器内部错误');
    }
}

// 发送电源状态响应
function sendPowerStateResponse(ws, terminalId, success, message, powerState) {
    const response = {
        type: 'power_state_response',
        terminal_id: terminalId,
        success: success,
        timestamp: new Date().toISOString()
    };
    
    if (message) {
        response.message = message;
    }
    
    if (powerState !== undefined) {
        response.power_state = powerState;
    }
    
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(response));
    }
}

// 发送错误响应
function sendErrorResponse(ws, message) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({
            type: 'error',
            message: message,
            timestamp: new Date().toISOString()
        }));
    }
}

wss.on('connection', function connection(ws, req) {
    // 检查最大连接数
    if (clients.size >= CONFIG.MAX_CLIENTS) {
        console.warn('已达到最大连接数，拒绝新连接');
        ws.close(1013, '服务器已达到最大连接数');
        return;
    }
    
    // 获取客户端IP地址
    const ip = req.headers['x-forwarded-for'] || req.socket.remoteAddress;
    console.log(`新客户端连接 (${ip})`);
    
    // 初始化连接属性
    ws.isAlive = true;
    ws.isDevice = false;
    ws.isBatteryTerminal = false;
    ws.isAuthenticated = false;
    ws.clientIp = ip;
    ws.connectionTime = new Date();
    
    clients.add(ws);
    
    // 设置心跳检测
    ws.on('pong', heartbeat);

    // 添加错误处理
    ws.on('error', (error) => {
        console.error(`WebSocket错误 (${ip}): `, error.message);
        cleanupConnection(ws);
    });

    // 处理来自客户端的消息
    ws.on('message', (message) => {
        try {
            // 解析收到的数据
            const data = JSON.parse(message);
            
            // 处理认证请求
            if (data.type === 'auth') {
                if (isValidApiKey(data.apiKey)) {
                    ws.isAuthenticated = true;
                    ws.isDevice = data.isDevice === true;
                    
                    // 检查是否是电池终端设备
                    if (data.deviceInfo && data.deviceInfo.deviceType === 'battery') {
                        ws.isBatteryTerminal = true;
                    }
                    
                    // 如果是设备，存储设备ID
                    if (ws.isDevice && data.deviceId) {
                        ws.deviceId = data.deviceId;
                        
                        if (ws.isBatteryTerminal) {
                            // 存储电池终端客户端
                            batteryClients.set(data.deviceId, ws);
                            console.log(`电池终端 ${data.deviceId} 已认证 (${ip})`);
                            
                            // 广播特定电池终端的连接状态更新
                            broadcastBatteryStatus(true, data.deviceId);
                        } else {
                            // 存储普通设备客户端
                            deviceClients.set(data.deviceId, ws);
                            console.log(`设备 ${data.deviceId} 已认证 (${ip})`);
                            
                            // 广播设备连接状态更新
                            updateDeviceStatus();
                        }
                    } else {
                        console.log(`网页客户端已认证 (${ip})`);
                        
                        // 网页客户端认证成功后，首先发送设备状态
                        setTimeout(() => {
                            if (ws.readyState === WebSocket.OPEN) {
                                // 发送传感器设备状态
                                const deviceConnected = hasConnectedDevices();
                                ws.send(JSON.stringify({
                                    type: 'device_status',
                                    connected: deviceConnected,
                                    timestamp: new Date().toISOString()
                                }));
                                
                                // 发送所有电池终端列表
                                sendBatteryTerminalList(ws);
                            }
                        }, 1000);
                    }
                    
                    // 发送认证成功响应
                    ws.send(JSON.stringify({
                        type: 'auth_response',
                        success: true,
                        message: '认证成功'
                    }));
                } else {
                    console.warn(`认证失败 (${ip}): 无效的API密钥`);
                    ws.send(JSON.stringify({
                        type: 'auth_response',
                        success: false,
                        message: '认证失败: 无效的API密钥'
                    }));
                    
                    // 延迟关闭连接，给客户端时间处理响应
                    setTimeout(() => {
                        ws.close(1008, '认证失败');
                    }, 1000);
                }
                return;
            }
            
            // 对于非认证请求，检查是否已认证
            if (!ws.isAuthenticated) {
                console.warn(`未认证客户端 (${ip}) 尝试发送数据`);
                ws.send(JSON.stringify({
                    type: 'error',
                    message: '请先进行认证'
                }));
                return;
            }
            
            // 处理电源控制命令
            if (data.type === 'power_command') {
                handlePowerCommand(ws, data);
                return;
            }
            
            // 处理电源状态响应（来自电池终端）
            if (data.type === 'power_state_response' && ws.isBatteryTerminal) {
                // 将响应广播给所有网页客户端
                clients.forEach((client) => {
                    if (client.readyState === WebSocket.OPEN && !client.isDevice && client.isAuthenticated) {
                        client.send(message);
                    }
                });
                return;
            }
            
            console.log(`收到${ws.isDevice ? '设备' : '网页'}数据 (${ip}):`, 
                        JSON.stringify(data).substring(0, 100) + (JSON.stringify(data).length > 100 ? '...' : ''));
            
            // 如果是传感器数据，验证并广播
            if (ws.isDevice && ws.isAuthenticated) {
                if (ws.isBatteryTerminal) {
                    // 添加设备类型标识，确保前端能识别
                    if (!data.device_type) {
                        data.device_type = 'battery';
                    }
                }
                
                broadcastToWebClients(data, ws);
            }
            
            // 在消息处理部分添加：
            console.log('收到原始电池数据:', message.toString());
            
        } catch (e) {
            console.error(`数据解析错误 (${ip}): `, e.message);
            ws.send(JSON.stringify({
                type: 'error',
                message: '数据格式错误: ' + e.message
            }));
            
            // 在验证失败时添加详细日志：
            console.warn('数据验证失败，无效字段:', JSON.stringify(data));
        }
    });

    ws.on('close', (code, reason) => {
        console.log(`客户端断开连接 (${ip}), 代码: ${code}, 原因: ${reason || '未指定'}`);
        
        // 保存连接类型和ID，因为cleanup后这些属性会丢失
        const wasDevice = ws.isDevice;
        const wasBatteryTerminal = ws.isBatteryTerminal;
        const deviceId = ws.deviceId;
        
        cleanupConnection(ws);
        
        // 如果是设备断开，通知所有客户端
        if (wasDevice) {
            // 延迟一点执行，确保清理已完成
            setTimeout(() => {
                if (wasBatteryTerminal && deviceId) {
                    // 广播特定电池终端的断开状态
                    broadcastBatteryStatus(false, deviceId);
                } else {
                    updateDeviceStatus();
                }
            }, 500);
        }
    });
    
    // 发送欢迎消息
    ws.send(JSON.stringify({
        type: 'info',
        message: '已连接到传感器数据服务器，请进行认证'
    }));
});

// 清理断开的连接
function cleanupConnection(ws) {
    clients.delete(ws);
    
    // 如果是设备客户端，从设备映射中移除
    if (ws.isDevice && ws.deviceId) {
        if (ws.isBatteryTerminal) {
            batteryClients.delete(ws.deviceId);
        } else {
            deviceClients.delete(ws.deviceId);
        }
    }
    
    if (ws.isAlive === false) {
        return ws.terminate();
    }
}

// 定期检查连接状态 - 使用更短的心跳间隔
const interval = setInterval(function ping() {
    wss.clients.forEach(function each(ws) {
        if (ws.isAlive === false) {
            console.log(`心跳超时，终止连接 (${ws.clientIp})`);
            return cleanupConnection(ws);
        }
        
        ws.isAlive = false;
        ws.ping(() => {});
        
        // 检查连接时间，超过超时时间且未认证的连接将被关闭
        if (!ws.isAuthenticated && (new Date() - ws.connectionTime) > CONFIG.CONNECTION_TIMEOUT) {
            console.log(`未认证连接超时 (${ws.clientIp})`);
            ws.close(1008, '认证超时');
            return cleanupConnection(ws);
        }
    });
}, CONFIG.HEARTBEAT_INTERVAL);

// 服务器关闭时清理
wss.on('close', function close() {
    clearInterval(interval);
    console.log('WebSocket服务器已关闭');
});

// 添加服务器错误处理
wss.on('error', (error) => {
    console.error('WebSocket服务器错误：', error.message);
});

// 输出服务器状态信息
function printServerStats() {
    const stats = {
        totalConnections: clients.size,
        deviceConnections: [...clients].filter(c => c.isDevice && !c.isBatteryTerminal).length,
        batteryConnections: [...clients].filter(c => c.isDevice && c.isBatteryTerminal).length,
        webConnections: [...clients].filter(c => !c.isDevice).length,
        authenticatedConnections: [...clients].filter(c => c.isAuthenticated).length
    };
    
    console.log('服务器状态:', stats);
}

// 每分钟输出服务器状态
setInterval(printServerStats, 60000);

console.log(`WebSocket服务器已启动，监听端口：${CONFIG.PORT}`);
console.log('配置信息:', {
    心跳间隔: CONFIG.HEARTBEAT_INTERVAL + 'ms',
    连接超时: CONFIG.CONNECTION_TIMEOUT + 'ms',
    最大连接数: CONFIG.MAX_CLIENTS
});