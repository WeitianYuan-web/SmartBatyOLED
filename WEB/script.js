// WebSocket连接
let socket = null;
let reconnectAttempts = 0;
const MAX_RECONNECT_ATTEMPTS = 10;
const BASE_RECONNECT_DELAY = 3000; // 初始重连延迟(ms)

// API密钥（生产环境应通过更安全的方式获取）
const API_KEY = 'test_key_456';

// 连接状态
let serverConnected = false;  // 与服务器的连接状态
let deviceConnected = false;  // 与传感器设备的连接状态

// 电池终端管理
const batteryTerminals = new Map(); // 存储所有电池终端信息，键为终端ID

// 获取显示传感器数据的元素
const sensorDataElement = document.getElementById('sensor-data-content');

// 为不同来源的陀螺仪数据创建存储
const gyroDataSources = new Map(); // 存储各设备的陀螺仪数据
let currentGyroSource = 'default'; // 当前选择的数据源

// 添加初始化函数
function initGyroSourceSelector() {
    const selector = document.getElementById('gyro-data-source');
    if (!selector) return;
    
    // 添加选择事件监听
    selector.addEventListener('change', function() {
        currentGyroSource = this.value;
        addLogEntry(`已切换数据源: ${currentGyroSource}`, 'info');
        
        // 如果有该源的数据，立即更新可视化和传感器数据
        const data = gyroDataSources.get(currentGyroSource);
        if (data) {
            // 更新3D可视化
            updateCubeRotation(data);
            
            // 更新传感器数据显示
            updateSensorData(data);
            
            // 更新状态面板
            updateStatusPanel(data);
        }
    });
}

// 创建WebSocket连接
function connect() {
    // 如果已经存在连接，先关闭
    if (socket) {
        socket.close();
    }

    // 重置连接状态
    serverConnected = false;
    deviceConnected = false;
    clearAllBatteryTerminals();
    updateConnectionStatus();

    socket = new WebSocket('ws://localhost:8080');

    // 连接成功时
    socket.onopen = function() {
        console.log('WebSocket连接已建立');
        addLogEntry('WebSocket连接已建立');
        
        // 发送认证请求
        authenticate();
    };

    // 接收到消息时
    socket.onmessage = function(event) {
        try {
            const response = JSON.parse(event.data);
            
            // 处理不同类型的服务器响应
            switch(response.type) {
                case 'auth_response':
                    handleAuthResponse(response);
                    break;
                    
                case 'device_status':
                    handleDeviceStatus(response);
                    break;
                    
                case 'battery_status':
                    handleBatteryStatus(response);
                    break;
                    
                case 'battery_list':
                    handleBatteryList(response);
                    break;
                    
                case 'power_state_response':
                    handlePowerStateResponse(response);
                    break;
                    
                case 'info':
                    addLogEntry(response.message, 'info');
                    break;
                    
                case 'error':
                    addLogEntry('服务器错误: ' + response.message, 'error');
                    break;
                    
                default:
                    // 处理传感器数据（没有type字段的消息）
                    if (!response.type) {
                        // 检查是否是电池数据
                        if (response.device_type === 'battery' && response.terminal_id) {
                            // 处理电池数据
                            updateBatteryTerminalData(response);
                            
                            // 如果电池数据中包含陀螺仪数据，也存储起来
                            if (response.gyro_x !== undefined && 
                                response.gyro_y !== undefined && 
                                response.gyro_z !== undefined) {
                                
                                // 使用终端ID作为数据源标识
                                const sourceId = response.terminal_id;
                                
                                // 存储该源的数据
                                gyroDataSources.set(sourceId, response);
                                
                                // 更新下拉菜单
                                updateGyroSourceSelector();
                                
                                // 如果是当前选中的源，或者是第一个源且当前是默认源
                                if (sourceId === currentGyroSource || 
                                    (currentGyroSource === 'default' && gyroDataSources.size === 1)) {
                                    currentGyroSource = sourceId;
                                    updateCubeRotation(response);
                                }
                            }
                        } else {
                            // 验证传感器数据格式完整性
                            if (!validateSensorData(response)) {
                                throw new Error('数据格式不完整或无效');
                            }
                            
                            // 收到有效的传感器数据，说明至少有一个设备已连接
                            if (!deviceConnected) {
                                deviceConnected = true;
                                updateConnectionStatus();
                                addLogEntry('传感器设备数据已接收', 'info');
                            }
                            
                            // 为非电池终端的传感器添加数据源
                            if (response.device_ip) {
                                const sourceId = `sensor-${response.device_ip}`;
                                gyroDataSources.set(sourceId, response);
                                updateGyroSourceSelector();
                                
                                // 更新可视化（如果是当前选择的源）
                                if (sourceId === currentGyroSource || 
                                    (currentGyroSource === 'default' && gyroDataSources.size === 1)) {
                                    currentGyroSource = sourceId;
                                    updateCubeRotation(response);
                                    updateSensorData(response);
                                    updateStatusPanel(response);
                                }
                            } else {
                                // 没有源标识的数据，使用默认处理
                                updateSensorData(response);
                                updateStatusPanel(response);
                                updateCubeRotation(response);
                            }
                        }
                    }
            }
        } catch (e) {
            addLogEntry('数据解析错误：' + e.message, 'error');
        }
    };

    // 处理错误
    socket.onerror = function(error) {
        console.error('WebSocket错误：', error);
        addLogEntry('WebSocket错误：' + (error.message || '未知错误'), 'error');
        serverConnected = false;
        updateConnectionStatus();
    };

    // 连接关闭时
    socket.onclose = function(event) {
        console.log('WebSocket连接已关闭, 代码:', event.code);
        serverConnected = false;
        deviceConnected = false;
        updateConnectionStatus();
        
        // 添加详细的关闭原因
        let closeReason = '';
        if (event.code === 1000) closeReason = '正常关闭';
        else if (event.code === 1001) closeReason = '离开页面';
        else if (event.code === 1008) closeReason = '策略违规';
        else if (event.code === 1011) closeReason = '服务器错误';
        else if (event.code === 1013) closeReason = '服务器过载';
        
        addLogEntry(`WebSocket连接已关闭 (${event.code}: ${closeReason || event.reason || '未知原因'})`, 'error');
        
        // 优化重连逻辑 - 添加指数退避和最大重试限制
        if (reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
            const delay = BASE_RECONNECT_DELAY * Math.pow(1.5, reconnectAttempts);
            reconnectAttempts++;
            
            addLogEntry(`尝试重新连接 (${reconnectAttempts}/${MAX_RECONNECT_ATTEMPTS})，等待 ${Math.round(delay/1000)} 秒...`);
            
            setTimeout(() => {
                console.log(`尝试重新连接 (${reconnectAttempts}/${MAX_RECONNECT_ATTEMPTS})...`);
                connect();
            }, delay);
        } else {
            addLogEntry('已达到最大重连尝试次数，请手动刷新页面重试', 'error');
        }
    };

    // 添加这一行初始化下拉菜单
    initGyroSourceSelector();
}

// 处理设备状态更新
function handleDeviceStatus(response) {
    if (response.connected !== undefined) {
        deviceConnected = response.connected;
        updateConnectionStatus();
        
        if (deviceConnected) {
            addLogEntry('传感器设备已连接', 'info');
        } else {
            addLogEntry('传感器设备已断开连接', 'error');
        }
    }
}

// 发送认证请求
function authenticate() {
    if (socket && socket.readyState === WebSocket.OPEN) {
        // 发送认证消息
        socket.send(JSON.stringify({
            type: 'auth',
            apiKey: API_KEY,
            isDevice: false,  // 标识为网页客户端
            clientInfo: {
                userAgent: navigator.userAgent,
                language: navigator.language,
                timestamp: new Date().toISOString()
            }
        }));
        
        addLogEntry('正在进行连接认证...', 'info');
    } else {
        console.error('无法发送认证请求: WebSocket未连接');
    }
}

// 处理认证响应
function handleAuthResponse(response) {
    if (response.success) {
        serverConnected = true;
        updateConnectionStatus();
        addLogEntry('服务器认证成功', 'info');
        
        // 现在服务器已连接，但设备状态仍然未知，直到收到设备数据
        addLogEntry('等待传感器设备数据...', 'info');
        
        // 重置重连计数
        reconnectAttempts = 0;
    } else {
        addLogEntry('认证失败: ' + response.message, 'error');
        // 不自动重连，认证失败通常需要人工干预
        reconnectAttempts = MAX_RECONNECT_ATTEMPTS;
    }
}

// 验证传感器数据的完整性
function validateSensorData(data) {
    // 检查必要字段是否存在
    const requiredFields = ['device_ip', 'interval'];
    const sensorFields = ['temperature', 'humidity', 'pressure', 'gyro_x', 'gyro_y', 'gyro_z'];
    
    // 至少需要一个传感器数据字段
    let hasSensorData = false;
    
    // 检查必要字段
    for (const field of requiredFields) {
        if (data[field] === undefined) {
            console.warn(`数据缺少必要字段: ${field}`);
            return false;
        }
    }
    
    // 检查是否有至少一个传感器数据
    for (const field of sensorFields) {
        if (data[field] !== undefined) {
            hasSensorData = true;
            // 检查传感器数据是否可以解析为数值
            if (isNaN(parseFloat(data[field]))) {
                console.warn(`传感器数据无效: ${field} = ${data[field]}`);
                return false;
            }
        }
    }
    
    return hasSensorData;
}

// 更新3D可视化旋转
function updateCubeRotation(data) {
    try {
        // 只有当所有陀螺仪数据都存在时才更新
        if (data.gyro_x !== undefined && data.gyro_y !== undefined && data.gyro_z !== undefined) {
            // 提取角度数值（去掉单位°）
            const gyroX = parseFloat(data.gyro_x);
            const gyroY = parseFloat(data.gyro_y);
            const gyroZ = parseFloat(data.gyro_z);
            
            if (!isNaN(gyroX) && !isNaN(gyroY) && !isNaN(gyroZ)) {
                // 检查值的合理范围 (-180到180度)
                if (gyroX >= -180 && gyroX <= 180 && 
                    gyroY >= -180 && gyroY <= 180 && 
                    gyroZ >= -180 && gyroZ <= 180) {
                    cubeVisualizer.updateRotation(gyroX, gyroY, gyroZ);
                } else {
                    console.warn('陀螺仪数据超出合理范围:', {gyroX, gyroY, gyroZ});
                }
            }
        }
    } catch (e) {
        console.error('更新3D旋转时出错:', e);
    }
}

// 更新传感器数据显示
function updateSensorData(data) {
    let html = '';
    
    // 环境数据
    html += '<div class="data-section"><h3>环境数据</h3>';
    ['temperature', 'humidity', 'pressure'].forEach(key => {
        if (data[key]) {
            let label = translateLabel(key);
            html += `<p><strong>${label}</strong><span class="value">${data[key]}</span></p>`;
        }
    });
    html += '</div>';
    
    // 陀螺仪数据
    html += '<div class="data-section"><h3>陀螺仪数据</h3>';
    ['gyro_x', 'gyro_y', 'gyro_z'].forEach(key => {
        if (data[key]) {
            let label = translateLabel(key);
            html += `<p><strong>${label}</strong><span class="value">${data[key]}</span></p>`;
        }
    });
    html += '</div>';
    
    sensorDataElement.innerHTML = html;
}

// 添加标签翻译函数
function translateLabel(key) {
    const labels = {
        'temperature': '温度',
        'humidity': '湿度',
        'pressure': '气压',
        'gyro_x': 'X轴角度',
        'gyro_y': 'Y轴角度',
        'gyro_z': 'Z轴角度'
    };
    return labels[key] || key;
}

// 更新状态面板
function updateStatusPanel(data) {
    document.getElementById('device-ip').textContent = data.device_ip || '--';
    document.getElementById('update-interval').textContent = data.interval + 'ms' || '--';
    document.getElementById('last-update').textContent = new Date().toLocaleTimeString();
}

// 添加日志条目
function addLogEntry(message, type = 'info') {
    const logContainer = document.getElementById('error-log');
    const entry = document.createElement('div');
    entry.className = `log-entry ${type}`;
    entry.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
    logContainer.insertBefore(entry, logContainer.firstChild);
    
    // 限制日志条目数量
    if (logContainer.children.length > 50) {
        logContainer.removeChild(logContainer.lastChild);
    }
}

// 更新连接状态
function updateConnectionStatus() {
    const statusElement = document.getElementById('connection-status');
    
    if (!serverConnected) {
        // 服务器未连接
        statusElement.textContent = '未连接';
        statusElement.className = 'status-value disconnected';
    } else if (!deviceConnected) {
        // 服务器已连接，但无设备
        statusElement.textContent = '已连接服务器(无设备)';
        statusElement.className = 'status-value partial';
    } else {
        // 服务器和设备都已连接
        statusElement.textContent = '已连接设备';
        statusElement.className = 'status-value connected';
    }
}

// 处理电池状态更新 - 修改为支持多个电池
function handleBatteryStatus(response) {
    // 可能接收已连接/已断开的单个电池终端ID
    if (response.terminal_id) {
        if (response.connected === true) {
            addLogEntry(`电池终端 ${response.terminal_id} 已连接`, 'info');
        } else if (response.connected === false) {
            addLogEntry(`电池终端 ${response.terminal_id} 已断开连接`, 'error');
            removeBatteryTerminal(response.terminal_id);
        }
    }
    
    // 更新电池终端计数
    updateBatteryTerminalCount();
}

// 处理电池列表 - 新增函数
function handleBatteryList(response) {
    if (response.terminals && Array.isArray(response.terminals)) {
        // 清除当前所有电池终端
        clearAllBatteryTerminals();
        
        // 添加返回的所有电池终端
        response.terminals.forEach(terminal => {
            if (terminal.id) {
                createBatteryTerminal(terminal.id);
                if (terminal.data) {
                    updateBatteryTerminalData(terminal.data);
                }
            }
        });
        
        // 更新计数
        updateBatteryTerminalCount();
        
        addLogEntry(`已加载 ${response.terminals.length} 个电池终端`, 'info');
    }
}

// 清除所有电池终端UI和数据
function clearAllBatteryTerminals() {
    // 清除UI中的所有电池终端卡片
    const container = document.getElementById('battery-terminals-container');
    const headerCard = container.querySelector('.battery-header-card');
    
    // 保留header卡片，移除其他所有电池终端卡片
    while (container.children.length > 1) {
        if (container.lastChild !== headerCard) {
            container.removeChild(container.lastChild);
        }
    }
    
    // 清空电池终端数据Map
    batteryTerminals.clear();
    
    // 更新计数
    updateBatteryTerminalCount();
}

// 更新电池终端计数
function updateBatteryTerminalCount() {
    document.getElementById('battery-count').textContent = batteryTerminals.size;
}

// 创建新的电池终端UI元素
function createBatteryTerminal(terminalId) {
    // 如果已存在，则不重复创建
    if (batteryTerminals.has(terminalId)) {
        return batteryTerminals.get(terminalId).element;
    }
    
    // 从模板创建新的电池终端元素
    const template = document.getElementById('battery-terminal-template');
    const clone = document.importNode(template.content, true).children[0];
    
    // 设置ID和初始值
    clone.setAttribute('data-terminal-id', terminalId);
    clone.querySelector('.terminal-id').textContent = `ID: ${terminalId}`;
    
    // 添加电源按钮事件处理
    const powerButton = clone.querySelector('.power-btn');
    powerButton.addEventListener('click', function() {
        // 获取当前状态
        const currentState = this.getAttribute('data-power-state') === 'on';
        // 发送相反的命令
        sendPowerCommand(terminalId, !currentState);
        // 暂时禁用按钮，防止重复点击
        this.disabled = true;
        setTimeout(() => {
            // 如果没有收到响应，3秒后重新启用按钮
            if (this.disabled) {
                this.disabled = false;
            }
        }, 3000);
    });
    
    // 添加到容器
    const container = document.getElementById('battery-terminals-container');
    container.appendChild(clone);
    
    // 存储在电池终端Map中
    batteryTerminals.set(terminalId, {
        id: terminalId,
        element: clone,
        connected: false,
        powerState: null,
        lastUpdate: new Date()
    });
    
    // 更新计数
    updateBatteryTerminalCount();
    
    return clone;
}

// 删除电池终端
function removeBatteryTerminal(terminalId) {
    if (!batteryTerminals.has(terminalId)) return;
    
    // 获取元素
    const terminalInfo = batteryTerminals.get(terminalId);
    
    // 如果元素存在，从DOM中移除
    if (terminalInfo && terminalInfo.element) {
        terminalInfo.element.remove();
    }
    
    // 从Map中移除
    batteryTerminals.delete(terminalId);
    
    // 更新计数
    updateBatteryTerminalCount();
}

// 更新电池终端数据
function updateBatteryTerminalData(data) {
    // 必须有终端ID
    if (!data.terminal_id) {
        console.warn('电池数据缺少终端ID');
        return;
    }
    
    const terminalId = data.terminal_id;
    
    // 如果终端不存在，创建新的
    if (!batteryTerminals.has(terminalId)) {
        createBatteryTerminal(terminalId);
    }
    
    const terminalInfo = batteryTerminals.get(terminalId);
    const element = terminalInfo.element;
    
    // 更新连接状态
    terminalInfo.connected = true;
    terminalInfo.lastUpdate = new Date();
    
    // 更新UI状态
    const connectionBadge = element.querySelector('.connection-badge');
    connectionBadge.textContent = '已连接';
    connectionBadge.classList.add('connected');
    
    // 更新电池基本信息
    if (data.device_ip) {
        element.querySelector('.battery-ip').textContent = data.device_ip;
    }
    
    // 更新最后更新时间
    element.querySelector('.battery-update').textContent = new Date().toLocaleTimeString();
    
    // 更新电源状态（如果数据中包含）
    if (data.power_state !== undefined) {
        terminalInfo.powerState = data.power_state;
        updateTerminalPowerState(element, data.power_state);
    }
    
    // 更新电池参数
    if (data.cells) {
        element.querySelector('.battery-cells').textContent = data.cells + 'S';
    }
    
    if (data.voltage) {
        element.querySelector('.battery-voltage').textContent = data.voltage + 'V';
    }
    
    if (data.power) {
        element.querySelector('.battery-power').textContent = data.power + 'W';
        
        // 更新功率柱状图
        updatePowerBar(element, parseFloat(data.power));
    }
    
    if (data.temperature) {
        element.querySelector('.battery-temperature').textContent = data.temperature + '°C';
    }
    
    // 更新电池电量显示
    let percent = 0;
    if (data.battery_percent !== undefined) {
        percent = data.battery_percent;
    } else if (data.voltage && data.cells) {
        // 如果没有直接提供电量百分比，尝试根据电压和电池节数估算
        // 假设每节电池的标称电压为3.7V，满电为4.2V，最低为3.0V
        const cellVoltage = parseFloat(data.voltage) / parseInt(data.cells);
        percent = (cellVoltage - 3.0) / (4.2 - 3.0) * 100;
        percent = Math.max(0, Math.min(100, Math.round(percent)));
    }
    
    updateBatteryLevelUI(element, percent);
}

// 新增函数：更新功率柱状图
function updatePowerBar(element, power) {
    // 功率最大值为30W
    const MAX_POWER = 30;
    
    // 计算百分比
    let percent = (power / MAX_POWER) * 100;
    
    // 限制在0-100范围内
    percent = Math.max(0, Math.min(100, percent));
    
    // 更新柱状图高度
    const powerBar = element.querySelector('.power-bar');
    powerBar.style.height = percent + '%';
    
    // 更新功率文本
    const powerPercent = element.querySelector('.power-percent');
    powerPercent.textContent = `${power.toFixed(1)}W / ${MAX_POWER}W`;
    
    // 根据功率大小调整颜色
    powerBar.className = 'power-bar';
    if (percent > 80) {
        powerBar.classList.add('high');
    } else if (percent > 50) {
        powerBar.classList.add('medium');
    } else if (percent > 20) {
        powerBar.classList.add('low');
    } else {
        powerBar.classList.add('very-low');
    }
}

// 更新电池电量UI
function updateBatteryLevelUI(element, percent) {
    const batteryLevel = element.querySelector('.battery-level');
    const batteryPercent = element.querySelector('.battery-percent');
    
    // 更新电量百分比文本
    batteryPercent.textContent = Math.round(percent) + '%';
    
    // 更新电池图标填充
    batteryLevel.style.width = percent + '%';
    
    // 根据电量设置不同的颜色
    batteryLevel.className = 'battery-level';
    if (percent <= 20) {
        batteryLevel.classList.add('low');
    } else if (percent <= 50) {
        batteryLevel.classList.add('medium');
    }
}

// 处理电源状态响应
function handlePowerStateResponse(response) {
    if (!response.terminal_id) {
        console.warn('电源状态响应缺少终端ID');
        return;
    }
    
    const terminalId = response.terminal_id;
    const success = response.success === true;
    const powerState = response.power_state;
    
    // 查找对应的终端
    if (!batteryTerminals.has(terminalId)) {
        console.warn(`收到未知终端(${terminalId})的电源状态响应`);
        return;
    }
    
    const terminalInfo = batteryTerminals.get(terminalId);
    
    if (success) {
        // 更新电源状态
        terminalInfo.powerState = powerState;
        
        // 更新UI
        updateTerminalPowerState(terminalInfo.element, powerState);
        
        addLogEntry(`终端 ${terminalId} 的电源状态已${powerState ? '开启' : '关闭'}`, 'info');
    } else {
        addLogEntry(`终端 ${terminalId} 的电源状态更改失败: ${response.message || '未知错误'}`, 'error');
    }
}

// 发送电源状态命令
function sendPowerCommand(terminalId, powerState) {
    if (!socket || socket.readyState !== WebSocket.OPEN) {
        addLogEntry('无法发送电源命令: WebSocket未连接', 'error');
        return false;
    }
    
    // 构造命令消息
    const command = {
        type: 'power_command',
        terminal_id: terminalId,
        power_state: powerState,
        timestamp: new Date().toISOString()
    };
    
    // 发送命令
    socket.send(JSON.stringify(command));
    
    addLogEntry(`已发送电源${powerState ? '开启' : '关闭'}命令到终端 ${terminalId}`, 'info');
    
    return true;
}

// 更新电池终端的电源状态UI
function updateTerminalPowerState(element, powerState) {
    // 更新电源状态文本
    const stateText = element.querySelector('.battery-power-state');
    stateText.textContent = powerState ? '开启' : '关闭';
    
    // 更新CSS类
    stateText.className = 'battery-power-state status-value';
    stateText.classList.add(powerState ? 'on' : 'off');
    
    // 更新电源按钮
    const powerButton = element.querySelector('.power-btn');
    powerButton.className = 'btn power-btn';
    powerButton.classList.add(powerState ? 'on' : 'off');
    powerButton.disabled = false;
    
    // 更新数据属性以跟踪当前状态
    powerButton.setAttribute('data-power-state', powerState ? 'on' : 'off');
}

// 添加新函数：更新陀螺仪数据源选择器
function updateGyroSourceSelector() {
    const selector = document.getElementById('gyro-data-source');
    if (!selector) return;
    
    // 保存当前选中的值
    const currentValue = selector.value;
    
    // 清除所有选项（除了默认选项）
    while (selector.options.length > 1) {
        selector.remove(1);
    }
    
    // 添加所有可用的数据源
    for (const [sourceId, data] of gyroDataSources.entries()) {
        const option = document.createElement('option');
        option.value = sourceId;
        
        // 只显示设备ID，简化显示内容
        if (sourceId.startsWith('battery-')) {
            // 从 "battery-123" 提取 "123" 部分
            const deviceId = sourceId.substring(8);
            option.text = `电池终端 ${deviceId}`;
        } else if (data.terminal_id) {
            // 如果有终端ID属性，直接使用
            option.text = data.terminal_id;
        } else if (sourceId.startsWith('sensor-')) {
            // 提取传感器ID
            const deviceId = sourceId.split('-').pop(); // 获取最后一部分
            option.text = `传感器 ${deviceId}`;
        } else {
            // 使用原始ID作为后备
            option.text = sourceId;
        }
        
        selector.add(option);
    }
    
    // 恢复选中状态，如果之前选中的不存在，则选择第一个
    if (gyroDataSources.has(currentValue)) {
        selector.value = currentValue;
    } else if (gyroDataSources.size > 0 && currentValue === 'default') {
        // 如果之前是默认并且有数据源，选择第一个数据源
        selector.selectedIndex = 1;
        currentGyroSource = selector.value;
    }
}

// 初始化连接
connect();