# 更新软件包列表
sudo apt update

# 升级已安装的软件包
sudo apt upgrade -y

# 安装必要的工具
sudo apt install -y curl wget git htop net-tools

# 添加Node.js 16.x仓库
curl -fsSL https://deb.nodesource.com/setup_16.x | sudo -E bash -

# 安装Node.js和npm
sudo apt install -y nodejs

# 验证安装
node --version
npm --version

# 创建应用目录
sudo mkdir -p /opt/battery-monitor
sudo chown -R $USER:$USER /opt/battery-monitor
cd /opt/battery-monitor

# 在应用目录中初始化npm
cd /opt/battery-monitor
npm init -y

# 安装WebSocket服务器依赖
npm install ws

# 启动WebSocket服务器
node websocket-server.js


