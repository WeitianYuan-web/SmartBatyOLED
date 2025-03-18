class CubeVisualizer {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(75, this.container.clientWidth / this.container.clientHeight, 0.1, 1000);
        
        // 设置渲染器
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.renderer.setClearColor(0xf8f9fa);
        this.container.appendChild(this.renderer.domElement);

        // 创建正方体
        const geometry = new THREE.BoxGeometry(2, 2, 2);
        
        // 创建带标记的材质，区分不同面
        const materials = [
            new THREE.MeshPhongMaterial({ color: 0x2980b9, transparent: true, opacity: 0.9 }), // 右
            new THREE.MeshPhongMaterial({ color: 0x3498db, transparent: true, opacity: 0.9 }), // 左
            new THREE.MeshPhongMaterial({ color: 0x27ae60, transparent: true, opacity: 0.9 }), // 上
            new THREE.MeshPhongMaterial({ color: 0x2ecc71, transparent: true, opacity: 0.9 }), // 下
            new THREE.MeshPhongMaterial({ color: 0xe74c3c, transparent: true, opacity: 0.9 }), // 前
            new THREE.MeshPhongMaterial({ color: 0xc0392b, transparent: true, opacity: 0.9 })  // 后
        ];
        
        this.cube = new THREE.Mesh(geometry, materials);
        
        // 添加边框
        const edges = new THREE.EdgesGeometry(geometry);
        const lineMaterial = new THREE.LineBasicMaterial({ color: 0x34495e, linewidth: 2 });
        const wireframe = new THREE.LineSegments(edges, lineMaterial);
        this.cube.add(wireframe);

        this.scene.add(this.cube);

        // 添加光源
        const light = new THREE.DirectionalLight(0xffffff, 1);
        light.position.set(5, 5, 5);
        this.scene.add(light);
        this.scene.add(new THREE.AmbientLight(0xffffff, 0.5));

        // 修改相机位置到右上方
        this.camera.position.set(5, 5, 5);  // x, y, z 坐标都设为5，实现右上45度视角
        this.camera.lookAt(0, 0, 0);        // 相机看向原点

        // 添加坐标轴和标签
        this.addAxesWithLabels();
        
        // 添加方向指示器
        this.addOrientationIndicator();

        // 开始动画循环
        this.animate();

        // 处理窗口大小变化
        window.addEventListener('resize', () => this.onWindowResize());
    }

    addAxesWithLabels() {
        // 添加坐标轴
        const axesHelper = new THREE.AxesHelper(3);
        this.scene.add(axesHelper);
        
        // 添加坐标轴标签
        const addLabel = (text, position, color) => {
            const canvas = document.createElement('canvas');
            canvas.width = 128;
            canvas.height = 64;
            const context = canvas.getContext('2d');
            context.fillStyle = '#ffffff';
            context.fillRect(0, 0, canvas.width, canvas.height);
            context.font = '48px Arial';
            context.fillStyle = color;
            context.textAlign = 'center';
            context.textBaseline = 'middle';
            context.fillText(text, canvas.width / 2, canvas.height / 2);
            
            const texture = new THREE.CanvasTexture(canvas);
            const material = new THREE.SpriteMaterial({ map: texture });
            const sprite = new THREE.Sprite(material);
            sprite.position.copy(position);
            sprite.scale.set(0.5, 0.25, 1);
            this.scene.add(sprite);
        };
        
        // 添加 X, Y, Z 标签
        addLabel('X', new THREE.Vector3(3.2, 0, 0), '#ff0000');
        addLabel('Y', new THREE.Vector3(0, 3.2, 0), '#00ff00');
        addLabel('Z', new THREE.Vector3(0, 0, 3.2), '#0000ff');
    }
    
    addOrientationIndicator() {
        // 添加一个小箭头表示"前"方向
        const frontArrow = new THREE.ConeGeometry(0.3, 0.6, 16);
        const arrowMaterial = new THREE.MeshPhongMaterial({ color: 0xe74c3c });
        const frontIndicator = new THREE.Mesh(frontArrow, arrowMaterial);
        frontIndicator.position.set(0, 0, 1.3);
        frontIndicator.rotation.x = Math.PI / 2;
        this.cube.add(frontIndicator);
        
        // 添加"上"方向指示器
        const topIndicator = new THREE.Mesh(
            new THREE.CylinderGeometry(0.1, 0.1, 0.4, 16),
            new THREE.MeshPhongMaterial({ color: 0x27ae60 })
        );
        topIndicator.position.set(0, 1.3, 0);
        this.cube.add(topIndicator);
    }

    updateRotation(x, y, z) {
        // 将角度转换为弧度
        const degToRad = Math.PI / 180;
        
        // 使用欧拉角设置旋转 - 以航空姿态标准顺序应用
        // 先清空当前旋转
        this.cube.rotation.set(0, 0, 0);
        
        // 按照 Z-Y-X 顺序应用旋转（与陀螺仪标准一致）
        // 注意角度符号：根据右手法则，正角度为逆时针旋转
        this.cube.rotateZ(z * degToRad); // 偏航 Yaw
        this.cube.rotateY(y * degToRad); // 俯仰 Pitch
        this.cube.rotateX(-x * degToRad); // 横滚 Roll (负号使其与传感器方向一致)
    }

    animate() {
        requestAnimationFrame(() => this.animate());
        this.renderer.render(this.scene, this.camera);
    }

    onWindowResize() {
        this.camera.aspect = this.container.clientWidth / this.container.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
    }
}

// 创建可视化实例
const cubeVisualizer = new CubeVisualizer('cube-container'); 