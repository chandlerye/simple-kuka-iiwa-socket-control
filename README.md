# KUKA机器人API控制库

一个用于控制KUKA iiwa 7 R800机器人的Python API库，通过Socket通信实现远程控制。支持笛卡尔坐标运动、关节角度运动、位姿查询和逆运动学计算等功能。

**适用环境：**
- 机器人型号：KUKA iiwa 7 R800
- 软件版本：FRI软件包 3.0
- 控制器：KUKA Sunrise控制器

**为什么选择本项目？**

本项目专门为 **FRI 3.0** 版本设计。如果您使用的是FRI 3.0，但无法使用 [lbr_fri_ros2_stack](https://github.com/lbr-stack/lbr_fri_ros2_stack) 项目（该项目要求FRI版本不大于2.7），那么本项目正是您需要的解决方案。

**与其他项目的区别：**
- ✅ **支持FRI 3.0**：本项目完全兼容FRI软件包3.0版本
- ✅ **轻量级**：纯Python实现，易于集成
- ✅ **简单易用**：提供简洁的Python API
- ⚠️ **注意**：如果您使用的是FRI 2.7或更早版本，建议使用 [lbr_fri_ros2_stack](https://github.com/lbr-stack/lbr_fri_ros2_stack) 项目

## 📋 目录

- [功能特性](#功能特性)
- [系统要求](#系统要求)
- [项目结构](#项目结构)
- [安装说明](#安装说明)
- [快速开始](#快速开始)
- [API文档](#api文档)
- [使用示例](#使用示例)
- [注意事项](#注意事项)
- [许可证](#许可证)

## ✨ 功能特性

- **笛卡尔坐标运动**：支持指定位置和姿态的末端执行器运动
- **关节角度运动**：支持7个关节的独立角度控制
- **位姿查询**：实时获取机器人末端执行器的位置和姿态
- **关节角度查询**：实时获取所有关节的当前角度
- **逆运动学计算**：基于ikpy库的逆运动学求解（可选功能）
- **连续控制模式**：支持不等待完成的连续运动控制
- **上下文管理**：支持Python上下文管理器，自动管理连接

## 🔧 系统要求

### Python环境
- Python 3.10+
- 操作系统：Windows / Linux / macOS

### KUKA机器人硬件
- **机器人型号**：KUKA iiwa 7 R800
- **控制器**：KUKA Sunrise控制器

### KUKA软件环境
- **FRI软件包**：**3.0版本**（Fast Robot Interface 3.0）⚠️ **必需**
- **开发环境**：KUKA Sunrise Workbench（用于编译和部署Java服务器程序）
- Java服务器程序（`LBRRobotAPIServer.java`）需要在控制器上运行

**重要提示：**
- 本项目**仅支持FRI 3.0版本**
- 如果您使用的是FRI 2.7或更早版本，请考虑使用 [lbr_fri_ros2_stack](https://github.com/lbr-stack/lbr_fri_ros2_stack) 项目
- lbr_fri_ros2_stack项目要求FRI版本不大于2.7，不支持FRI 3.0

### 网络要求
- KUKA控制器和Python客户端需要在同一网络
- 端口范围：30000-30010（KUKA iiwa限制）
- 默认端口：30009

## 📁 项目结构

```
.
├── kuka_robot_api.py          # Python API客户端库
├── demo.py                     # 使用示例和演示程序
├── java/
│   └── LBRRobotAPIServer.java # Java服务器端程序（需部署到KUKA控制器）
├── requirements.txt            # Python依赖包
└── README.md                  # 项目说明文档
```

## 📦 安装说明

### 1. 克隆或下载项目

```bash
git clone <your-repo-url>
cd open
```

### 2. 安装Python依赖

**基础功能（必需）：**
```bash
pip install -r requirements.txt
```

**完整功能（包括逆运动学）：**
```bash
pip install numpy ikpy scipy
```

### 3. 部署Java服务器程序到KUKA控制器

**前提条件：**
- KUKA iiwa 7 R800机器人
- KUKA Sunrise控制器
- FRI软件包3.0已安装
- KUKA Sunrise Workbench开发环境

**部署步骤：**

1. 打开KUKA Sunrise Workbench
2. 创建新的Java项目或导入现有项目
3. 将 `java/LBRRobotAPIServer.java` 文件复制到项目中
4. 确保项目包含必要的KUKA FRI 3.0依赖库
5. 编译项目（Build）
6. 部署到KUKA控制器（Deploy）
7. 在控制器上启动应用程序
8. 确认服务器程序正在运行，监听端口30009

**验证部署：**
- 检查控制器日志，确认看到 "Robot API Server Ready!" 消息
- 使用Python客户端测试连接

## 🚀 快速开始

### 基本连接示例

```python
from kuka_robot_api import KUKARobotAPI

# 使用上下文管理器（推荐）
with KUKARobotAPI(host="172.31.1.147", port=30009) as api:
    if api.connected:
        # 获取当前位姿
        pose = api.get_pose()
        print(f"当前位姿: {pose}")
        
        # 执行笛卡尔坐标运动
        api.move_cartesian(450.0, 0.0, 620.0, 180.0, 0.0, 180.0, velocity=0.25)
```

### 运行演示程序

```bash
python demo.py
```

演示程序提供以下功能：
1. 笛卡尔坐标运动
2. 关节角度运动
3. 获取当前位姿
4. 获取当前关节角度
5. 组合操作
6. 序列运动

## 📚 API文档

### KUKARobotAPI类

#### 初始化

```python
KUKARobotAPI(host="172.31.1.147", port=30009, timeout=10.0)
```

**参数：**
- `host` (str): KUKA控制器IP地址，默认 `172.31.1.147`
- `port` (int): Socket端口，默认 `30009`（必须在30000-30010范围内）
- `timeout` (float): 连接超时时间（秒），默认 `10.0`

#### 连接管理

```python
api.connect()      # 连接到KUKA控制器
api.disconnect()   # 断开连接
api.ping()         # 测试连接
```

#### 运动控制

##### 1. 笛卡尔坐标运动

```python
api.move_cartesian(x, y, z, alpha, beta, gamma, velocity=0.25, wait_for_completion=False)
```

**参数：**
- `x, y, z` (float): 位置坐标（单位：毫米）
- `alpha, beta, gamma` (float): 欧拉角（单位：度，ZYX顺序）
- `velocity` (float): 相对速度，范围0.0-1.0，默认0.25
- `wait_for_completion` (bool): 是否等待运动完成，默认False（连续控制模式）

**返回：**
- `bool`: 命令发送成功返回True，失败返回False

**示例：**
```python
# 移动到指定位置和姿态
api.move_cartesian(450.0, 0.0, 620.0, 180.0, 0.0, 180.0, velocity=0.3)
```

##### 2. 关节角度运动

```python
api.move_joint(joints, velocity=0.25)
```

**参数：**
- `joints` (List[float]): 7个关节角度值（单位：度）
- `velocity` (float): 相对速度，范围0.0-1.0，默认0.25

**返回：**
- `bool`: 运动成功返回True，失败返回False

**示例：**
```python
# 移动到指定关节角度
joints = [0.0, 30.0, 0.0, 0.0, 0.0, 60.0, 0.0]
api.move_joint(joints, velocity=0.25)
```

#### 状态查询

##### 3. 获取当前位姿

```python
pose = api.get_pose()
```

**返回：**
- `Tuple[float, float, float, float, float, float]` 或 `None`
  - 成功：`(x, y, z, alpha, beta, gamma)`
    - `x, y, z`: 位置（毫米）
    - `alpha, beta, gamma`: 欧拉角（度，ZYX顺序）
  - 失败：`None`

**示例：**
```python
pose = api.get_pose()
if pose:
    x, y, z, alpha, beta, gamma = pose
    print(f"位置: [{x:.3f}, {y:.3f}, {z:.3f}] mm")
    print(f"姿态: [{alpha:.3f}°, {beta:.3f}°, {gamma:.3f}°]")
```

##### 4. 获取当前关节角度

```python
joints = api.get_joint()
```

**返回：**
- `List[float]` 或 `None`
  - 成功：7个关节角度值（度）
  - 失败：`None`

**示例：**
```python
joints = api.get_joint()
if joints:
    print(f"关节角度: {joints}")
    for i, angle in enumerate(joints, 1):
        print(f"  关节 {i} (A{i}): {angle:.3f}°")
```

## 💡 使用示例

### 示例1：基本运动控制

```python
from kuka_robot_api import KUKARobotAPI

with KUKARobotAPI() as api:
    if not api.connected:
        print("连接失败")
        exit(1)
    
    # 获取初始位姿
    pose = api.get_pose()
    print(f"初始位姿: {pose}")
    
    # 执行笛卡尔坐标运动
    api.move_cartesian(450.0, 0.0, 620.0, 180.0, 0.0, 180.0, velocity=0.25)
    
    # 获取运动后位姿
    pose = api.get_pose()
    print(f"运动后位姿: {pose}")
```

### 示例2：连续运动控制

```python
from kuka_robot_api import KUKARobotAPI

with KUKARobotAPI() as api:
    if not api.connected:
        exit(1)
    
    # 连续发送多个运动命令（不等待完成）
    api.move_cartesian(450.0, 0.0, 620.0, 180.0, 0.0, 180.0, 
                      velocity=0.25, wait_for_completion=False)
    api.move_cartesian(470.0, 25.0, 600.0, 180.0, 0.0, 175.0, 
                      velocity=0.25, wait_for_completion=False)
    api.move_cartesian(430.0, -20.0, 610.0, 175.0, -3.0, 185.0, 
                      velocity=0.25, wait_for_completion=False)
```

### 示例3：关节角度控制

```python
from kuka_robot_api import KUKARobotAPI

with KUKARobotAPI() as api:
    if not api.connected:
        exit(1)
    
    # 移动到初始位置
    joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    api.move_joint(joints, velocity=0.2)
    
    # 移动到指定关节角度
    joints = [0.0, 30.0, 0.0, 0.0, 0.0, 60.0, 0.0]
    api.move_joint(joints, velocity=0.25)
```

### 示例4：序列运动

```python
from kuka_robot_api import KUKARobotAPI

with KUKARobotAPI() as api:
    if not api.connected:
        exit(1)
    
    # 定义一系列目标点
    targets = [
        (450.0, 0.0, 620.0, 180.0, 0.0, 180.0, 0.25),
        (470.0, 25.0, 600.0, 180.0, 5.0, 175.0, 0.25),
        (430.0, -20.0, 610.0, 175.0, -3.0, 185.0, 0.25),
    ]
    
    for x, y, z, alpha, beta, gamma, vel in targets:
        api.move_cartesian(x, y, z, alpha, beta, gamma, velocity=vel)
```

## ⚠️ 注意事项

### 安全警告
- **操作机器人前请确保安全区域无人**
- **首次使用建议低速测试（velocity < 0.2）**
- **确保急停按钮可随时使用**
- **检查工作空间内无障碍物**

### 技术限制
1. **硬件要求**：本项目针对KUKA iiwa 7 R800开发，其他型号可能需要调整
2. **软件版本**：需要FRI软件包3.0，其他版本可能存在兼容性问题
3. **端口限制**：KUKA iiwa只能使用端口范围30000-30010
4. **网络要求**：确保防火墙允许相应端口通信
5. **运动速度**：建议velocity值在0.1-0.5之间，过高可能导致运动不稳定
6. **坐标系统**：位姿基于机器人基坐标系
7. **欧拉角顺序**：使用ZYX顺序（Roll-Pitch-Yaw）

### 常见问题

**Q: 连接失败怎么办？**
- 检查KUKA控制器IP地址是否正确
- 确认Java服务器程序正在运行
- 检查网络连接和防火墙设置
- 确认端口在30000-30010范围内
- 验证FRI软件包3.0已正确安装
- 确认机器人型号为KUKA iiwa 7 R800

**Q: 运动命令执行失败？**
- 检查目标位置是否在工作空间内
- 确认关节角度在限位范围内
- 检查是否有碰撞风险
- 降低运动速度重试

**Q: 如何获取逆运动学解？**
- 需要安装numpy、ikpy、scipy库
- 使用`create_iiwa_chain()`创建运动链
- 使用`inverse_kinematics_to_pose()`计算逆解

**Q: 我应该使用本项目还是lbr_fri_ros2_stack？**
- **使用本项目**：如果您使用的是 **FRI 3.0** 版本
- **使用lbr_fri_ros2_stack**：如果您使用的是 **FRI 2.7或更早版本**，且需要ROS 2集成
- **注意**：[lbr_fri_ros2_stack](https://github.com/lbr-stack/lbr_fri_ros2_stack) 项目要求FRI版本不大于2.7，不支持FRI 3.0

## 📄 许可证

本项目采用 MIT 许可证。详见 LICENSE 文件（如有）。

## 🤝 贡献

欢迎提交Issue和Pull Request！

## 📧 联系方式

如有问题或建议，请通过GitHub Issues联系。

---

**免责声明**：使用本库控制机器人存在安全风险，使用者需自行承担所有责任。请确保在安全的环境下操作，并遵守所有相关安全规范。

