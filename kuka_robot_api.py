#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
KUKA机器人API封装
提供笛卡尔坐标运动、关节角度运动和获取位姿的API
包含逆运动学（IK）计算功能
"""

import socket
from typing import Tuple, Optional, List

# IK相关依赖（可选，仅在需要IK功能时使用）
try:
    import numpy as np
    from ikpy.chain import Chain
    from ikpy.link import OriginLink, DHLink
    from scipy.optimize import minimize
    IK_AVAILABLE = True
except ImportError:
    IK_AVAILABLE = False
    np = None


class KUKARobotAPI:
    """KUKA机器人API类"""
    
    def __init__(self, host: str = "172.31.1.147", port: int = 30009, timeout: float = 10.0):
        """
        初始化KUKA机器人API客户端
        
        参数:
            host: KUKA控制器IP地址（默认172.31.1.147）
            port: Socket端口（默认30009，必须在30000-30010范围内）
            timeout: 连接超时时间（秒）
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None
        self.connected = False
    
    def connect(self) -> bool:
        """
        连接到KUKA控制器
        
        返回:
            True: 连接成功
            False: 连接失败
        """
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout)
            self.sock.connect((self.host, self.port))
            self.connected = True
            
            # 接收并清空欢迎消息（可能有多行）
            self._receive_until_ready()
            print(f"✓ 已连接到KUKA控制器: {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"✗ 连接失败: {e}")
            self.connected = False
            return False
    
    def _receive_until_ready(self):
        """接收并清空欢迎消息，直到准备好接收命令响应"""
        if not self.sock:
            return
        try:
            # 设置较短的超时来接收欢迎消息
            self.sock.settimeout(1.0)
            while True:
                try:
                    data = self.sock.recv(1024).decode('utf-8', errors='ignore')
                    if not data:
                        break
                    # 如果收到空数据或已经接收完欢迎消息，退出
                    if "Ready!" in data or "EXIT" in data:
                        break
                except socket.timeout:
                    # 超时说明欢迎消息已接收完
                    break
            # 恢复原始超时
            self.sock.settimeout(self.timeout)
        except Exception:
            # 忽略接收欢迎消息时的错误
            pass
    
    def disconnect(self):
        """断开连接"""
        if self.sock:
            try:
                if self.connected:
                    self._send_command("EXIT")
                self.sock.close()
                self.connected = False
                print("✓ 已断开连接")
            except Exception as e:
                print(f"✗ 断开连接时出错: {e}")
    
    def _send_command(self, command: str, wait_for_complete: bool = True) -> Tuple[bool, str]:
        """
        发送命令并接收响应（内部方法）
        
        参数:
            command: 要发送的命令字符串（不需要换行符）
            wait_for_complete: 是否等待命令完成（运动命令需要等待，GET_POSE不需要）
        
        返回:
            (成功标志, 响应消息)
        """
        if not self.connected or not self.sock:
            return False, "未连接到服务器"
        
        try:
            # 对于GET_POSE等快速命令，使用较短的超时时间
            original_timeout = self.sock.gettimeout()
            if not wait_for_complete:
                # GET_POSE等快速命令，设置较短的超时（1秒）
                self.sock.settimeout(1.0)
            
            try:
                # 发送命令（添加换行符）
                self.sock.sendall((command + "\n").encode('utf-8'))
                
                # 接收响应
                # 对于运动命令，需要等待完整响应（可能包含多行）
                # 对于GET_POSE，可以立即返回
                response = self._receive_response(wait_for_complete)
                
                # 判断成功标志
                success = response.startswith("OK") or response == "pong"
                
                return success, response
            finally:
                # 恢复原始超时设置
                if not wait_for_complete:
                    self.sock.settimeout(original_timeout)
        except socket.timeout:
            return False, "接收响应超时"
        except Exception as e:
            return False, f"发送命令时出错: {e}"
    
    def _receive_response(self, wait_for_complete: bool = True) -> str:
        """
        接收服务器响应
        
        参数:
            wait_for_complete: 是否等待完整响应
        
        返回:
            响应字符串
        """
        if not self.sock:
            return ""
        
        response_parts = []
        try:
            if wait_for_complete:
                # 对于需要等待完成的运动命令，等待完整响应
                # Java端会先发送"Command received"，然后发送"Motion completed"
                # 我们需要等待"Motion completed"响应
                max_attempts = 100  # 最多尝试100次，避免无限循环
                attempt = 0
                received_completed = False
                
                while attempt < max_attempts and not received_completed:
                    attempt += 1
                    data = self.sock.recv(1024).decode('utf-8', errors='ignore')
                    if not data:
                        # 如果没有数据，等待一下再试
                        import time
                        time.sleep(0.01)
                        continue
                    
                    response_parts.append(data)
                    
                    # 检查是否收到完成响应（"Motion completed"或"Final"）
                    if "Motion completed" in data or "Final" in data:
                        received_completed = True
                        break
                    
                    # 如果响应包含换行符，检查每一行
                    if '\n' in data:
                        lines = data.split('\n')
                        for line in lines:
                            if "Motion completed" in line or "Final" in line:
                                received_completed = True
                                break
                        if received_completed:
                            break
            else:
                # 对于不等待完成的命令（连续控制模式），只接收"Command received"响应
                # 设置较短的超时，快速接收响应后立即返回
                max_attempts = 5  # 只尝试几次，快速返回
                attempt = 0
                received_response = False
                
                while attempt < max_attempts and not received_response:
                    attempt += 1
                    try:
                        data = self.sock.recv(1024).decode('utf-8', errors='ignore')
                        if data:
                            response_parts.append(data)
                            # 收到"Command received"即可返回
                            if "Command received" in data or "OK:" in data:
                                received_response = True
                                break
                    except socket.timeout:
                        # 超时说明可能没有响应，直接返回
                        break
            
            response = ''.join(response_parts).strip()
            
            # 过滤掉欢迎消息的残留
            if "Robot API Server Ready!" in response:
                # 提取最后一行作为实际响应
                lines = response.split('\n')
                for line in reversed(lines):
                    if line.strip() and not line.startswith("Supported") and not line.startswith("  "):
                        response = line.strip()
                        break
            
            # 如果是多行响应，提取正确的行
            if '\n' in response:
                lines = response.split('\n')
                # 对于运动命令，优先选择包含"Motion completed"或"Final"的行
                if wait_for_complete:
                    for line in reversed(lines):
                        if "Motion completed" in line or "Final" in line:
                            response = line.strip()
                            break
                    else:
                        # 如果没有找到，使用最后一行
                        response = lines[-1].strip()
                else:
                    # 对于GET_POSE等快速命令，选择包含"OK:"或"ERROR:"的行
                    for line in reversed(lines):
                        if line.strip().startswith("OK:") or line.strip().startswith("ERROR:"):
                            response = line.strip()
                            break
                    else:
                        # 如果没有找到，使用最后一行
                        response = lines[-1].strip()
            
            return response
        except socket.timeout:
            return ''.join(response_parts).strip() if response_parts else ""
        except Exception as e:
            return f"接收响应时出错: {e}"
    
    def move_cartesian(self, x: float, y: float, z: float, 
                      alpha: float, beta: float, gamma: float,
                      velocity: float = 0.25, wait_for_completion: bool = False) -> bool:
        """
        API 1: 笛卡尔坐标运动
        
        参数:
            x, y, z: 位置（单位：毫米）
            alpha, beta, gamma: 欧拉角（单位：度，ZYX顺序）
            velocity: 相对速度（0.0-1.0，默认0.25）
            wait_for_completion: 是否等待运动完成（默认False，用于连续控制）
        
        返回:
            True: 命令发送成功
            False: 命令发送失败
        """
        if not self.connected:
            print("✗ 错误: 未连接到服务器")
            return False
        
        # 限制速度范围
        velocity = max(0.0, min(1.0, velocity))
        
        # 构建命令
        cmd = f"MOVE_CARTESIAN {x:.3f} {y:.3f} {z:.3f} {alpha:.3f} {beta:.3f} {gamma:.3f} {velocity:.3f}"
        
        # 发送命令（不等待完成，实现连续控制）
        success, response = self._send_command(cmd, wait_for_complete=wait_for_completion)
        
        # 只在失败时打印，成功时不打印（避免刷屏）
        if not success:
            print(f"✗ 笛卡尔坐标运动失败: {response}")
        
        return success
    
    def move_joint(self, joints: List[float], velocity: float = 0.25) -> bool:
        """
        API 2: 关节角度运动
        
        参数:
            joints: 7个关节角度值（单位：度）
            velocity: 相对速度（0.0-1.0，默认0.25）
        
        返回:
            True: 运动成功
            False: 运动失败
        """
        if not self.connected:
            print("✗ 错误: 未连接到服务器")
            return False
        
        if len(joints) != 7:
            print("✗ 错误: 关节角度必须是7个值")
            return False
        
        # 限制速度范围
        velocity = max(0.0, min(1.0, velocity))
        
        # 构建命令
        cmd = f"MOVE_JOINT {' '.join([f'{j:.3f}' for j in joints])} {velocity:.3f}"
        
        # 发送命令（运动命令需要等待完成）
        success, response = self._send_command(cmd, wait_for_complete=True)
        
        if success:
            print(f"✓ 关节角度运动成功: {joints}, 速度={velocity:.2f}")
        else:
            print(f"✗ 关节角度运动失败: {response}")
        
        return success
    
    def get_joint(self) -> Optional[List[float]]:
        """
        API 4: 获取当前关节角度
        
        返回:
            7个关节角度值（单位：度），如果失败返回None
        """
        if not self.connected:
            print("✗ 错误: 未连接到服务器")
            return None
        
        cmd = "GET_JOINT"
        success, response = self._send_command(cmd, wait_for_complete=False)
        
        if not success:
            print(f"✗ 获取关节角度失败: {response}")
            return None
        
        # 解析响应: "OK: j1 j2 j3 j4 j5 j6 j7"
        try:
            if response.startswith("OK:"):
                parts = response[3:].strip().split()
                if len(parts) >= 7:
                    joints = [float(parts[i]) for i in range(7)]
                    return joints
                else:
                    print(f"✗ 响应格式错误: {response}")
                    return None
            else:
                print(f"✗ 获取关节角度失败: {response}")
                return None
        except (ValueError, IndexError) as e:
            print(f"✗ 解析关节角度失败: {e}, 响应: {response}")
            return None
    
    def get_pose(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        """
        API 3: 获取当前末端在机械臂坐标系下的位姿
        
        返回:
            如果成功，返回 (x, y, z, alpha, beta, gamma)
            - x, y, z: 位置（单位：毫米）
            - alpha, beta, gamma: 欧拉角（单位：度，ZYX顺序）
            如果失败，返回 None
        """
        if not self.connected:
            print("✗ 错误: 未连接到服务器")
            return None
        
        # 发送命令（GET_POSE需要等待完整响应，但不需要等待运动完成）
        success, response = self._send_command("GET_POSE", wait_for_complete=False)
        
        if not success:
            print(f"✗ 获取位姿失败: {response}")
            return None
        
        try:
            # 解析响应
            # 格式: "OK: x y z alpha beta gamma"
            # 清理响应，移除可能的残留数据
            response = response.strip()
            
            # 如果响应包含多行，提取包含"OK:"的行
            if '\n' in response:
                lines = response.split('\n')
                for line in lines:
                    if line.strip().startswith("OK:"):
                        response = line.strip()
                        break
                else:
                    # 如果没找到OK:，使用最后一行
                    response = lines[-1].strip()
            
            # 检查响应是否以OK:开头
            if not response.startswith("OK:"):
                print(f"✗ 错误: 响应格式不正确: {response}")
                return None
            
            parts = response.split(":", 1)  # 只分割一次
            if len(parts) < 2:
                print(f"✗ 错误: 响应格式不正确: {response}")
                return None
            
            values = parts[1].strip().split()
            if len(values) != 6:
                print(f"✗ 错误: 响应数据不完整，期望6个值，实际{len(values)}个: {values}")
                print(f"  完整响应: {response}")
                return None
            
            x = float(values[0])
            y = float(values[1])
            z = float(values[2])
            alpha = float(values[3])
            beta = float(values[4])
            gamma = float(values[5])
            
            print(f"✓ 当前位姿: [{x:.3f}, {y:.3f}, {z:.3f}] mm, [{alpha:.3f}°, {beta:.3f}°, {gamma:.3f}°]")
            return (x, y, z, alpha, beta, gamma)
            
        except (ValueError, IndexError) as e:
            print(f"✗ 解析位姿数据失败: {e}")
            return None
    
    def ping(self) -> bool:
        """
        测试连接
        
        返回:
            True: 连接正常
            False: 连接异常
        """
        if not self.connected:
            return False
        
        success, response = self._send_command("PING")
        return success
    
    def __enter__(self):
        """上下文管理器入口"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.disconnect()


# ============================================================================
# 逆运动学（IK）相关函数
# ============================================================================

def _extract_dh_from_pdf():
    """从 pdf_content.txt 提取 DH 参数"""
    if not IK_AVAILABLE:
        raise ImportError("IK功能需要安装: numpy, ikpy, scipy")
    dh_parameters = [
        [0.0, -np.pi/2, 0.34, 0.0],     # Joint 1: α=-π/2, d_bs=340mm
        [0.0, np.pi/2, 0.0, 0.0],       # Joint 2: α=π/2, d=0
        [0.0, np.pi/2, 0.4, 0.0],       # Joint 3: α=π/2, d_se=400mm
        [0.0, -np.pi/2, 0.0, 0.0],      # Joint 4: α=-π/2, d=0
        [0.0, -np.pi/2, 0.4, 0.0],      # Joint 5: α=-π/2, d_ew=400mm
        [0.0, np.pi/2, 0.0, 0.0],       # Joint 6: α=π/2, d=0
        [0.0, 0.0, 0.126, 0.0],         # Joint 7: α=0, d_wf=126mm
    ]
    return dh_parameters


def create_iiwa_chain():
    """
    创建 KUKA IIWA 7 运动链
    
    根据 KUKA IIWA 7 R800 技术文档，关节限位为：
    - A1: ±170°
    - A2: ±120°
    - A3: ±170°
    - A4: ±120°
    - A5: ±170°
    - A6: ±120°
    - A7: ±175°
    
    返回:
        chain: ikpy Chain对象
    """
    if not IK_AVAILABLE:
        raise ImportError("IK功能需要安装: numpy, ikpy, scipy")
    
    joint_limits_deg = [
        (-170, 170),  # Joint 1
        (-120, 120),  # Joint 2
        (-170, 170),  # Joint 3
        (-120, 120),  # Joint 4
        (-170, 170),  # Joint 5
        (-120, 120),  # Joint 6
        (-175, 175),  # Joint 7
    ]
    
    dh_params = _extract_dh_from_pdf()
    links = [OriginLink()]
    
    for i, (a, alpha, d, theta) in enumerate(dh_params):
        min_limit, max_limit = joint_limits_deg[i]
        bounds = (np.radians(min_limit), np.radians(max_limit))
        
        links.append(DHLink(
            name=f"iiwa_joint_{i+1}",
            a=a,
            alpha=alpha,
            d=d,
            theta=theta,
            bounds=bounds
        ))
    
    chain = Chain(links=links, name="KUKA_IIWA_7")
    return chain


def _euler_zyx_to_rotation_matrix(roll, pitch, yaw):
    """将欧拉角（ZYX顺序，旋转轴）转换为旋转矩阵"""
    if not IK_AVAILABLE:
        raise ImportError("IK功能需要安装: numpy")
    
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    R = Rz @ Ry @ Rx
    return R


def forward_kinematics(chain, joint_angles):
    """
    计算正运动学
    
    参数:
        chain: ikpy 运动链对象
        joint_angles: 关节角度列表（弧度）
    
    返回:
        position: 末端执行器位置 [x, y, z]
        rotation: 旋转矩阵 3x3
    """
    if not IK_AVAILABLE:
        raise ImportError("IK功能需要安装: numpy, ikpy")
    
    fk_result = chain.forward_kinematics(joint_angles)
    position = fk_result[:3, 3]
    rotation = fk_result[:3, :3]
    return position, rotation


def _normalize_joint_angles(joint_angles, chain):
    """规范化关节角度到限位范围内"""
    if not IK_AVAILABLE:
        raise ImportError("IK功能需要安装: numpy")
    
    normalized = np.array(joint_angles).copy()
    
    for i, link in enumerate(chain.links):
        if hasattr(link, 'bounds') and link.bounds is not None:
            min_bound, max_bound = link.bounds
            normalized[i] = np.clip(normalized[i], min_bound, max_bound)
    
    return normalized


def check_joint_limits(joint_angles, chain):
    """
    检查关节角度是否在限位内
    
    参数:
        joint_angles: 关节角度数组（弧度）
        chain: 运动链对象
    
    返回:
        is_valid: 是否所有角度都在限位内
        violations: 超出限位的关节信息列表
    """
    if not IK_AVAILABLE:
        raise ImportError("IK功能需要安装: numpy")
    
    joint_limits_deg = [
        (-170, 170),  # Joint 1
        (-120, 120),  # Joint 2
        (-170, 170),  # Joint 3
        (-120, 120),  # Joint 4
        (-170, 170),  # Joint 5
        (-120, 120),  # Joint 6
        (-175, 175),  # Joint 7
    ]
    
    violations = []
    
    for i in range(1, len(chain.links)):
        joint_num = i - 1
        if 0 <= joint_num < len(joint_limits_deg):
            min_limit_deg, max_limit_deg = joint_limits_deg[joint_num]
            min_bound = np.radians(min_limit_deg)
            max_bound = np.radians(max_limit_deg)
            angle = joint_angles[i]
            
            if angle < min_bound or angle > max_bound:
                violations.append({
                    'joint': i,
                    'angle_rad': angle,
                    'angle_deg': np.degrees(angle),
                    'min_limit_deg': min_limit_deg,
                    'max_limit_deg': max_limit_deg
                })
    
    return len(violations) == 0, violations


def _compute_pose_error(chain, joint_angles, target_position, target_rotation):
    """计算位姿误差"""
    if not IK_AVAILABLE:
        raise ImportError("IK功能需要安装: numpy")
    
    actual_position, actual_rotation = forward_kinematics(chain, joint_angles)
    position_error = np.linalg.norm(actual_position - target_position)
    rotation_error_matrix = actual_rotation - target_rotation
    orientation_error = np.linalg.norm(rotation_error_matrix, 'fro')
    return position_error, orientation_error


def inverse_kinematics_to_pose(chain, target_position, target_orientation_deg, 
                                initial_position=None,
                                tolerance=1e-6, max_iterations=200,
                                verbose=True):
    """
    计算逆运动学，使末端执行器以指定的位姿到达指定点
    
    参数:
        chain: ikpy 运动链对象
        target_position: 目标位置 [x, y, z] (米)
        target_orientation_deg: 目标方向 [roll, pitch, yaw] (度)
        initial_position: 初始关节位置（弧度），如果为None则使用ikpy默认值
        tolerance: 误差容差
        max_iterations: 最大迭代次数
        verbose: 是否打印详细信息
    
    返回:
        joint_angles: 关节角度列表（弧度），如果失败返回None
        success: 是否成功
        error_info: 误差信息字典
    """
    if not IK_AVAILABLE:
        raise ImportError("IK功能需要安装: numpy, ikpy, scipy")
    
    target_position = np.array(target_position)
    target_orientation_rad = np.radians(target_orientation_deg)
    roll, pitch, yaw = target_orientation_rad
    target_rotation = _euler_zyx_to_rotation_matrix(roll, pitch, yaw)
    
    if verbose:
        print(f"\n目标位置: {target_position} (米)")
        print(f"目标方向 (度): roll={target_orientation_deg[0]:.2f}°, "
              f"pitch={target_orientation_deg[1]:.2f}°, yaw={target_orientation_deg[2]:.2f}°")
        if initial_position is not None:
            initial_deg = [np.degrees(angle) for angle in initial_position[1:]]
            print(f"初始关节位置 (度): {initial_deg}")
    
    try:
        if verbose:
            print("\n步骤1: 使用 ikpy 计算初始解...")
        
        joint_limits_deg = [
            (-170, 170), (-120, 120), (-170, 170), (-120, 120),
            (-170, 170), (-120, 120), (-175, 175),
        ]
        
        if initial_position is not None:
            if len(initial_position) == len(chain.links):
                initial_pos = initial_position.copy()
            else:
                initial_pos = np.zeros(len(chain.links))
                active_joint_indices = []
                for i, link in enumerate(chain.links):
                    if hasattr(link, 'bounds') and link.bounds is not None:
                        active_joint_indices.append(i)
                for idx, joint_idx in enumerate(active_joint_indices):
                    if idx < len(initial_position):
                        initial_pos[joint_idx] = initial_position[idx]
        else:
            initial_pos = None
        
        try:
            if verbose:
                print("尝试使用旋转矩阵作为 target_orientation...")
            if initial_pos is not None:
                sol = chain.inverse_kinematics(
                    target_position=target_position,
                    target_orientation=target_rotation,
                    orientation_mode="all",
                    initial_position=initial_pos
                )
            else:
                sol = chain.inverse_kinematics(
                    target_position=target_position,
                    target_orientation=target_rotation,
                    orientation_mode="all"
                )
            
            is_valid, violations = check_joint_limits(sol, chain)
            if verbose:
                if is_valid:
                    print(f"✓ 找到在限位内的解！")
                else:
                    print(f"  解超出限位（{len(violations)} 个关节）")
            initial_solution = sol
        except Exception as e1:
            if verbose:
                print(f"使用旋转矩阵失败: {e1}")
            try:
                if verbose:
                    print("尝试使用欧拉角...")
                if initial_pos is not None:
                    sol = chain.inverse_kinematics(
                        target_position=target_position,
                        target_orientation=target_orientation_rad,
                        orientation_mode="all",
                        initial_position=initial_pos
                    )
                else:
                    sol = chain.inverse_kinematics(
                        target_position=target_position,
                        target_orientation=target_orientation_rad,
                        orientation_mode="all"
                    )
                
                is_valid, violations = check_joint_limits(sol, chain)
                if verbose:
                    if is_valid:
                        print(f"✓ 找到在限位内的解！")
                    else:
                        print(f"  解超出限位（{len(violations)} 个关节）")
                initial_solution = sol
            except Exception as e2:
                if verbose:
                    print(f"使用欧拉角也失败: {e2}")
                raise
        
        pos_err, ori_err = _compute_pose_error(chain, initial_solution, target_position, target_rotation)
        if verbose:
            print(f"初始解误差: 位置={pos_err*1000:.6f} mm, 方向={ori_err:.6f}")
        
        is_valid, violations = check_joint_limits(initial_solution, chain)
        if not is_valid and verbose:
            print(f"⚠ 初始解超出限位（{len(violations)} 个关节），需要进行优化...")
            for v in violations:
                print(f"  关节 {v['joint']}: {v['angle_deg']:.2f}° "
                      f"(限位: {v['min_limit_deg']:.2f}° ~ {v['max_limit_deg']:.2f}°)")
        
        if pos_err < tolerance and ori_err < tolerance and is_valid:
            if verbose:
                print("初始解已满足精度要求且在限位内！")
            return initial_solution, True, {'position_error': pos_err, 'orientation_error': ori_err}
        
        if verbose:
            print("初始解精度不满足，进行优化...")
        
        # 优化步骤
        active_joint_indices = []
        for i, link in enumerate(chain.links):
            if hasattr(link, 'bounds') and link.bounds is not None:
                active_joint_indices.append(i)
        
        initial_active_angles = initial_solution[active_joint_indices]
        
        for idx, joint_idx in enumerate(active_joint_indices):
            joint_num = joint_idx - 1
            if 0 <= joint_num < len(joint_limits_deg):
                min_limit, max_limit = joint_limits_deg[joint_num]
                min_bound = np.radians(min_limit)
                max_bound = np.radians(max_limit)
                initial_active_angles[idx] = np.clip(initial_active_angles[idx], min_bound, max_bound)
        
        def objective(active_angles):
            full_angles = np.zeros(len(chain.links))
            for idx, joint_idx in enumerate(active_joint_indices):
                full_angles[joint_idx] = active_angles[idx]
            actual_pos, actual_rot = forward_kinematics(chain, full_angles)
            pos_error = np.linalg.norm(actual_pos - target_position)
            rot_error_matrix = actual_rot - target_rotation
            ori_error = np.linalg.norm(rot_error_matrix, 'fro')
            return 1000.0 * pos_error + 100.0 * ori_error
        
        bounds = []
        for joint_idx in active_joint_indices:
            joint_num = joint_idx - 1
            if 0 <= joint_num < len(joint_limits_deg):
                min_limit, max_limit = joint_limits_deg[joint_num]
                bounds.append((np.radians(min_limit), np.radians(max_limit)))
            else:
                bounds.append((-np.pi, np.pi))
        
        result = minimize(
            objective,
            initial_active_angles,
            method='L-BFGS-B',
            bounds=bounds,
            options={
                'maxiter': max_iterations * 2,
                'ftol': tolerance * 10,
                'gtol': tolerance * 10,
                'maxls': 50,
            }
        )
        
        optimized_angles = np.zeros(len(chain.links))
        for idx, joint_idx in enumerate(active_joint_indices):
            optimized_angles[joint_idx] = result.x[idx]
        
        optimized_angles = _normalize_joint_angles(optimized_angles, chain)
        is_valid, violations = check_joint_limits(optimized_angles, chain)
        
        if not is_valid and verbose:
            print(f"\n⚠ 警告: 以下关节超出限位:")
            for v in violations:
                print(f"  关节 {v['joint']}: {v['angle_deg']:.2f}° "
                      f"(限位: {v['min_limit_deg']:.2f}° ~ {v['max_limit_deg']:.2f}°)")
        
        pos_err, ori_err = _compute_pose_error(chain, optimized_angles, target_position, target_rotation)
        
        if verbose:
            print(f"优化完成: 位置误差={pos_err*1000:.6f} mm, 方向误差={ori_err:.6f}")
            print(f"优化迭代次数: {result.nit}")
            if result.success:
                print("✓ 优化算法收敛成功")
            else:
                print(f"⚠ 优化算法状态: {result.message}")
        
        if is_valid and pos_err < tolerance * 10 and ori_err < tolerance * 10:
            if verbose:
                print("✓ 优化成功，解在限位内且误差可接受！")
            return optimized_angles, True, {'position_error': pos_err, 'orientation_error': ori_err}
        elif is_valid:
            if verbose:
                print("⚠ 解在限位内，但误差略大（可能目标位姿难以精确达到）")
            return optimized_angles, True, {'position_error': pos_err, 'orientation_error': ori_err}
        else:
            if verbose:
                print("⚠ 警告: 优化后仍有关节超出限位（目标位姿可能无法在限位内实现）")
            return optimized_angles, False, {'position_error': pos_err, 'orientation_error': ori_err}
            
    except Exception as e:
        if verbose:
            print(f"✗ 逆运动学计算失败: {e}")
            import traceback
            traceback.print_exc()
        return None, False, None


if __name__ == "__main__":
    # 测试代码
    api = KUKARobotAPI()
    if api.connect():
        # 测试获取位姿
        pose = api.get_pose()
        if pose:
            print(f"当前位姿: {pose}")
        # 断开连接
        api.disconnect()

