#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
KUKA机器人API使用示例
演示如何使用封装的API控制机器人
"""

import time
from kuka_robot_api import KUKARobotAPI


def demo_cartesian_motion():
    """示例1: 笛卡尔坐标运动"""
    print("=" * 60)
    print("示例1: 笛卡尔坐标运动")
    print("=" * 60)
    
    with KUKARobotAPI() as api:
        if not api.connected:
            return
        
        # 移动到指定位置和姿态
        # 位置: [450, 0, 620] mm
        # 姿态: Roll=180°, Pitch=0°, Yaw=180°
        # 速度: 0.3 (30%)
        # 注意：move_cartesian会等待运动完成，不需要额外sleep
        # api.move_cartesian(450.0, 0.0, 620.0, 180.0, 0.0, 180.0, velocity=0.3)
        
        # 移动到另一个位置（连续运动，无需等待）
        api.move_cartesian(-470.0, 25.0, 600.0, 180.0, 0.0, 175.0, velocity=0.25)
        api.move_cartesian(470.0, 100.0, 600.0, 180.0, 0.0, 175.0, velocity=0.25)
        api.move_cartesian(470.0, -300.0, 600.0, 180.0, 0.0, 175.0, velocity=0.25)
def demo_joint_motion():
    """示例2: 关节角度运动"""
    print("\n" + "=" * 60)
    print("示例2: 关节角度运动")
    print("=" * 60)
    
    with KUKARobotAPI() as api:
        if not api.connected:
            return
        
        # 移动到初始位置（所有关节为0度）
        # 注意：move_joint会等待运动完成，不需要额外sleep
        joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        api.move_joint(joints, velocity=0.2)
        
        # 移动到指定关节角度（连续运动）
        joints = [0.0, 30.0, 0.0, 0.0, 0.0, 60.0, 0.0]
        api.move_joint(joints, velocity=0.25)
        
        # 移动到另一个关节角度（连续运动）
        joints = [0.0, 30.0, 60.0, 0.0, 0.0, 60.0, 0.0]
        api.move_joint(joints, velocity=0.3)


def demo_get_pose():
    """示例3: 获取当前位姿"""
    print("\n" + "=" * 60)
    print("示例3: 获取当前位姿")
    print("=" * 60)
    
    with KUKARobotAPI() as api:
        if not api.connected:
            return
        
        # 获取当前位姿
        pose = api.get_pose()
        if pose:
            x, y, z, alpha, beta, gamma = pose
            print(f"\n当前末端位姿:")
            print(f"  位置: X={x:.3f} mm, Y={y:.3f} mm, Z={z:.3f} mm")
            print(f"  姿态: Roll={alpha:.3f}°, Pitch={beta:.3f}°, Yaw={gamma:.3f}°")


def demo_get_joint():
    """示例4: 获取当前关节角度"""
    print("\n" + "=" * 60)
    print("示例4: 获取当前关节角度")
    print("=" * 60)
    
    with KUKARobotAPI() as api:
        if not api.connected:
            return
        
        # 获取当前关节角度
        joints = api.get_joint()
        if joints:
            print(f"\n当前关节角度 (度):")
            for i, angle in enumerate(joints, 1):
                print(f"  关节 {i} (A{i}): {angle:.3f}°")
            
            # 显示关节角度数组格式（方便复制使用）
            print(f"\n关节角度数组格式:")
            print(f"  joints = {joints}")
            
            # 显示可用于move_joint的格式
            print(f"\n可用于 move_joint() 的格式:")
            print(f"  api.move_joint({joints}, velocity=0.25)")


def demo_combined_operations():
    """示例5: 组合操作"""
    print("\n" + "=" * 60)
    print("示例4: 组合操作（运动 + 获取位姿）")
    print("=" * 60)
    
    with KUKARobotAPI() as api:
        if not api.connected:
            return
        
        # 1. 获取初始位姿
        print("\n步骤1: 获取初始位姿")
        pose1 = api.get_pose()
        if pose1:
            print(f"  初始位姿: {pose1}")
        
        # 2. 执行笛卡尔坐标运动
        print("\n步骤2: 执行笛卡尔坐标运动")
        api.move_cartesian(450.0, 0.0, 620.0, 180.0, 0.0, 180.0, velocity=0.25)
        
        # 3. 获取运动后的位姿
        print("\n步骤3: 获取运动后的位姿")
        pose2 = api.get_pose()
        if pose2:
            print(f"  运动后位姿: {pose2}")
        
        # 4. 执行关节角度运动
        print("\n步骤4: 执行关节角度运动")
        joints = [0.0, 30.0, 0.0, 0.0, 0.0, 60.0, 0.0]
        api.move_joint(joints, velocity=0.25)
        
        # 5. 获取最终位姿
        print("\n步骤5: 获取最终位姿")
        pose3 = api.get_pose()
        if pose3:
            print(f"  最终位姿: {pose3}")


def demo_sequence_motion():
    """示例6: 序列运动"""
    print("\n" + "=" * 60)
    print("示例5: 序列运动")
    print("=" * 60)
    
    with KUKARobotAPI() as api:
        if not api.connected:
            return
        
        # 定义一系列笛卡尔坐标目标
        cartesian_targets = [
            (450.0, 0.0, 620.0, 180.0, 0.0, 180.0, 0.25),
            (470.0, 25.0, 600.0, 180.0, 5.0, 175.0, 0.25),
            (430.0, -20.0, 610.0, 175.0, -3.0, 185.0, 0.25),
        ]
        
        print("\n执行笛卡尔坐标序列:")
        for i, (x, y, z, alpha, beta, gamma, vel) in enumerate(cartesian_targets, 1):
            print(f"\n目标 {i}/{len(cartesian_targets)}")
            api.move_cartesian(x, y, z, alpha, beta, gamma, velocity=vel)
        
        # 定义一系列关节角度目标
        joint_targets = [
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.2),
            ([0.0, 30.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.25),
            ([0.0, 30.0, 60.0, 0.0, 0.0, 0.0, 0.0], 0.25),
            ([0.0, 30.0, 60.0, 0.0, 0.0, 60.0, 0.0], 0.25),
        ]
        
        print("\n执行关节角度序列:")
        for i, (joints, vel) in enumerate(joint_targets, 1):
            print(f"\n目标 {i}/{len(joint_targets)}")
            api.move_joint(joints, velocity=vel)


def main():
    """主函数"""
    print("\n" + "=" * 60)
    print("KUKA机器人API使用示例")
    print("=" * 60)
    print("\n注意: 确保KUKA控制器上运行的是 LBRRobotAPIServer.java\n")
    
    print("请选择示例:")
    print("  1 - 笛卡尔坐标运动")
    print("  2 - 关节角度运动")
    print("  3 - 获取当前位姿")
    print("  4 - 获取当前关节角度")
    print("  5 - 组合操作（运动 + 获取位姿）")
    print("  6 - 序列运动")
    print("  7 - 运行所有示例")
    
    try:
        choice = input("\n请输入选项 (1-7): ").strip()
        
        if choice == "1":
            demo_cartesian_motion()
        elif choice == "2":
            demo_joint_motion()
        elif choice == "3":
            demo_get_pose()
        elif choice == "4":
            demo_get_joint()
        elif choice == "5":
            demo_combined_operations()
        elif choice == "6":
            demo_sequence_motion()
        elif choice == "7":
            demo_cartesian_motion()
            demo_joint_motion()
            demo_get_pose()
            demo_get_joint()
            demo_combined_operations()
            demo_sequence_motion()
        else:
            print("✗ 无效的选项")
            return 1
        
        print("\n" + "=" * 60)
        print("示例完成")
        print("=" * 60)
        return 0
        
    except KeyboardInterrupt:
        print("\n\n程序被用户中断")
        return 1
    except Exception as e:
        print(f"\n✗ 发生错误: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    import sys
    sys.exit(main())

