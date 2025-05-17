"""
===========================================================
 🦾 Diablo 机器人自主导航控制逻辑（基于YOLO目标检测）

 📌 功能概述：
 本节点接收来自目标检测节点（YOLO识别）的目标信息，根据检测到的目标类别
（电磁铁 class_id=0，机器人 class_id=1）及其空间位置信息，自动控制机器人前进、
转向、腿高调节等，实现基于视觉的自主导航与跟踪控制。

 📍 控制流程逻辑（按优先级）：

 1️⃣ 未检测到机器人（class_id=1）和电磁铁（class_id=0）：
     → 原地顺时针旋转，持续扫描环境，尝试重新识别目标。

 2️⃣ 情况 A：检测到机器人，且其距离 robot_z > 0.8 米：
     → 利用机器人距离 robot_z 进行 KP 比例控制（v = Kp * robot_z），同时结合
        目标 x 坐标进行横向调整（left = -kp_x * target_x），速度限制在 [v_min, v_max] 内。

 3️⃣ 情况 C：机器人距离在 (0, 1.0)m 且检测到电磁铁时：
     → 利用电磁铁距离 magnet_z 进行 KP 比例控制（v = Kp * magnet_z），并利用目标 x 坐标调整横向运动。

 4️⃣ 情况 B：检测到机器人且其距离在 (0, 0.3)m 且未检测到电磁铁（magnet_z = 0）：
     → 启动“盲推”策略，以固定前进速度 0.2 m/s 向前推进 5 秒（不进行左右调整）。

 5️⃣ 默认跟踪模式（Fallback）：
     → 若检测到目标但不满足上述条件，使用机器人距离进行 KP 比例前进（裁剪后）
        同时根据目标 x/y 偏差进行左右及腿高调整。

 ⚙️ 控制策略说明：
 - 前进控制（除情况 B）采用比例控制：v = Kp × 距离，并剪裁在 [v_min, v_max]
 - 横向控制：在情况 A、C以及 fallback 中使用目标 x 偏差进行左右调整
 - 情况 B 中固定前进，不做横向调整

===========================================================
"""

import rclpy
from rclpy.node import Node
from motion_msgs.msg import MotionCtrl
from tensorrt_yolo_msg.msg import Results

class AutoNavNode(Node):
    def __init__(self):
        super().__init__('xzzk_node')

        # ===== ROS 通信接口 =====
        self.publisher = self.create_publisher(MotionCtrl, '/diablo/MotionCmd', 2)
        self.subscription = self.create_subscription(
            Results,
            '/infer_results',
            self.target_callback,
            10)

        # ===== 控制参数 =====
        self.ctrl_msg = MotionCtrl()
        self.kp = 0.01            # 前进比例控制增益（用于计算 forward = kp * distance）
        self.kp_x = 0.01         # 转弯比例控制增益（用于计算 left = -kp_x * target_x）
        # self.v_min = 0.05         # 最小前进速度（m/s）
        self.v_max = 0.2        # 最大前进速度（m/s）
        self.turn_gain = 0.05     # 左右修正增益（备用控制）
        self.leg_gain = 0.8       # 上下修正增益（备用控制）
        self.rotate_speed = 0.1   # 原地旋转速度
        self.r_max = 0.05      # 最大左右修正速度（m/s）

        # ===== 目标检测数据缓存 =====
        self.current_detection = None
        self.target_x = 0.00  
        self.target_z = 0.00 
        # ===== 定时前进控制参数（用于情况B） =====
        self.forward_start_time = None   # 定时前进开始时间
        self.required_duration = 0.0       # 定时前进持续时间（秒）
        self.is_forwarding = False         # 是否处于定时前进状态

        # ===== 控制循环 10Hz =====
        self.timer = self.create_timer(0.1, self.control_loop)
        # ===== 记录是否进入b情况 =====
        self.status = 0

    # 辅助函数：裁剪前进速度到 [v_min, v_max]
    def clip_speed(self, speed: float, max: float) -> float:
        if abs(speed) < abs(max):
            return speed
        if abs(speed) > abs(max):
            return max if speed > 0 else -max
        return 0.0

    # 发布运动控制消息
    def generate_msgs(self, forward: float, left: float, up: float):
        self.ctrl_msg.value.forward = forward
        self.ctrl_msg.value.left = left
        self.ctrl_msg.value.up = up
        self.publisher.publish(self.ctrl_msg)
        self.get_logger().debug(f"CMD: Fwd={forward:.2f}, left={left:.2f}, Up={up:.2f}")

    # 停止运动并安全关闭节点
    """ def shutdown_node(self):
        self.get_logger().info("Mission complete. Stopping and shutting down.")
        self.generate_msgs(0.0, 0.0, 0.0)
        self.destroy_node()
        rclpy.shutdown() """

    # 主控制逻辑循环
    def control_loop(self):
        # 1️⃣ 未检测到机器人和电磁铁时，原地顺时针旋转
        if not self.current_detection :
            self.get_logger().info("No magnet detected. Rotating.")
            self.generate_msgs(0.0, -self.rotate_speed, 0.0)
            return
        # 获取目标距离信息
        # robot_z = self.target_z_dict.get(1, 0.0)
        # robot_x = self.target_x_dict.get(1, 0.0)
        magnet_z = self.target_z
        magnet_x = self.target_x - self.target_z * 0.01 - 5 # 目标 x 坐标修正

        # 4️⃣ 情况 B：当机器人距离在 (0, 0.5)m 且未检测到电磁铁时（不进行左右调整）
        if 0< magnet_z < 55 and self.current_detection and self.status == 0:
            self.status = 1
            self.get_logger().warn("B condition occur")
            self.forward_speed = 0.1  # 固定前进速度
            self.required_duration = 5.0  # 定时
            self.forward_start_time = self.get_clock().now()
            self.get_logger().info("[B] Too close (<0.5m) and no magnet. ")
            self.is_forwarding = True
            # self.generate_msgs(self.forward_speed, 0.0, 0.0)
            return

        # 定时前进过程（情况 B 正在进行）
        if self.is_forwarding:
            elapsed = (self.get_clock().now() - self.forward_start_time).nanoseconds / 1e9
            if elapsed < self.required_duration:
                self.generate_msgs(self.forward_speed, 0.0, 0.0)
            else:
                self.generate_msgs(0.0, 0.0, 0.0)
                self.get_logger().info(f"[B] Forwarding completed ")
                self.is_forwarding = False
            return

        # 3️⃣ 情况 A：当机器人距离在 (0, 1.0)m 且检测到电磁铁时
        if 55 <= magnet_z < 300:
            self.get_logger().warn("A condition occur")
            self.status = 0
            # 使用电磁铁距离计算前进速度并裁剪
            forward_speed = self.clip_speed(self.kp * magnet_z, self.v_max)
            left_cmd = -self.clip_speed(self.kp_x * magnet_x, self.r_max) if self.current_detection else 0.0
            self.get_logger().info(f"[A] Magnet@{magnet_z:.2f}cm → Forward = {forward_speed:.2f} m/s, Left = {left_cmd:.2f} m/s")
            self.generate_msgs(forward_speed, left_cmd, 0.0)
            return
        """         
        # 2️⃣ 情况 A：当机器人距离 > 0.8m 时
        if magnet_z >=:
            self.get_logger().warn("A condition occur")
            # 使用机器人距离计算前进速度并裁剪
            forward_speed = self.clip_speed(self.kp * magnet_z, self.v_max)
            # 结合目标 x 坐标进行左右修正
            left_cmd = -self.clip_speed(self.kp * magnet_z, self.r_max) if self.current_detection else 0.0
            self.get_logger().info(f"[A] Robot far ({magnet_z:.2f}m). Forward = {forward_speed:.2f} m/s, left = {left_cmd:.2f} m/s")
            self.generate_msgs(forward_speed, left_cmd, 0.0)
            return

        """

        """  # 5️⃣ fallback 跟踪控制：当检测到目标但不满足前述条件时
        if self.current_detection:
            self.get_logger().debug("D condition occur")
            target_x = self.current_detection.center.x
            target_y = self.current_detection.center.y
            # 备用左右调整
            left_cmd = -self.r_max
            up_cmd = self.leg_gain * target_y
            forward_speed = self.clip_speed(self.kp * robot_z) if robot_z > 0 else 0.0
            self.get_logger().info(f"[TRACK] Forward={forward_speed:.2f}, Left={left_cmd:.2f}, Up={up_cmd:.2f}")
            self.generate_msgs(forward_speed, left_cmd, up_cmd)
        else:
            self.generate_msgs(0.0, -self.rotate_speed, 0.0)
        """
    # 目标检测回调：存储检测到的目标数据
    def target_callback(self, msg: Results):
        """处理目标检测结果的回调函数，存储并记录检测到的目标信息"""
        if not msg.results:  # 检查结果是否为空
            self.get_logger().debug("Received empty detection results")
            return

        for result in msg.results:  # 处理多个检测结果
            try:
                # 验证必要字段存在
                if not hasattr(result, 'class_id') or not hasattr(result, 'coordinate'):
                    self.get_logger().warn("Invalid detection result structure")
                    continue
                self.current_detection = result  # 更新当前检测结果

                class_id = int(result.class_id)
                coordinates = result.coordinate
                
                # 验证坐标数据完整性
                if len(coordinates) < 3:
                    self.get_logger().warn(f"Incomplete coordinates for class {class_id}")
                    continue

                # 更新目标字典
                self.target_x = float(coordinates[0])
                self.target_y = float(coordinates[1])
                self.target_z = float(coordinates[2])

                # 记录检测信息（使用DEBUG级别减少日志量）
                self.get_logger().debug(
                    f"[DETECT] Class {class_id}: "
                    f"x={coordinates[0]:.2f}, "
                    f"y={coordinates[1]:.2f}, "
                    f"z={coordinates[2]:.2f}"
                )
            except (ValueError, TypeError) as e:
                self.get_logger().error(f"Data processing error: {str(e)}")
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {str(e)}", throttle_duration_sec=5)

# ROS 入口
def main(args=None):
    rclpy.init(args=args)
    node = AutoNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
