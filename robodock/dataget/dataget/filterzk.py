import rclpy
from rclpy.node import Node
from motion_msgs.msg import MotionCtrl
from tensorrt_yolo_msg.msg import Results
import numpy as np
import csv
import time

class KalmanFilter1D:
    """一维卡尔曼滤波器"""
    def __init__(self, process_variance=1.0, measurement_variance=1.0, initial_value=0.0):
        self.process_variance = process_variance  # 过程噪声
        self.measurement_variance = measurement_variance  # 测量噪声
        self.estimate = initial_value  # 初始估计值
        self.estimate_error = 0.0  # 初始估计误差
    
    def update(self, measurement):
        # 预测步骤
        prediction_error = self.estimate_error + self.process_variance
        
        # 更新步骤
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.estimate_error = (1 - kalman_gain) * prediction_error
        
        return self.estimate
    
class lowpass_filter:
    def __init__(self, alpha=0.01):
        self.alpha = alpha
        self.last_value = None

    def update(self, value):
        if self.last_value is None:
            self.last_value = value
        else:
            # 保持符号与测量值一致
            sign = -1 if value < 0 else 1
            self.last_value = (1 - self.alpha) * self.last_value + self.alpha * value
            # self.last_value = sign * abs(self.last_value)  # 强制符号匹配
        return self.last_value

class AutoNavNode(Node):
    def __init__(self):
        super().__init__('filter_node')

        # ===== ROS 通信接口 =====
        self.publisher = self.create_publisher(MotionCtrl, '/diablo/MotionCmd', 2)
        self.subscription = self.create_subscription(
            Results,
            '/infer_results',
            self.target_callback,
            10)

        # ===== 控制参数 =====
        self.ctrl_msg = MotionCtrl()
        self.kp = 0.01            # 前进比例控制增益
        self.kp_x = 0.01         # 转弯比例控制增益
        self.v_max = 0.2        # 最大前进速度（m/s）
        self.turn_gain = 0.05     # 左右修正增益（备用控制）
        self.leg_gain = 0.8       # 上下修正增益（备用控制）
        self.rotate_speed = 0.1   # 原地旋转速度
        self.r_max = 0.05      # 最大左右修正速度（m/s）

        # ===== 目标检测数据缓存 =====
        self.current_detection = None
        self.target_x = 0.00  
        self.target_y = 0.00
        self.target_z = 0.00 
        
        # ===== 卡尔曼滤波器 =====
        self.kalman_filter = KalmanFilter1D(
            process_variance=1.0,    # 过程噪声，根据目标运动速度调整
            measurement_variance=5.0, # 测量噪声，根据传感器精度调整
            initial_value=0.0
        )

        # ===== 低通滤波器 =====
        self.lowpass_filter = lowpass_filter(alpha=0.1)  # 低通滤波器实例
        
        # ===== 定时前进控制参数 =====
        self.forward_start_time = None
        self.required_duration = 0.0
        self.is_forwarding = False
        self.status = 0

        # ===== 数据记录 =====
        self.data_records = []
        self.start_time = time.time()

        # ===== 控制循环 10Hz =====
        self.timer = self.create_timer(0.1, self.control_loop)

    def clip_speed(self, speed: float, max: float) -> float:
        if abs(speed) < abs(max):
            return speed
        if abs(speed) > abs(max):
            return max if speed > 0 else -max
        return 0.0

    def generate_msgs(self, forward: float, left: float, up: float):
        self.ctrl_msg.value.forward = forward
        self.ctrl_msg.value.left = left
        self.ctrl_msg.value.up = up
        self.publisher.publish(self.ctrl_msg)
        self.get_logger().debug(f"CMD: Fwd={forward:.2f}, left={left:.2f}, Up={up:.2f}")

    def control_loop(self):
        if not self.current_detection:
            self.get_logger().info("No magnet detected. Rotating.")
            self.generate_msgs(0.0, -self.rotate_speed, 0.0)
            return
            
        magnet_z = self.target_z 
        # 使用卡尔曼滤波后的x值
        magnet_x = self.filtered_x - magnet_z * 0.01 - 6.0  # 目标 x 坐标修正

        if 0 < magnet_z < 55 and self.current_detection and self.status == 0:
            self.status = 1
            self.get_logger().warn("B condition occur")
            self.forward_speed = 0.1
            self.required_duration = 5.0
            self.forward_start_time = self.get_clock().now()
            self.get_logger().info("[B] Too close (<0.5m) and no magnet. ")
            self.is_forwarding = True
            return

        if self.is_forwarding:
            elapsed = (self.get_clock().now() - self.forward_start_time).nanoseconds / 1e9
            if elapsed < self.required_duration:
                self.generate_msgs(self.forward_speed, 0.0, 0.0)
            else:
                self.generate_msgs(0.0, 0.0, 0.0)
                self.get_logger().info(f"[B] Forwarding completed ")
                self.is_forwarding = False
            return

        if 55 <= magnet_z < 300:
            self.get_logger().warn("A condition occur")
            self.status = 0
            forward_speed = self.clip_speed(self.kp * magnet_z, self.v_max)
            left_cmd = -self.clip_speed(self.kp_x * magnet_x, self.r_max) if self.current_detection else 0.0
            self.get_logger().info(f"[A] Magnet@{magnet_z:.2f}cm → Forward = {forward_speed:.2f} m/s, Left = {left_cmd:.2f} m/s")
            self.generate_msgs(forward_speed, left_cmd, 0.0)
            return

    def target_callback(self, msg: Results):
        if not msg.results:
            self.get_logger().debug("Received empty detection results")
            return

        for result in msg.results:
            try:
                if not hasattr(result, 'class_id') or not hasattr(result, 'coordinate'):
                    self.get_logger().warn("Invalid detection result structure")
                    continue
                self.current_detection = result

                class_id = int(result.class_id)
                coordinates = result.coordinate
                
                if len(coordinates) < 3:
                    self.get_logger().warn(f"Incomplete coordinates for class {class_id}")
                    continue

                # 原始x坐标
                raw_x = float(coordinates[0])
                # raw_z = float(coordinates[2])
                # 使用卡尔曼滤波更新x坐标
                self.filtered_x = self.kalman_filter.update(raw_x)
                
                self.target_x = raw_x  # 仍然保留原始值用于记录
                self.target_y = float(coordinates[1])
                self.target_z = float(coordinates[2])

                # 记录数据
                current_time = time.time() - self.start_time
                self.data_records.append({
                    'time': current_time,
                    'raw_x': raw_x,
                    'filtered_x': self.filtered_x
                })

                self.get_logger().debug(
                    f"[DETECT] Class {class_id}: "
                    f"x={coordinates[0]:.2f}(filtered={self.filtered_x:.2f}), "
                    f"y={coordinates[1]:.2f}, "
                    f"z={coordinates[2]:.2f}"
                )
            except (ValueError, TypeError) as e:
                self.get_logger().error(f"Data processing error: {str(e)}")
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {str(e)}", throttle_duration_sec=5)

    def save_data_to_csv(self, filename=None):
        if filename is None:
            import os
            filename = os.path.join(os.getcwd(), 'filtered_data.csv')  # 默认当前目录
    
        with open(filename, mode='w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=['time', 'raw_x', 'filtered_x'])
            writer.writeheader()
            for record in self.data_records:
                writer.writerow(record)
        self.get_logger().info(f"Data saved to {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = AutoNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_data_to_csv()  # 保存数据到CSV文件
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()