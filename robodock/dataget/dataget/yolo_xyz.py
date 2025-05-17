import rclpy
from rclpy.node import Node
from motion_msgs.msg import MotionCtrl
from tensorrt_yolo_msg.msg import Results
import numpy as np

""" 还需要设置和调试:理想x值,理想y值,卡尔曼误差系数 """




    

class KalmanFilter1D:
    """一维卡尔曼滤波器"""
    def __init__(self, process_variance=1.0, measurement_variance=1.0, initial_value=0.0):
        self.process_variance = process_variance  # 过程噪声
        self.measurement_variance = measurement_variance  # 测量噪声
        self.estimate = initial_value  # 初始估计值
        self.estimate_error = 1.0  # 初始估计误差
    
    def update(self, measurement):
        # 预测步骤
        prediction_error = self.estimate_error + self.process_variance
        
        # 更新步骤
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.estimate_error = (1 - kalman_gain) * prediction_error
        
        return self.estimate

class AutoNavNode(Node):
    def __init__(self):
        super().__init__('yoloxyz_node')

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
        self.kp_y = 0.01 
        self.vy_max = 0.05        # 最大垂直速度
        self.vx_max = 0.2        # 最大前进速度（m/s）
        self.turn_gain = 0.05     # 左右修正增益（备用控制）
        self.leg_gain = 0.8       # 上下修正增益（备用控制）
        self.rotate_speed = 0.1   # 原地旋转速度
        self.r_max = 0.05      # 最大左右修正速度（m/s）
        self.forward_speed = 0.0

        # ===== 目标检测数据缓存 =====
        self.current_detection = None
        self.target_x = 0.00  
        self.target_y = 0.00
        self.target_z = 0.00
        self.up_cmd = 0.0 
        
        # ===== 卡尔曼滤波器 =====
        self.kalman_filter_x = KalmanFilter1D(
            process_variance=3.0,    # 过程噪声，根据目标运动速度调整
            measurement_variance=2.0, # 测量噪声，根据传感器精度调整
            initial_value=0.0
        )
        self.kalman_filter_y = KalmanFilter1D(
            process_variance=3.0,    # 过程噪声，根据目标运动速度调整
            measurement_variance=2.0, # 测量噪声，根据传感器精度调整
            initial_value=0.0
        )
        self.kalman_filter_z = KalmanFilter1D(
            process_variance=3.0,    # 过程噪声，根据目标运动速度调整
            measurement_variance=2.0, # 测量噪声，根据传感器精度调整
            initial_value=0.0
        )
        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.filtered_z = 0.0
        
        # ===== 定时前进控制参数 =====
        self.forward_start_time = None
        self.required_duration = 0.0
        self.is_forwarding = False
        self.status = 0

        # ===== 控制模式标志位 =====
        # self.init_msgs(True,True,self.up_cmd)

        # ===== 控制循环 10Hz =====
        self.timer = self.create_timer(0.1, self.control_loop)

    def quit(self):
        self.ctrl_msg.value.forward = 0.0
        self.ctrl_msg.value.left = 0.0
        self.publisher.publish(self.ctrl_msg)
        return

    def clip_speed(self, speed: float, max: float) -> float:
        if abs(speed) < abs(max):
            return speed
        if abs(speed) > abs(max):
            return max if speed > 0 else -max
        return 0.0

    def xz_msgs(self, forward: float, left: float):
        self.ctrl_msg.value.forward = forward
        self.ctrl_msg.value.left = left
        self.publisher.publish(self.ctrl_msg)
        self.get_logger().debug(f"CMD: Fwd={forward:.2f}, left={left:.2f}")

    def xyz_msgs(self, forward: float, left: float, up: float):
        self.ctrl_msg.value.forward = forward
        self.ctrl_msg.value.left = left
        self.publisher.publish(self.ctrl_msg)
        self.ctrl_msg.value.up = up
        self.publisher.publish(self.ctrl_msg)
        self.get_logger().debug(f"CMD: Fwd={forward:.2f}, left={left:.2f}, Up={up:.2f}")

    def init_msgs(self, mode_mark: bool, stand_mode: bool, up: float):
        self.ctrl_msg.mode_mark = mode_mark
        self.ctrl_msg.mode.stand_mode = stand_mode
        self.publisher.publish(self.ctrl_msg)
        self.ctrl_msg.value.up = up
        self.ctrl_msg.value.pitch = 0.0
        self.publisher.publish(self.ctrl_msg)
        self.get_logger().warn(f"CMD: mode_mark={mode_mark}, stand_mode={stand_mode}, Up={up:.2f}")
    
    def control_loop(self):
        if not self.current_detection:
            self.get_logger().info("No magnet detected. Rotating.")
            self.xz_msgs(0.0, -self.rotate_speed)
            return 


        magnet_z = self.filtered_z
        # 使用卡尔曼滤波后的x值
        magnet_x = self.filtered_x - magnet_z * 0.01 - 5  # 目标 x 坐标修正
        magnet_y = self.filtered_y   # 目标 y 坐标修正


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
                self.xz_msgs(self.forward_speed, 0.0)
            else:
                self.xz_msgs(0.0, 0.0)
                self.get_logger().info(f"[B] Forwarding completed ")
                self.is_forwarding = False
            return

        """ if 55 <= magnet_z < 150:
            self.get_logger().warn("A condition occur")
            self.status = 0
            forward_speed = self.clip_speed(self.kp * magnet_z, self.vx_max)
            left_cmd = -self.clip_speed(self.kp_x * magnet_x, self.r_max) if self.current_detection else 0.0
            self.up_cmd = self.up_cmd + self.clip_speed(self.kp_y * magnet_y, self.vy_max)
            self.get_logger().info(f"[A] Magnet@{magnet_z:.2f}cm → Forward = {forward_speed:.2f} m/s, Left = {left_cmd:.2f} m/s, Up = {self.up_cmd:.2f}")
            self.xyz_msgs(forward_speed, left_cmd,self.up_cmd)
            return
        
        if 150 <= magnet_z < 300:
            self.get_logger().warn("A condition occur")
            self.status = 0
            forward_speed = self.clip_speed(self.kp * magnet_z, self.vx_max)
            left_cmd = -self.clip_speed(self.kp_x * magnet_x, self.r_max) if self.current_detection else 0.0
            self.get_logger().info(f"[A] Magnet@{magnet_z:.2f}cm → Forward = {forward_speed:.2f} m/s, Left = {left_cmd:.2f} m/s")
            self.xz_msgs(forward_speed, left_cmd)
            return """
        
        if 55 <= magnet_z < 300:
            self.get_logger().warn("A condition occur")
            self.status = 0
            forward_speed = self.clip_speed(self.kp * magnet_z, self.vx_max)
            left_cmd = -self.clip_speed(self.kp_x * magnet_x, self.r_max) if self.current_detection else 0.0
            self.get_logger().info(f"[A] Magnet@{magnet_z:.2f}cm → Forward = {forward_speed:.2f} m/s, Left = {left_cmd:.2f} m/s")
            self.xz_msgs(forward_speed, left_cmd)
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
                raw_y = float(coordinates[1])
                raw_z = float(coordinates[2])

                # 使用卡尔曼滤波更新x坐标
                self.filtered_x = self.kalman_filter_x.update(raw_x)
                self.filtered_y = self.kalman_filter_y.update(raw_y)
                self.filtered_z = self.kalman_filter_z.update(raw_z)
                self.target_x = raw_x  # 仍然保留原始值用于记录
                self.target_y = raw_y
                self.target_z = raw_z

                self.get_logger().debug(
                    f"[DETECT] Class {class_id}: "
                    f"x={coordinates[0]:.2f}(filtered={self.filtered_x:.2f}), "
                    f"y={coordinates[1]:.2f}(filtered={self.filtered_y:.2f}), "
                    f"z={coordinates[2]:.2f}"
                )
            except (ValueError, TypeError) as e:
                self.get_logger().error(f"Data processing error: {str(e)}")
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {str(e)}", throttle_duration_sec=5)

def main(args=None):
    rclpy.init(args=args)
    node = AutoNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.quit()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()