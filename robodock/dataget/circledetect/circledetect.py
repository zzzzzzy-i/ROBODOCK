"""
逻辑链：
1. 初始化 ROS2 节点，启动 Intel RealSense 摄像头（通过 realsense2_camera 节点）。
2. 订阅 /camera/color/image_raw 和 /camera/aligned_depth_to_color/image_raw 话题，使用 message_filters 同步彩色图和深度图。
3. 在回调中将 ROS 图像消息转换为 OpenCV 格式。
4. 对彩色图像进行灰度化、高斯模糊和霍夫圆检测，识别白色圆形电磁铁，获取像素坐标 (u, v) 和半径 r。
5. 从对齐的深度图中读取像素 (u, v) 的深度值 depth（米）。
6. 获取相机内参，调用 rs2_deproject_pixel_to_point，将 (u, v, depth) 转换为相机坐标系下的三维坐标 (X, Y, Z)。
7. 将检测结果（圆心像素坐标、半径、三维坐标）发布为自定义消息或标准消息（geometry_msgs/PointStamped）。
8. 可视化输出：在彩色图像上绘制检测结果并通过 image_pub 发布或 OpenCV 窗口显示。

Node 结构：
- 类 CircleDetector(Node)
  - 构造函数：初始化订阅、发布器、CvBridge、相机内参占位
  - init_camera_info：从 camera_info 话题获取内参
  - image_callback：同步回调处理图像、检测圆、反投影、发布结果

"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import message_filters
import cv2
import numpy as np
import pyrealsense2 as rs

class CircleDetector(Node):
    def __init__(self):
        super().__init__('circle_detector')
        # 初始化 CvBridge
        self.bridge = CvBridge()
        # 相机内参变量
        self.intrinsics = None
        # 订阅 CameraInfo 获取内参
        self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10)
        # 同步订阅彩色图和对齐深度图
        color_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw')
        ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.image_callback)
        # 发布检测结果
        self.point_pub = self.create_publisher(PointStamped, 'detected_circle_point', 10)
        self.image_pub = self.create_publisher(Image, 'detected_circle_image', 10)
        self.get_logger().info('CircleDetector 节点已启动')

    def camera_info_callback(self, msg: CameraInfo):
        # 仅获取一次内参
        if self.intrinsics is None:
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = msg.width
            self.intrinsics.height = msg.height
            self.intrinsics.ppx = msg.k[2]
            self.intrinsics.ppy = msg.k[5]
            self.intrinsics.fx = msg.k[0]
            self.intrinsics.fy = msg.k[4]
            # 假设无畸变或仅径向畸变
            self.intrinsics.model = rs.distortion.none
            self.intrinsics.coeffs = [0, 0, 0, 0, 0]
            self.get_logger().info('相机内参已获取')

    def image_callback(self, color_msg: Image, depth_msg: Image):
        if self.intrinsics is None:
            return  # 尚未获取内参，跳过

        # 转换为 OpenCV 图像
        color_image = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

        # 1. 预处理：灰度 + 高斯模糊
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2)

        # 2. 霍夫圆检测（参数可调）：
        #    dp       = 1             # 累加器分辨率比（1 表示与原图同分辨率，可设 2 提速但精度略低）
        #    minDist  = 100           # 检测到的圆心最小距离（像素），可设为圆半径 1~2 倍，防止重复检测
        #    param1   = 50            # Canny 高阈值（低阈值为 0.5*param1），可 30~150 调整以滤弱边缘或保留细节
        #    param2   = 30            # 霍夫累加器阈值，越大越严格（减少误检），越小越容易漏检
        #    minRadius= 10            # 最小圆半径（像素），根据电磁铁实际大小与拍摄距离估算
        #    maxRadius= 100           # 最大圆半径（像素），同上
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=100,
            param1=50,
            param2=30,
            minRadius=10,
            maxRadius=100)

        if circles is not None:
            circles = np.uint16(np.around(circles[0, :]))
            u, v, r = circles[0]  # 取第一个圆心坐标和半径
            # 绘制检测结果
            cv2.circle(color_image, (u, v), r, (0, 255, 0), 2)
            cv2.circle(color_image, (u, v), 2, (0, 0, 255), 3)

            # 3. 获取深度并反投影
            depth = depth_image[v, u] * 0.001  # 深度单位转换：若为毫米，则乘 0.001 得到米
            X, Y, Z = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth)

            # 4. 发布三维坐标
            point_msg = PointStamped()
            point_msg.header = depth_msg.header
            point_msg.point.x = X
            point_msg.point.y = Y
            point_msg.point.z = Z
            self.point_pub.publish(point_msg)

        # 发布带标注的图像
        image_out_msg = self.bridge.cv2_to_imgmsg(color_image, 'bgr8')
        image_out_msg.header = color_msg.header
        self.image_pub.publish(image_out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CircleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
