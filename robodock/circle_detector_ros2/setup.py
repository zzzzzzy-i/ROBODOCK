from setuptools import setup

package_name = 'circle_detector_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装 launch 文件
        ('share/' + package_name + '/launch', ['launch/detect.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='你的名字',
    maintainer_email='your@email.com',
    description='使用 Realsense 和 OpenCV 检测白色圆形电磁铁并发布三维坐标',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle_detector_node = circle_detector_ros2.circle_detector_node:main',
        ],
    },
)
