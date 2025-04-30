# 双轮足机器人的对接技术

## 1. 项目介绍
本项目实现双轮足机器人与目标平台的自动对接技术，主要包含以下功能：
- **高精度定位**：基于激光雷达/视觉的位姿估计
- **运动控制**：动态路径规划与轮足协同运动
- **对接验证**：接触力检测与状态反馈

## 2. 环境配置
### 2.1 硬件要求
| 设备 | 规格 |
|-------|-------|
| 机器人 | 双轮足平衡机器人（如MIT Mini Cheetah） |
| 主控 | Intel NUC i7 + ROS主控板 |
| 传感器 | Intel Realsense D435i / Velodyne 16线雷达 |

### 2.2 软件依赖
```bash
# Ubuntu 20.04
sudo apt install ros-noetic-desktop-full
pip install numpy opencv-contrib-python
