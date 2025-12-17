# 🤖 LOS Path Following Robot (ROS 2 & Gazebo)

![ROS2](https://img.shields.io/badge/ROS2-Humble%7CIron-blue)
![Gazebo](https://img.shields.io/badge/Simulation-Gazebo%20Ignition-orange)
![License](https://img.shields.io/badge/License-MIT-green)

Dự án này triển khai thuật toán **Line-of-Sight (LOS)** kết hợp với bộ điều khiển **PID** để điều hướng robot tự hành bám theo quỹ đạo định sẵn trong môi trường mô phỏng **Gazebo Ignition/Fortress**.

Hệ thống bao gồm tính năng sửa lỗi trượt bánh (**Odom Drift Correction**) bằng cách sử dụng Ground Truth từ Gazebo và công cụ Visualize thời gian thực bằng Python/Matplotlib.

---

## 📸 Demo & Visualization

<img width="400" height="400" alt="Figure_01" src="https://github.com/user-attachments/assets/d39976e2-1c46-46a6-8dfb-f2d21b2a8659" />

[Screencast from 12-17-2025 09:25:07 PM.webm](https://github.com/user-attachments/assets/27ff3a84-c603-43d7-98ae-7d3fbb80a57b)



---

## 🛠️ Yêu cầu hệ thống (Prerequisites)

* **ROS 2** (Humble/Foxy/Iron)
* **Gazebo Ignition/Fortress**
* **Thư viện Python:** `matplotlib`, `numpy`
* **Package phụ trợ:** `ros_gz_bridge`, `tf2_ros`

---

## 📦 Cài đặt (Installation)

**1. Clone repository về workspace:**
```bash
cd ~/ros2_ws/src
git clone https://github.com/MinhV201/ROS2-HUST.git
```
**2. Build workspace:**
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
## 🚀 Hướng dẫn khởi chạy (Usage)

**1. Launh Gazebo and Rivz (Terminal 1):**
```bash
ros2 launch my_robot_bringup display.launch.py
```
**2. Run node PF (Terminal 2) :**
```bash
ros2 run ros_x pf_node
```
**3. Run node path_sender (Terminal 3) :**
```bash
ros2 run ros_x_bringup path_sender
```
**4. Run node path_debug to plot (Terminal 4) :**
```bash
ros2 run ros_x_bringup debug_plotter 
```
