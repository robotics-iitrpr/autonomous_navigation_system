# Autonomous Navigation System (ANS)

A **3-wheeled omnidirectional robot** capable of **mapping unknown environments** and **navigating autonomously** using ROS 2.
Built with a **Kiwi drive mechanism**, this bot can travel in any direction on a 2D plane.

---

## 📸 Overview

* **Electronics**

  * **Raspberry Pi 5** (ROS 2 master, sensor interface)
  * **SLLIDAR** (LiDAR sensor)
  * **3 × Arduino Nano** (one per motor with encoder)
  * **BNO055 IMU** (orientation + acceleration data)
  * **3 × 12V geared DC motors with encoders**
  * **4-cell 14.8V battery** (motors power)
  * **Buck converter (14.8V → 5V)** (encoder + logic power)

* **Mechanics**

  * Triangular **laser-cut chassis**
  * **3 omni-wheels**, one at each corner
  * **Kiwi drive configuration**

* **Software**

  * **OS**: Ubuntu 24.04
  * **ROS 2 Jazzy Jalisco**
  * **SLAM Toolbox** (mapping)
  * **Nav2** (navigation + path planning)

---

## 🛠 Code Structure

```
├── ans_ws/                         # Main ROS 2 workspace
│   ├── bno055_driver/              # IMU driver
│   │   └── bno055_node             # Publishes /imu/data, /imu/euler, /imu/quaternion
│   │
│   ├── encoder_publish/            # Motor control + encoder handling
│   │   ├── forward.py              # Test: move forward
│   │   ├── backward.py             # Test: move backward
│   │   ├── calc_rpm.py             # Converts raw counts → /rpm{A,B,C}
│   │   ├── joystick.py             # Control with joystick
│   │   ├── omni_teleop.py          # Control with keyboard
│   │   ├── publish.py              # Serial comms with Arduinos (encoder + motor commands)
│   │   └── rotate.py               # Test: rotate bot
│   │
│   ├── pos_calc/                   # Odometry
│   │   └── pos_calc                # Combines IMU + RPM → /odom
│   │
│   └── ...
│
├── ws_lidar/                       # Lidar workspace
│   └── sllidar_ros2/               # Publishes /scan
│
├── arduino_code.cpp                # Runs on each Nano (encoder + motor control)
│
└── ...
```

---

## ⚙️ System Flow

1. **On the Bot (Raspberry Pi 5):**

   * Collects data from **LiDAR**, **IMU**, and **encoders**.
   * Publishes sensor topics via ROS 2.

2. **On the PC (Remote workstation):**

   * Subscribes to shared topics over WiFi.
   * Processes sensor data:

     * Compute RPM & odometry.
     * Map environment using **SLAM Toolbox**.
     * Plan paths with **Nav2**.
   * Sends velocity/motor commands back to Pi → Arduinos → motors.

---

## 🚀 Setup & Usage

### 1. Network Configuration

* Setup a PC with Ubuntu 24.04 and ROS 2 Jazzy.
 
* Install slam_toolbox and Nav2 on the PC.
  
* Clone this repo to your PC and build ans_ws.

### 2. Network Configuration

* Connect **Raspberry Pi** and **PC** to the same WiFi.

* Set ROS domain ID (on both PC and Pi):

  ```bash
  export ROS_DOMAIN_ID=0
  ```

* SSH into Pi:

  ```bash
  ssh ans@<pi-ip-address>
  # password: ans
  ```

### 3. Run on Raspberry Pi

In separate terminals:

```bash
sudo ./start.sh
ros2 launch sllidar_ros2 sllidar_a1_launch.py
ros2 run bno055_driver bno055_node
ros2 run encoder_publish encoder_publish
```

### 4. Run on PC

In separate terminals:

```bash
ros2 run encoder_publish calc_rpm
ros2 run pos_calc pos_calc
ros2 launch slam_toolbox online_async_launch.py
```

### 5. Visualization

```bash
rviz2
```

* View map on `/map`.
* Monitor odometry on `/odom`.
* Send navigation goals via **Nav2 RViz panel**.

---

## 🧭 Features

* Omni-directional **Kiwi drive**.
* Real-time **mapping & localization** with LiDAR + IMU + encoders.
* Autonomous **navigation with obstacle avoidance**.
* **Teleop control** (joystick + keyboard).

---
