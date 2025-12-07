# 🐕 Quadruped Robot Simulation — ROS 2 Humble (Ubuntu 22.04)

This repository contains the full simulation setup for a custom-designed quadruped robot built for gait generation, URDF visualization, and Gazebo/RViz-based testing.
The mechanical design, control logic, and simulation configurations were developed as part of a robotics project focused on modular quadruped locomotion.

The project includes:

* URDF model generated from SolidWorks
* STL meshes for all robot links
* ros2_control configuration
* Gait pattern Python scripts
* RViz visualization
* Gazebo Classic simulation
* Launch files for testing, debugging, and visualization

📄 **Full project report:** Included in this repository (`Quadruped_report.pdf`) 

---

# 📌 Tested On

| Component     | Version                |
| ------------- | ---------------------- |
| **OS**        | Ubuntu 22.04 LTS       |
| **ROS**       | ROS 2 Humble Hawksbill |
| **Simulator** | Gazebo Classic 11      |
| **RViz**      | RViz2                  |
| **Python**    | 3.10                   |

---

# 📸 Project Images

> Replace the placeholders below by inserting your actual images.

### 1️⃣ Quadruped CAD Model

<img width="878" height="581" alt="Screenshot 2025-12-06 at 7 45 06 PM" src="https://github.com/user-attachments/assets/b5c6a3fa-ab37-4bac-ab43-4fcf0d22272d" />
<img width="598" height="505" alt="Screenshot 2025-12-06 at 9 08 12 PM" src="https://github.com/user-attachments/assets/0f9b1779-6e08-4f50-83e2-bc1185a80424" />



---

### 2️⃣ Leg Assembly
<img width="552" height="375" alt="Screenshot 2025-12-06 at 9 09 31 PM" src="https://github.com/user-attachments/assets/a12475b2-ffaf-4034-8684-44fbbb3e1462" />



---

### 3️⃣ URDF Tree Diagram

Generate the tree using:

```bash
urdf_to_graphviz new_assembly_URDF.urdf
```
<img width="612" height="274" alt="Screenshot 2025-12-06 at 9 10 11 PM" src="https://github.com/user-attachments/assets/d6d79cd7-3e86-491c-bb46-67d898939d9f" />


---

### 4️⃣ RViz TF Visualization
<img width="601" height="416" alt="Screenshot 2025-12-06 at 9 10 36 PM" src="https://github.com/user-attachments/assets/2f3a361f-0bc7-4031-82f7-e88cbf9d189b" />



---

### 5️⃣ Gazebo Spawn Screenshot
<img width="594" height="320" alt="Screenshot 2025-12-06 at 9 11 04 PM" src="https://github.com/user-attachments/assets/4d33cf2b-ae8b-4819-9ca1-e08cea067fb0" />



---

# 📁 Repository Structure

```
robot_dog_description/
│
├── config/
│   ├── collision_only.rviz
│   ├── dog_ros2_control.yaml
│   └── joint_names_new_assembly_URDF.yaml
│
├── include/robot_dog_description/
│
├── launch/
│   ├── cmd_vel_to_command.launch.py
│   ├── collision_view.launch.py
│   ├── display.launch.py
│   ├── gazebo.launch.py
│   └── (other launch files)
│
├── meshes/
│   ├── base_link.STL
│   ├── lf1_Link.STL
│   ├── lf2_Link.STL
│   ├── rf1_Link.STL
│   ├── rf2_Link.STL
│   ├── lw1_Link.STL
│   └── (all leg link STL files)
│
├── scripts/
│   ├── base_mover.py
│   ├── gait_patterns.py
│   └── gait_trajectory.py
│
├── urdf/
│   ├── new_assembly_URDF.urdf
│   ├── new_assembly_URDF.csv
│   ├── Assem_urdf.gv
│   └── Assem_urdf.pdf
│
└── worlds/
    ├── empty.sdf
    └── flat_ground.world
```

---

# 🛠 Installation

### 1. Install dependencies

```bash
sudo apt update
sudo apt install ros-humble-desktop-full \
                 ros-humble-urdf-tutorial \
                 ros-humble-xacro \
                 ros-humble-gazebo-ros \
                 ros-humble-joint-state-publisher \
                 ros-humble-robot-state-publisher
```

---

# 🚀 Build the Workspace

```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --merge-install
```

Source it:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
```

---

# 🦴 Run the Quadruped

## 1️⃣ Visualize URDF in RViz

```bash
ros2 launch robot_dog_description display.launch.py
```

This loads:

* Robot model
* Joint frames
* TF tree
* Mesh visualization

---

## 2️⃣ Spawn the Robot in Gazebo

```bash
ros2 launch robot_dog_description gazebo.launch.py
```

This will:

* Start Gazebo Classic
* Load STL meshes
* Spawn URDF model
* Bring up ros2_control interfaces (if enabled in URDF)

---

## 3️⃣ Run Gait Pattern (RViz Animation)

```bash
ros2 run robot_dog_description gait_patterns.py
```

This script outputs sinusoidal and cyclic gait trajectories for:

* Hip joints
* Knee joints

and visualizes the stepping motion inside RViz.

---

# 🧩 ros2_control Notes

The URDF includes a `<ros2_control>` block with:

* Position command interface
* Velocity + position state feedback
* `gazebo_ros2_control/GazeboSystem` plugin

Configuration file:

```
config/dog_ros2_control.yaml
```

List loaded controllers:

```bash
ros2 control list_controllers
```

---

# 📝 Important Notes (From Detailed Report)

From the *Quadruped_report.pdf* included in this repo:


* STL meshes require correct origin alignment
* CAD export inertias are very small → Gazebo instability
* Legs may jitter or explode under physics due to low mass
* Gait visualization works best in RViz
* Hardware servos (Waveshare ST3025) were partially integrated
* Walking in real physics requires manual inertia tuning & collision simplification

---

# 🔭 Future Work

* Full hardware integration with servo controller board
* PID tuning for each joint
* Stabilizing Gazebo simulation (manual inertias, footpads, collision meshes)
* Implement trotting, pacing, bounding gait modes
* Add IMU & joint encoder feedback
* Hardware-in-the-loop simulation
* Real-time controller + state estimation
