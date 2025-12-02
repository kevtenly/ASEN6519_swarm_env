# 5-Drone ROS 2 + Gazebo Sim Swarm Environment

This repository contains a **5-drone multicopter swarm** simulated in **Gazebo Sim (Harmonic)** and controlled via **ROS 2 Jazzy**.  
Each drone exposes:

- **State**: `/uav_i/odom` (`nav_msgs/Odometry`)
- **Control**: `/uav_i/cmd_vel` (`geometry_msgs/Twist`)

A sample ROS 2 node (`swarm_controller`) is provided which:

- Subscribes to `/uav_i/odom` for `i = 1..5`
- Publishes `/uav_i/cmd_vel`
- Makes all 5 drones move to a given waypoint or change mode

---

## 1. Prerequisites

Tested on:

- **OS**: Ubuntu 24.04
- **ROS 2**: Jazzy (`echo $ROS_DISTRO` → `jazzy`)
- **Gazebo Sim**: Harmonic (e.g. `Gazebo Sim, version 8.x`)

Install ROS–Gazebo integration:
```bash
sudo apt update
sudo apt install \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  python3-colcon-common-extensions
```

Make sure ROS 2 is sourced (add this to your `~/.bashrc` if you like):
```bash
source /opt/ros/jazzy/setup.bash
```

---

## 2. Workspace & Repository Layout

Create and initialize a ROS 2 workspace (if you don't already have one):
```bash
mkdir -p ~/ros2_swarm_ws/src
cd ~/ros2_swarm_ws
colcon build
source install/setup.bash
```

Clone this repo into `src`:
```bash
cd ~/ros2_swarm_ws/src
```

Expected directory layout
```text
ros2_swarm_ws/
  build/
  install/
  log/
  src/
    swarm_sim/
      worlds/
        uav5_swarm.sdf         # 5-drone Gazebo world (provided)
        uav_swarm_5_outdoor.sdf    #5-drone in an outdoor environment
    swarm_control/
      package.xml
      setup.py
      swarm_control/
        __init__.py
        swarm_controller.py    # swarm controller node
        swarm_logic.py
        update_params.py
        params.json
  uav_swarm_bridge.yaml        # ROS ↔ Gazebo bridge config
```

---

## 3. Gazebo World: 5 Multicopter UAVs

The world file `uav_swarm_5_outdoor.sdf` should contain:

* An outdoor environment etc.
* 5 multicopter models: `UAV_1`, `UAV_2`, `UAV_3`, `UAV_4`, `UAV_5`
* For each UAV:
  * **Velocity control plugin** listening on `/UAV_i/gazebo/command/twist`
  * **OdometryPublisher** plugin publishing `/model/UAV_i/odometry`

You don't have to edit this file if you use the one provided in the repo.

---

## 4. Build the Workspace

From the workspace root:
```bash
cd ~/ros2_swarm_ws
colcon build
source install/setup.bash
```

Remember to `source install/setup.bash` in any new terminal where you run ROS 2 commands.

---

## 5. Running the Full System

You'll use **four terminals**.

### Terminal 1 – Gazebo Sim with 5 UAVs
```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_swarm_ws
gz sim -r src/swarm_sim/worlds/uav_swarm_5_outdoor.sdf
```

In the Gazebo GUI, click **Play** ▶ so the simulation time runs.

### Terminal 2 – ROS–Gazebo Bridge
```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_swarm_ws
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:=$HOME/ros2_swarm_ws/uav_swarm_bridge.yaml
```

You should see log lines creating bridges for each UAV's `cmd_vel` and `odom`.

You can confirm from another terminal:
```bash
ros2 topic list | grep uav_
# Expect:
/uav_1/cmd_vel
/uav_1/odom
...
/uav_5/cmd_vel
/uav_5/odom
```

### Terminal 3 – Swarm Controller
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_swarm_ws/install/setup.bash
ros2 run swarm_control swarm_controller
```

Expected behavior:

* All 5 drones take off and climb toward **5 m altitude** while awaiting waypoints.
* Their positions are available via `/uav_i/odom`.


You can watch one drone's odom:
```bash
ros2 topic echo /uav_1/odom | grep -A 3 'position'
```

### Terminal 4 - Update Swarm Mode & Waypoints
``` bash
python3 update_params.py
```
* For behavior testing enter Swarm modes: "tight", "dispersed", "relaxed", "aggressive", "chaotic"
* Add waypoint: "waypoint X Y" eg waypoint 10 10
* Clear waypoints: "clear"

---

## 6. Summary

By following this README, you get:

* A **5-drone multicopter swarm** in Gazebo Sim.
* ROS 2 topics:
  * `/uav_i/odom` for state
  * `/uav_i/cmd_vel` for control
* A working **swarm_controller** node that lifts all drones to ~5 m and moves them to a given waypoint.

## TODO

* Plug in your own LLM logic that works with the existing swarm controller
* Extend the world (obstacles, goal markers).

---