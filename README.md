
# waypoint_manager

A package for waypoint navigation using Nav2, enabling robots to autonomously follow predefined waypoints.

## 1. Features

- Read a specified waypoints `.csv` file

- Navigation along predefined waypoints

- Saving waypoints

- Waypoint Skip Function

- Event Flag Function

## 2. Usage Guide

### 2.1 Clone the Repository

  ```sh
  git clone https://github.com/kzm784/waypoint_manager.git
  ```

### 2.2 Install Dependencies

[ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) (Has not been tested with other ROS versions)

```sh
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

### 2.3 Build

```sh
cd /path/to/your/ros2_ws/
colcon build --symlink-install
```

### 2.4 Launch the waypoint_manager 

```sh
source install/setup.sh
ros2 launch waypoint_manager waypoint_manager.launch.py
```

## 3. Parameter Description

You can configure the node's parameters by modifying the `waypoint_manager/waypoint_manager.yaml` file.

  ```yaml
  waypoint_manager_node:
    ros__parameters:
      # Set waypoints_csv file in the launch file
      waypoints_csv: ''
      loop_enable: True
      loop_count: 2
  
  waypoint_visualizer_node:
    ros__parameters:
      # Set waypoints_csv file in the launch file
      waypoints_csv: ''
      map_frame: 'map'
  
  waypoint_skipper_node:
    ros__parameters:
      # Set waypoints_csv file in the launch file
      waypoints_csv: ''
      waypoint_skip_tolerance: 1.0
      skip_scan_count: 5
  ```
