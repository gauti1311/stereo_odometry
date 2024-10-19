## Stereo Visual Odometry in ROS2

This repository is C++ OpenCV implementation of Stereo Visual Odometry with ROS2


### Requirements

- opencv
- ros2 (humble)

### Dataset
Tested on [KITTI](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) odometry dataset

### Installation 
```bash

cd ros_ws/src
git clone https://github.com/gauti1311/stereo_odometry.git
```

```bash
cd ros2_ws/ 

colcon build

source install/setup.bash
```
Odometry **Camera Parameters** in config/kitti.yaml, put your own camera parameters in the same format and pass the path when you run.

### Usage

```bash
ros2 run stereo_odometry stereo_odometry_node
```

### unsig launch file
```
ros2 launch stereo_odometry stereo_odometry.launch.py 
```

- `stereo_odometry_node` subscribes to synochronized `/left_image` and `/right_image` topics and publishes `/odom_cam` w.r.t to base_link as child link
