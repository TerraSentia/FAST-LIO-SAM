# FAST-LIO-SAM
This repository is a ROS2 version of the original ROS1 [FAST-LIO-SAM](https://github.com/engcang/FAST-LIO-SAM)

This README provides details about running fast_lio_sam node and its other dependencies.


## Dependencies
1. Ubuntu 22.04
2. ROS2 Humble
3. Make sure that all the dependencies mentioned in the [README_ROS1.md](./README_ROS1.md) document have been installed on the system. 

## How to run (Using Source Code)
1. Open a terminal and navigate to the [workspace](./docker_suite_intel_x86_64_amd64/FAST-LIO-SAM) containing the `fast_lio_sam` package. 
2. Build the package using - `colcon build --packages-select fast_lio_sam fast_lio`, the fast_lio package should be present in the third_party directory of the repo. 
3. Source the install space in the terminal - `source ./install/setup.bash`
4. Run the command `ros2 launch fast_lio_sam run_lio_all.launch.py sam_rviz:=true lio_config_file:=avia.yaml sam_delay:=5`
5. In another terminal, source the install space and run the respective rosbag used for testing. 
6. Both the launch files mentioned above should launch a rviz window in which you can set up for visualisation purposes. 

## How to run (Using Docker)
1. Open a terminal and navigate to the directory `docker_suite_intel_x86_64_amd64`. 
2. Set docker permissions to not require sudo as follows (for convenience):
3. ` sudo groupadd docker`
4. ` sudo usermod -aG docker $USER`
5. Adjust configs as per wish in the `fast_lio_sam_config` folder
6. Run `docker build -t fast_lio_sam:devel_ros2 .` to build the image
7. If need to visualize in rviz in the machine that the stack is running, `export DISPLAY=0`
8. If need to visualize in rviz in the machine that the stack is running, `xhost +local:docker`
9. Edit the command in docker compose file based on whether you want rviz:=true or false
10. `docker compose up -d`
11. If `save_pose_yml` is `true`, The latest pose should be saved in `fast_lio_sam_config/latest_lidar_pose.yaml`
12. A backup of the same will be saved in `fast_lio_sam_config/latest_lidar_pose_bkp.yaml`


