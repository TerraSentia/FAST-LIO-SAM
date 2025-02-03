# FAST-LIO-SAM
This README provides details about running fast_lio_sam node and its other dependencies.


## Dependencies
Make sure that all the dependencies mentioned in the [README.md](./README.md) document have been installed on the system. 

## How to run
1. Open a terminal and navigate to the workspace containing the fast_lio_sam package. 
2. Build the package using - `colcon build --packages-select fast_lio_sam fast_lio`, the fast_lio package should be present in the third_party directory of the repo. 
3. Source the install space in the terminal - `source ./install/setup.bash`
4. Run the command `ros2 launch fast_lio mapping.launch.py config_file:=avia.yaml`
5. In another terminal, source the install space as done in step 3
6. Run the command `ros2 launch fast_lio_sam run_fast_lio_sam.launch.py lidar:=livox`
7. In another terminal, source the install space and run the respective rosbag used for testing. 
8. Both the launch files mentioned above should launch a rviz window in which you can set up for visualisation purposes. 


