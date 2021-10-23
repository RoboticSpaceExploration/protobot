Source code for Protobot. This directory is currently using ROS 1 (Noetic) tentatively

Some dependency installation may be required, especially for ORB_SLAM2

# Setup
1. Go to root directory of the repository and run ```catkin_make```
2. Go to src/orb_slam_2 and run ```git submodule init``` and then ```git submodule update --recursive```
3. Go back to root directory and rerun ```catkin_make```

# Launch 
You can only run the simulation for now, to do that first
1. (If first time running in the shell) Go to the root folder and do ```. devel/setup.sh```
2. To launch, ```roslaunch protobot_bringup protobot_simulation.launch```
