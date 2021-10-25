Source code for Protobot. This directory is currently using ROS 1 (Noetic) tentatively

Some dependency installation may be required, especially for ORB_SLAM2. Refer to following for dependency install instructions: https://github.com/appliedAI-Initiative/orb_slam_2_ros/tree/master/orb_slam2

# Setup
1. Go to root directory of the repository and run ```catkin_make```
2. Go to src/orb_slam_2_ros (and any other submodules) and run ```git submodule init --update --recursive```
3. Go back to root directory and rerun ```catkin_make``` to rebuild the project with the submodules included
4. It is recommended to automatically source the project inside of your ~/.bashrc file. Put the following at the end of ~/.bashrc: ```source <cloned directory root path>/devel/setup.sh ```

# Launch 
You can only run the simulation for now, to do that first
1. Source project at root directory using ```. devel/setup.sh``` when first opening the shell. If you don't want to do this everytime, refer to #4
2. To launch, ```roslaunch protobot_bringup protobot_simulation.launch```
