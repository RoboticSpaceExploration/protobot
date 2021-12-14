Source code for Protobot. This directory is currently using ROS 1 (Noetic) tentatively

# Dependencies
1. Install _DEPENDENCIES_ for ORB_SLAM2 https://github.com/appliedAI-Initiative/orb_slam_2_ros/tree/master/orb_slam2
2. Install librealsense2 with dev and dbg distribution included https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages
3. Also ensure that you have the following(Install with ```sudo apt-get install```): 
     - libpcl-dev
     - libeigen3-dev
     - ros-noetic-costmap2d
     - ros-noetic-ddynamic-reconfigure

# Setup
1. Go to src/ and run ```git submodule update --init --recursive```
2. Go back to the root directory and run ```rosdep init``` and then ```rosdep install --from-paths src --ignore-src -r -y```
3. Build your project using ```catkin_make -DCMAKE_BUILD_TYPE=Release```
4. It is recommended to automatically source the project inside of your ~/.bashrc file. Put the following at the end of ~/.bashrc: ```source <cloned directory root path>/devel/setup.sh ```

# Launch 
To run simulation
1. Source project at root directory using ```. devel/setup.sh``` when first opening the shell. If you don't want to do this everytime, refer to #4
2. To launch, ```roslaunch protobot_bringup protobot_simulation.launch```

To run on live rover
1. On live rover, ssh into it and run ```roslaunch protobot_bringup protobot_onboard.launch``` (Autnomous) OR ```roslaunch protobot_bringup protobot_teleop.launch``` (Teleop Mode)
1. On properly configured laptop (with correct ROS_IP and ROS_MASTER_URI pointing to own IPv4 address and rover's IPv4 address respectively), run ```roslaunch protobot_bringup protobot_remote.launch```

# Guidelines for this repository
1. Google C++ Style Guide (https://google.github.io/styleguide/cppguide.html)
2. Google Python Style Guide (https://google.github.io/styleguide/pyguide.html)
3. ROS Standardizations/Guidelines, particularly
     - REP-103 (https://www.ros.org/reps/rep-0103.html)
     - REP-105 (https://www.ros.org/reps/rep-0105.html)
     - REP-145 https://www.ros.org/reps/rep-0145.html
4. Guidelines for ROS Projects
     - https://github.com/leggedrobotics/ros_best_practices/wiki
