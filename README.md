Source code for Protobot. This directory is currently using ROS 1 (Noetic) tentatively

# Dependencies
1. Follow instructions to install _DEPENDENCIES_ here https://github.com/appliedAI-Initiative/orb_slam_2_ros/tree/master/orb_slam2
2. Also ensure that you have the following(Install with ```sudo apt-get install```): 
       a. libpcl-dev
       b. libeigen3-dev
       c. ros-noetic-costmap2d

# Setup
1. Go to src/orb_slam_2_ros (and any other submodules) and run ```git submodule update --init --recursive```
2. Go back to the root directory and run ```rosdep init``` and then ```rosdep install --from-paths src --ignore-src -r -y```
3. Build your project using ```catkin_make -DCMAKE_BUILD_TYPE=Release```
4. It is recommended to automatically source the project inside of your ~/.bashrc file. Put the following at the end of ~/.bashrc: ```source <cloned directory root path>/devel/setup.sh ```

# Launch 
You can only run the simulation for now, to do that first
1. Source project at root directory using ```. devel/setup.sh``` when first opening the shell. If you don't want to do this everytime, refer to #4
2. To launch, ```roslaunch protobot_bringup protobot_simulation.launch```

# Guidelines for this repository
1. Google C++ Style Guide (https://google.github.io/styleguide/cppguide.html)
2. Google Python Style Guide (https://google.github.io/styleguide/pyguide.html)
3. ROS Standardizations/Guidelines, particularly
    a. REP-103 (https://www.ros.org/reps/rep-0103.html)
    b. REP-105 (https://www.ros.org/reps/rep-0105.html)
    c. REP-145 https://www.ros.org/reps/rep-0145.html
4. Guidelines for ROS Projects
    a. https://github.com/leggedrobotics/ros_best_practices/wiki
