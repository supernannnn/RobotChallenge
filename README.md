# Citation

In this work, a portion of the visualization module from [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) was cited as a foundation. The algorithmic part of Minimumsnap was cited in this [work](https://github.com/Mesywang/MinimumSnap-Trajectory-Generation), and further extensions were made based on these two components.

# Highly Recommend Using VScode to Get Started Quickly

I am using the Ubuntu 16.04 operating system and the ROS Kinetic environment for compilation. Other environments have not been tested yet, so it is recommended to use the same compilation environment to reduce potential bugs.

```bash
git clone https://github.com/supernannnn/RobotChallenge.git
cd RobotChallenge
catkin_make -j2 -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes  #You can use "-j4" or "-j8" depending on your computer's performance
source devel/setup.bash
roslaunch minimumsnap rviz.launch & roslaunch minimumsnap fly.launch
```

When using VScode to read code, you need to install the ROS and C/C++ extensions. After that, you can add the following configuration to the `.vscode/c_cpp_properties.json` :

```json
"compileCommands": "${workspaceFolder}/build/compile_commands.json"
```

Then you can enjoy reading the code and use some common navigation operations to read more efficiently.