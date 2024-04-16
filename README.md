### Franka Cartesian Impedence Controller
First, we need to install `libfranka` to communicate with the robot. The library is publicly available and can be installed in the `$HOME` directory using the following commands:

```
cd $HOME
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
git clone --recursive https://github.com/frankaemika/libfranka
cd libfranka
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

In the next step, we install franka ros (we assume that you have set up a real-time kernel with ROS) for communication with the robot:

```
cd $HOME
mkdir -p catkin_ws/src
cd catkin_ws
source /opt/ros/<ros-distro>/setup.sh
catkin_init_workspace src
git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros
rosdep install --from-paths src --ignore-src --rosdistro <ros-distro> -y --skip-keys libfranka
source devel/setup.sh
```

Finally, we can install the custom controller used by the Platonics in the Robothon 2023 using the following:

```
cd src/franka_ros
git clone https://github.com/platonics-delft/franka_robothon_controllers.git
cd ..
source /opt/ros/<ros-distro>/setup.bash
catkin build -DMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build
```
You can now run the controller using:

```
roslaunch franka_robothon_controllers cartesian_variable_impedance_controller.launch robot_ip:=ROBOT_IP load_gripper:=True
```

To run the gazebo simulation:

- Open a terminal, in every terminal: 
  
```
source devel/setup.bash

roscd franka_robothon_controllers

python3 setup_gazebo.py

roslaunch franka_gazebo panda.launch x:=-0.5 world:=$(rospack find franka_gazebo)/world/stone.sdf controller:=cartesian_variable_impedance_controller rviz:=true```
