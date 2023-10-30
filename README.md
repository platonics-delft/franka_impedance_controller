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

In the next step, we install the ROS-wrapper (we assume that you have set up a real-time kernel with ROS) for the communication with the robot:

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

Finally, we can install the custom controller used by the Platonics in the robothon 2023 using the following:

```
roscd franka_ros
git clone git@github.com:platonics-delft/franka_control_robothon_challenge.git
roscd
cd ..
source /opt/ros/<ros-distro>/setup.bash
catkin_make -DMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build
```

To run the simulation:
- Open a terminal, in every terminal: ```source devel/setup.bash```
```python3 setup.py```
```roslaunch franka_gazebo panda.launch x:=-0.5 world:=$(rospack find franka_gazebo)/world/stone.sdf controller:=cartesian_variable_impedance_controller rviz:=true```
You can now run the controller using:

```
roslaunch franka_human_friendly_controllers cartesian_variable_impedance_controller.launch robot_ip:=ROBOT_IP load_gripper:=True
```
