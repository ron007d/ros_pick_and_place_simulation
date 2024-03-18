
<h1 align="center">Pick and place Demo</h1>
<h2 align="center">ROS, GAZEBO & OPENCV</h2>

![](./gif/pick_place_demo.gif)

### Requirements
- Install opencv for python
```bash
pip install opencv-python pandas
```
- install scikit learn for regression model
```bash
pip install -U scikit-learn
```
- install catkin tools for building
```bash
sudo apt install python3-catkin-tools ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-joint-trajectory-controller
```
- install extra for patches to work
```bash
sudo apt install \
    ros-$ROS_DISTRO-gazebo-ros \
    ros-$ROS_DISTRO-eigen-conversions \
    ros-$ROS_DISTRO-object-recognition-msgs \
    ros-$ROS_DISTRO-roslint
```
> Currently this has been tested on ros-noetic

### How to run
1. Clone this repo with submodules
```bash
git clone https://github.com/ron007d/ros_pick_and_place_simulation.git --recurse-submodules
```
2. Build the package
```bash
catkin build
```
3. add the sources of this folder
```bash
source devel/setup.bash
```
4. run the simulation with live video feed
```bash
roslaunch simulation_moveit_config demo_gazebo.launch
```
5. run the pick and place node
```bash
rosrun pick_and_place pick_and_place.py
```


## 👍 More information will be added soon

- Python-based framework using OpenCV and ROS (control)
- Identifies objects with OpenCV (potential enhancements)
- Employs diverse grasping strategies
- ROS & MoveIt for collision-free motion planning
- Modular design for extensibility
- Configurable parameters for easy experimentation
- Leverages Gazebo for high-fidelity simulation
- Logging & analysis for performance optimization
- Containerization potential for sharing & deployment