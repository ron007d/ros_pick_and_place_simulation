<center>
    <h1>Pick and place Demo</h1>
    <h2>ROS, GAZEBO & OPENCV</h2>
</center>

![](./gif/pick_place_demo.gif)

### Requirements
- Install opencv for python
```bash
pip install opencv-python
```
- install scikit learn for regression model
```bash
pip install -U scikit-learn
```

### How to run
1. Clone this repo with submodules
```bash
    git clone https://github.com/ron007/ros_pick_and_place_simulation.git --recurse-submodules
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