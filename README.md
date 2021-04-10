# Robot simulation

ROS project for simulation of a robot on a terrain with obstacles

## Setting up the environment
### Set global variables
```bash
nano ~/.bashrc
# add in the end
export TURTLEBOT3_MODEL=burger
# Press ctrl + x
# Press y
# Press Enter
```
### Adding lidar sensor
```bash
cp -r velodyne_hdl32 ~/.gazebo/models
cd velodyne_plugin
mkdir build 
cd build 
cmake ..
make
cp sudo cp libvelodyne_plugin.so /opt/ros/noetic/lib/
```

For tests (make it from build folder):
1. Start `roscore`
```bash
source /opt/ros/noetic/setup.bash
roscore
```
2. In a new terminal, start Gazebo
```bash
source /opt/ros/noetic/setup.bash
gazebo ../velodyne.world
```
3. In a new terminal, use `rostopic` to send a velocity message.
```bash
source /opt/ros/noetic/setup.bash
rostopic pub /my_velodyne/vel_cmd std_msgs/Float32 1.0
```
4. Change the last number of the above command to set different velocities.

### Additional software and libraries

1. `nlohmann-json`