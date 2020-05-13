# object_detection-Qt-PCL-ROS-GUI
## features
This catkin workspace have many features including ROSnode, PCL process, Qt GUI, pointcloud widget.
It can be used in pointcloud recognition and model processing tasks.

* main dependencies:
```
ROS-kinetic
PCL-1.71
Qt5
VTK6.2
CMake5<optional>
```

### basic step
* Here are some basic instructions for implement.

```
git clone https://github.com/wangzivector/object_detection-Qt-PCL-ROS-GUI.git

cd catkin_qtws
catkin_make 
source ./devel/setup.sh

roscore 
rosrun qt_ros_pcl gui_node
/// this are not all step, you need to add more based on the system information.
```

* also you can open and complie this project by Qtcreator-ros
the installation and configuration of qtcreator-ros are list in my repository named instructions_learning.

## functions
* rosnode handle
* pointcloud visualizer<qvtkwidget>

### notes
This repository is still under development. If there is any issue please inform it. 
