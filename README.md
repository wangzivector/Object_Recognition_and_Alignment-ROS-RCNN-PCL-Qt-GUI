# Object_Recognition_and_Alignment-ROS-RCNN-PCL-Qt-GUI

### 1.Features

This catkin workspace has many features including **ROSnode, PCL process, Qt GUI, vtk widget**.
Besides, it contains **RCNN** implemented(TF in Python) with socket communication.
It can be used in pointcloud recognition and model processing task.

This project includes the following detailed processing methods:

* **feature-based method**

  ├─ pointcloud preprocess
  
  ├─ keypoint extract
  
  ├─ segmentation
  
  ├─ normal estimation
  
  ├─ feature compute
  
  ├─ surface reconstruct -- *MLS
  
  ├─ alignment -- *RANSAC *SACIA
  
  ├─ registration -- *NDT *ICP
  
  ├─ diff feature same reco -- *shot352 *fpfh

  ├─using Registration to get pose

  

* **RCNN-based method**

  ├─RCNN for segmentation

  ├─using 2D segmentation to extract 3D pointcloud models (RGB mapping to Depth)

  ├─using Registration to get pose

  

**More detailed software description can be found in [software.md](./software.md)**

### 2.Functions

Performance of two methods to get the object pose or instance segmentation is shown below.

* **Feature-based method <hand craft descriptor>**

<div align="center"> <img src="./assets/pics/c.png" height="220" /><img src="./assets/pics/b.png" height="220" /></div>

Red pointclouds are **models** pre-processed using [CloudCompare](http://cloudcompare.org/) software. each of **The models** you want to get pose just need to get a model file in advance--You are **free from making a large dataset.** Once you get the model file, you can use it to **align the object** in the scene to **get** **its pose**.

However, this way needs great efforts to pre-process the scene to get the partial pointcloud belonging to the model object(from left to right). So I rather use Mask-RCNN to do the recognition and segmentation works. Also this method is not the robust one.

* **DNN-based method<Mask-RCNN>**

<div align="center"> <img src="./assets/pics/d.png" height="220" /><img src="./assets/pics/i.png" height="220" /></div>

Once I got the pointcloud(left) and image(right), I can implement RCNN in image and get the **2D segmentation** of known objects. (this can be done easily by famous image dataset. In other words, You are still **free from making 3D pointcloud dataset.**) **More significantly, you can make your own 3D pointcloud dataset using 2D image dataset** as long as you can tolerate the 2D image segmentation error.

The performance of Mask-RCNN-based dataset are pictured below.

<div align="center"> <img src="./assets/pics/h.png" height="220" /><img src="./assets/pics/j.png" height="220" /></div>

 Some outlier points are still been seen in left image, but it can be removed easily using traditional pointcloud cluster methods.

* **More over**

This project is for high robotic task. It has **ROS and Socket** Interface to interact with **DL framework and Robotics Simulation.**

### 3.Basic Steps

* main dependencies:
```
ROS-kinetic
PCL-1.71
Qt5
VTK6.2
CMake5<optional>

tensorflow-1.15.0
requirements.txt(/src/qt_ros_pcl/scripts/requirements.txt)
```

* Here are some basic instructions for implement.

```
git clone https://github.com/wangzivector/object_detection-Qt-PCL-ROS-GUI.git

cd catkin_qtws
catkin_make 
source ./devel/setup.sh

///start RCNN script in ./src/qt_ros_pcl/scripts/socket_tf_master.py 
///this script can bind a socket at address: 127.0.0.1 in port 6666(check script for detailed info)

python3 ./src/qt_ros_pcl/scripts/socket_tf_master.py

roscore 
rosrun qt_ros_pcl gui_node

/// this are not all step, you need to add more based on the system information.
/// As a project you have better check the code for learning, rather than just use it.
/// cause this project is essentially just for experiment.
```

* I recommend that you open and modify this project by Qtcreator-ros.
(the installation and configuration of qtcreator-ros are list in my other repository named **instructions_learning**.)

### 4.RCNN Related
the pretrained model and weight are listed in the repo sahalaan/VidaVSMilk-using-Mask-RCNN (train rcnn using dataset <milk_box> and <tissue_package>) 
detailed info please check this [repository](https://github.com/sahalaan/VidaVSMilk-using-Mask-RCNN) and [python script](/src/qt_ros_pcl/scripts/socket_tf_master.py)

### 5.Notes

This repository is out of development. If there is any issue please inform it. 
