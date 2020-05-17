# Software guide
## author: wangzi

1. 对象封装
(diff cpp files)

* Qtdisplay class
* Param     class
* PCL       class
* ROS       class

2. Dialog
* point cloud display window <left up big>
* detailed display window <left down small>
* param window <right up rect>
* information text display window <right down long>

3. Timeline
* setup ros_qt basic env  		<done>
* basic implement of PCL and RealSense	<done>
* Vision of point cloud			<done> -- based : QVTKwidget(PLC_qt_visualizer_tutorial)    librviz(testRVIZ)	//openGL
  thing about it, is it have to use ROS, also the rviz topic manage.
  what's the step of QVTKwidget. this should be more convenient.

* Class setup of pointcloud class       <doing>
* Param class
* GUI Interface setup
* Makeup 

4. Feature
* class orientation
* scalable<optional>
* Exception handling

5. cpp discription
* gui<done>
   ├─ pointcloud vtkwidget
   ├─ infoprint widget
   ├─ param widget(multipage)
   ├─ subview widget

* qrealsense<done>
   ├─ rs receive/config
   ├─ return frame
   ├─ pcd file read/store

* basic<done>
   ├─ publish final info in qt
   ├─ receive and printout in receive node

* qvtk<done>
   ├─ update pointcloud display
   ├─ add pointcloud display
   ├─ display config - colour visable size

* qpcl<done>
   ├─ pointcloud preprocess<done>
   ├─ keypoint extract<done>
   ├─ segmentation<done>
   ├─ normal estimation<done>
   ├─ feature compute<done>
   ├─ surface reconstruct<done>
   ├─ feature match<done>
   ├─ object detection<done>

* system
   ├─ pointcloud interface
   ├─ param editting
   ├─ error catching





