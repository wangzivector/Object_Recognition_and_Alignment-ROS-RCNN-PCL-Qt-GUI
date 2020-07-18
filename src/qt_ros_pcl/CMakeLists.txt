cmake_minimum_required(VERSION 2.8.3)
project(qt_ros_pcl)

 add_compile_options(-std=c++11)

#find_package(catkin REQUIRED COMPONENTS
#  roscpp
#)

#catkin_package(
##  INCLUDE_DIRS include
##  LIBRARIES qt_ros_pcl
##  CATKIN_DEPENDS roscpp
##  DEPENDS system_lib
#)

#include_directories(
## include
#  ${catkin_INCLUDE_DIRS}
#)

# add_executable(${PROJECT_NAME}_node src/basic.cpp)


# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
# 找到qt5中的widgets组件的关键步骤，使程序编译时能够找到qt5-widgets相关头文件和库函数。
find_package(Qt5  REQUIRED COMPONENTS  Widgets)

find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7.1 REQUIRED)
find_package(realsense2 2.29.0)


# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 指明需要参与编译的cpp及ui文件，分别给它们起个变量名叫SOURCES和FORMS，后续会用到。
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES
  gui/mainwindow.cpp
  gui/gui_node.cpp
  gui/qvtk.cpp
  src/qpcl.cpp
  src/qros.cpp
  src/pcd_io.cpp
  src/objreco.cpp
  src/qsocket.cpp
)
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})

set(FORMS
  gui/mainwindow.ui
)

add_message_files(FILES	info.msg)
generate_messages(DEPENDENCIES	std_msgs)

catkin_package()

include_directories( include  ${catkin_INCLUDE_DIRS} )
# 这是生成可执行文件，此时用到了SOURCES和FORMS所指明的cpp和ui文件。
 add_executable(gui_node ${SOURCES} ${FORMS} ${QT_RESOURCES_CPP})
 add_executable(${PROJECT_NAME}_node  src/basic.cpp)
#在链接环节，告诉目标文件需要qt5的widgets组件作为动态链接库。
 target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} )
 target_link_libraries(gui_node  Qt5::Widgets ${catkin_LIBRARIES} ${PCL_LIBRARIES} realsense2 ${OpenCV_LIBRARIES})
