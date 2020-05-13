/*
 * qvtk.cpp
 *
 *  Created on: May 8, 2020
 *      Author: wangzivector
 * This cppfile is for Qvtk widget visualizer
 */

#include "qvtk.h"

//===================================================
//  NDTRegistration
//  Normal Distributions Transform to get position
//  recognition to get the matrix transform the model
//===================================================
qvtk::qvtk(QWidget* parent) : QVTKWidget(parent)
{
  //
  // create a viewer and set up render stuff
  //
  viewer.reset(new pcl::visualization::PCLVisualizer(
      "viewer", false)); // setup viewer for pcl loader
  this->SetRenderWindow(this->viewer->getRenderWindow());
  this->viewer->setupInteractor(this->GetInteractor(), this->GetRenderWindow());
  this->viewer->setBackgroundColor(0, 0, 0);
  this->viewer->addCoordinateSystem();
}

//===================================================
//  addPointCloudExample
// an example for testing pcl and qt vision
//===================================================
void qvtk::addPointCloudExample()
{
  /// Setup the cloud pointer
  cloud.reset(new PointCloudT);
  /// The number of points in the cloud
  cloud->points.resize(200);

  /// The default color
  uint8_t red = 0;
  uint8_t green = 0;
  uint8_t blue = 0;

  /// Fill the cloud with some points
  for (std::size_t i = 0; i < cloud->points.size(); ++i)
  {
    cloud->points[i].x = 1024 * (rand() / (RAND_MAX + 1.0f));
    cloud->points[i].y = 1024 * (rand() / (RAND_MAX + 1.0f));
    cloud->points[i].z = 1024 * (rand() / (RAND_MAX + 1.0f));

//    cloud->points[i].r = red + uint8_t(255 * i / (cloud->points.size()));
//    cloud->points[i].g = green + uint8_t(255 * i / (cloud->points.size()));
//    cloud->points[i].b = blue + uint8_t(255 * i / (cloud->points.size()));
//    cloud->points[i].a = 255;
    cloud->points[i].r = red + uint8_t(255 * (rand() / (RAND_MAX + 1.0f)));
    cloud->points[i].g = green + uint8_t(255 * (rand() / (RAND_MAX + 1.0f)));
    cloud->points[i].b = blue + uint8_t(255 * (rand() / (RAND_MAX + 1.0f)));
  }

  showPointCloud(cloud, "cloud");
  setPointCloudProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                          "cloud");
  viewer->resetCamera();
  this->update();
}

qvtk::~qvtk()
{
  cloud.reset(new PointCloudT);
  viewer->close();
}

//===================================================
//  vtkAddPointCloud
//===================================================
inline bool qvtk::vtkAddPointCloud(const PointCloudT::Ptr new_pointcloud,
                                   QString cloud_name)
{
  return viewer->addPointCloud<PointT>(new_pointcloud,
                                       cloud_name.toStdString().c_str());
}

//===================================================
//  vtkUpdatePointCloud
//===================================================
inline bool qvtk::vtkUpdatePointCloud(const PointCloudT::Ptr pointcloud,
                                      QString cloud_name)
{
  return viewer->updatePointCloud(pointcloud, cloud_name.toStdString().c_str());
}

//===================================================
//  vtkRemovePointCloud
//===================================================
inline bool qvtk::vtkRemovePointCloud(QString cloud_name)
{
  return viewer->removePointCloud(cloud_name.toStdString().c_str());
}

//===================================================
//  showPointCloud
//  you can use it if you want to show a pointcloud
//  no matter add before or not.
//===================================================
bool qvtk::showPointCloud(const PointCloudT::Ptr pointcloud, QString cloud_name)
{
  if (!vtkUpdatePointCloud(pointcloud,
                           cloud_name)) /// if return false, dont exist yet
    return vtkAddPointCloud(pointcloud, cloud_name);
  return true;
}

//===================================================
//  setPointCloudProperties
//  set PointCloud Properties for the follow
//  based on pc name
//  PCL_VISUALIZER_POINT_SIZE
//  PCL_VISUALIZER_OPACITY
//  PCL_VISUALIZER_LINE_WIDTH
//  PCL_VISUALIZER_FONT_SIZE
//  PCL_VISUALIZER_COLOR
//===================================================
bool qvtk::setPointCloudProperties(
    pcl::visualization::RenderingProperties property_name, int property_value,
    QString pc_name)
{
  viewer->setPointCloudRenderingProperties(property_name, property_value,
                                           pc_name.toStdString().c_str());
}
