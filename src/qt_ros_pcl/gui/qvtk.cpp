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
  // set up render stuff
  //
  this->ren1 = vtkRenderer::New();
  this->GetRenderWindow()->AddRenderer(ren1);
  this->iren = this->GetInteractor();

  viewer.reset(new pcl::visualization::PCLVisualizer(
      "viewer", false)); // setup viewer for pcl loader
  // Setup the cloud pointer
  cloud.reset(new PointCloudT);
  // The number of points in the cloud
  cloud->points.resize(200);
  viewer->addPointCloud(cloud, "cloud");
}

//===================================================
//  addPointCloudExample
// an example for testing pcl and qt vision
//===================================================
void qvtk::addPointCloudExample()
{
  // Setup the cloud pointer
  cloud.reset(new PointCloudT);
  // The number of points in the cloud
  cloud->points.resize(200);

  // The default color
  uint8_t red = 128;
  uint8_t green = 128;
  uint8_t blue = 128;

  // Fill the cloud with some points
  for (std::size_t i = 0; i < cloud->points.size(); ++i)
  {
    cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);

    cloud->points[i].r = red;
    cloud->points[i].g = green;
    cloud->points[i].b = blue;
  }
  vtkUpdatePointCloud(cloud);
  viewer->resetCamera();
  this->update();
}

//===================================================
//  addPointCloudExample
// an example for testing pcl and qt vision
//===================================================
inline bool qvtk::vtkUpdatePointCloud(const PointCloudT::Ptr new_pointcloud)
{
    return viewer->updatePointCloud(new_pointcloud, "cloud");
}
