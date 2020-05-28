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
  this->viewer->setBackgroundColor(224, 224, 224);
  this->viewer->addCoordinateSystem();
  //  this->viewer->setCameraPosition(-0.825088, 0.568006, -0.633452, 0.322216,
  //                                  -0.780572, -0.535616);
  std::string camera_filename =
      "/home/wang/catkin_qtws/src/qt_ros_pcl/include/camera.config";
  //  viewer->saveCameraParameters(camera_filename);
  viewer->loadCameraParameters(camera_filename);
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
  cloud->points.resize(1000);

  /// The default color
  uint8_t red = 0;
  uint8_t green = 0;
  uint8_t blue = 0;

  /// Fill the cloud with some points
  for (std::size_t i = 0; i < cloud->points.size(); ++i)
  {
    cloud->points[i].x = 1 * (rand() / (RAND_MAX + 1.0f));
    cloud->points[i].y = 1 * (rand() / (RAND_MAX + 1.0f));
    cloud->points[i].z = 1 * (rand() / (RAND_MAX + 1.0f));

    cloud->points[i].r = red + uint8_t(255 * (rand() / (RAND_MAX + 1.0f)));
    cloud->points[i].g = green + uint8_t(255 * (rand() / (RAND_MAX + 1.0f)));
    cloud->points[i].b = blue + uint8_t(255 * (rand() / (RAND_MAX + 1.0f)));
  }

  //  viewer->resetCamera();
  showPointCloud(cloud, "cloud");
  setPointCloudProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                          "cloud");
  this->update();
}

qvtk::~qvtk()
{
  cloud.reset(new PointCloudT);
  viewer->close();
}

//===================================================
//  vtkRemovePointCloud
//  remove all pointclouds
//===================================================
bool qvtk::vtkRemovePointCloud(QString cloud_name, bool all)
{
  if(!all)
    return viewer->removePointCloud(cloud_name.toStdString().c_str());
  else return viewer->removeAllPointClouds();
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
  if(property_name != 4)
  viewer->setPointCloudRenderingProperties(property_name, property_value,
                                           pc_name.toStdString().c_str());
  else
    viewer->setPointCloudRenderingProperties(property_name, property_value, 0, 0,
                                             pc_name.toStdString().c_str());
}

//===================================================
//  showPointNormal
//  for pointxyzrgbnomal type visualization
//===================================================
bool qvtk::showPointNormal(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                           QString cloud_name)
{
  viewer->removeAllPointClouds();
  viewer->addPointCloudNormals<pcl::PointXYZRGBNormal>(
      cloud, 40, 0.03f, cloud_name.toStdString().c_str(), 0);
}


//===================================================
//  addPlotterExample
//  shot352 feature visualization in histogram
//===================================================
void qvtk::addPlotterExample(pcl::PointCloud<pcl::SHOT352>::Ptr descri_shot352, std::string name)
{
  pcl::visualization::PCLPlotter plotter;
  for(int i = 0; i<descri_shot352->size(); i+=100)
  {
    plotter.clearPlots();
    plotter.setTitle((QString(name.c_str()) + QString::number(i)).toStdString().c_str());
    plotter.addFeatureHistogram(*descri_shot352, "shot", i);
    plotter.spinOnce(100);
  }
}


//===================================================
//  addPlotterExample
//  fpfh feature visualization in histogram
//  in flash form
//===================================================
void qvtk::addPlotterExample(pcl::PointCloud<pcl::FPFHSignature33>::Ptr descri_fpfh, std::string name)
{
  pcl::visualization::PCLPlotter plotter;
  for(int i = 0; i<descri_fpfh->size(); i+=100)
  {
    plotter.clearPlots();
    plotter.setTitle((QString(name.c_str()) + QString::number(i)).toStdString().c_str());
    plotter.addFeatureHistogram(*descri_fpfh, "fpfh", i);
    plotter.spinOnce(100);
  }
}
