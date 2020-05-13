/*
 * qvtk.h
 *
 *  Created on: May 8, 2020
 *      Author: wangzivector
 * This headfile is for Qvtk widget visualizer
 */

#ifndef QVTK_H
#define QVTK_H

/// Qt
#include <QVTKWidget.h>
#include <QWidget>

/// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

/// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


namespace Ui
{
  class qvtk;
}

// class of pointcloud visualize with QVTKWidget
class qvtk : public QVTKWidget
{
  Q_OBJECT
public:
  explicit qvtk(QWidget* parent = nullptr);
  ~qvtk();

  // an example for show QVTKWidget works with pcl and qt.
  void addPointCloudExample();

  // basic pointcloud process in Qvtk widget
  bool vtkUpdatePointCloud(PointCloudT::Ptr pointcloud, QString cloud_name);
  bool vtkAddPointCloud(PointCloudT::Ptr new_pointcloud, QString cloud_name);
  bool vtkRemovePointCloud(QString cloud_name);
  bool showPointCloud(const PointCloudT::Ptr pointcloud, QString cloud_name);
  bool
  setPointCloudProperties(pcl::visualization::RenderingProperties property_name,
                          int property_value, QString pc_name);

protected:
  pcl::visualization::PCLVisualizer::Ptr viewer;
  PointCloudT::Ptr cloud;
signals:

public slots:
};

#endif // QVTK_H
