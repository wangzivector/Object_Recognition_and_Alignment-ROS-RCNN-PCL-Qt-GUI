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
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class vtkRenderer;
class vtkRenderWindowInteractor;

// class of pointcloud visualize with QVTKWidget
class qvtk : public QVTKWidget
{
  Q_OBJECT
public:
  explicit qvtk(QWidget* parent = nullptr);

  // an example for show QVTKWidget works with pcl and qt.
  void addPointCloudExample();

  //update the pointcloud in Qvtk widget
  bool vtkUpdatePointCloud(PointCloudT::Ptr new_pointcloud);

  vtkRenderer* ren1; // pointer of vtk visualizers render
  vtkRenderWindowInteractor* iren;
  pcl::visualization::PCLVisualizer::Ptr viewer;
  PointCloudT::Ptr cloud;

protected:
signals:

public slots:
};

#endif // QVTK_H
