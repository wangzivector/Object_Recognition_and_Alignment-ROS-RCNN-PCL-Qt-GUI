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

typedef pcl::PointXYZRGB PointT;
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

  /// an example for show QVTKWidget works with pcl and qt.
  void addPointCloudExample();

  bool
  setPointCloudProperties(pcl::visualization::RenderingProperties property_name,
                          int property_value, QString pc_name);

  bool showPointNormal(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, QString cloud_name);

  /// basic pointcloud process in Qvtk widget
  template <typename CloudType>
  inline bool vtkAddPointCloud(const CloudType new_pointcloud,
                               QString cloud_name)
  {
    return viewer->addPointCloud(new_pointcloud,
                                 cloud_name.toStdString().c_str());
  }
  template <typename CloudType>
  inline bool vtkUpdatePointCloud(const CloudType pointcloud,
                                  QString cloud_name)
  {
    return viewer->updatePointCloud(pointcloud,
                                    cloud_name.toStdString().c_str());
  }
  bool vtkRemovePointCloud(QString cloud_name);

  //===================================================
  //  showPointCloud
  //  AS THIS IS TEMPLATE, DEFINE IT WITH DECLARATION
  //  you can use it if you want to show a pointcloud
  //  no matter add before or not.
  //===================================================
    template <typename Type>
  bool showPointCloud(Type pointcloud, QString cloud_name)
  {
    if (pointcloud->size() == 0)
      return false;

    /// if return false, dont exist yet
    if (vtkUpdatePointCloud(pointcloud, cloud_name))
      this->update();
    else if (vtkAddPointCloud(pointcloud, cloud_name))
      this->update();
    return true;
  }

protected:
  pcl::visualization::PCLVisualizer::Ptr viewer;
  PointCloudT::Ptr cloud;
signals:

public slots:
};

#endif // QVTK_H
