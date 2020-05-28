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
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/histogram_visualizer.h>

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

  //
  // manipulate pointcloud in vtk widget]
  //
  void addPlotterExample(pcl::PointCloud<pcl::SHOT352>::Ptr descri_sho352, std::string name);
  void addPlotterExample(pcl::PointCloud<pcl::FPFHSignature33>::Ptr descri_fpfh, std::string name);

  bool
  setPointCloudProperties(pcl::visualization::RenderingProperties property_name,
                          int property_value, QString pc_name);

  bool showPointNormal(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, QString cloud_name);

  bool vtkRemovePointCloud(QString cloud_name, bool all = false);

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
    if (!vtkUpdatePointCloud(pointcloud, cloud_name))
       vtkAddPointCloud(pointcloud, cloud_name);
    setPointCloudProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                            cloud_name);
    if(cloud_name.contains("object", Qt::CaseSensitive)){
      setPointCloudProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 3,
                              cloud_name);
      setPointCloudProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                              cloud_name);
    }
    this->update();
    return true;
  }

protected:
  pcl::visualization::PCLVisualizer::Ptr viewer;
  PointCloudT::Ptr cloud;

};

#endif // QVTK_H
