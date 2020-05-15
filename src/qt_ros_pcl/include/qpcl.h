/*
 * qpcl.h
 *
 *  Created on: May 8, 2020
 *      Author: wangzivector
 * pointcloud process in this file, including create/read downsample and so on
 */

#ifndef QPCL_H
#define QPCL_H
/// pointcloud headfile
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

/// z filter/grid/outliner/plane/background
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/segmentation/sac_segmentation.h>

/// sample
#include <pcl/keypoints/uniform_sampling.h>

/// segmentation
#include <pcl/segmentation/region_growing_rgb.h>

typedef pcl::PointXYZRGB PointType;            //接收点云的格式
typedef pcl::Normal NormalType;                //点云法向量格式
typedef pcl::PointCloud<PointType> PointCloud; //接收点云的”存储“格式
typedef pcl::PointCloud<NormalType> NormalCloud; //接收点云法向量存储格式

class qpcl
{
public:
  qpcl();

  //
  // filters
  //
  bool axisFilter(PointCloud::Ptr cloud, PointCloud::Ptr cloud_axis,
                  const float axis_size, bool keep_organized);
  bool gridFilter(PointCloud::Ptr cloud, PointCloud::Ptr cloud_grid,
                  float grid_size);
  bool planeFilter(PointCloud::Ptr cloud, PointCloud::Ptr cloud_extract,
                   double threshold_plane, bool keep_organized);
  bool outlierFilter(PointCloud::Ptr cloud, PointCloud::Ptr cloud_outlier,
                     int outlier_meanK, double outlier_Thresh);
  bool backgroundFilter(PointCloud::Ptr cloud_g, PointCloud::Ptr cloud_extract,
                        PointCloud::Ptr cloud_i, int noise_filter,
                        double resolution);
  bool downSample(PointCloud::Ptr cloud,
                  pcl::PointCloud<int>::Ptr sampled_indices_cloud,
                  double sample_radius);

  bool rgbSegmentationIndex(
      PointCloud::Ptr cloud, pcl::PointCloud<int>::Ptr sampled_indices_cloud,
      std::vector<pcl::PointIndices>* clusters_rgb_deliver, float dist_thre,
      float point_color_thre, float region_color_thre, int min_cluster);

  bool rgbSegmentation(PointCloud::Ptr cloud,
                       std::vector<pcl::PointIndices>* clusters_rgb,
                       float dist_thre, float point_color_thre,
                       float region_color_thre, int min_cluster);
};

#endif // QPCL_H
