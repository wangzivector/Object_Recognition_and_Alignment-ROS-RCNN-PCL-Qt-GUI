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

/// estimation
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h> ///accelarate
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>

/// reconstruction
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

typedef pcl::PointXYZRGB PointType;                ///接收点云的格式
typedef pcl::Normal NormalType;                    ///点云法向量格式
typedef pcl::PointXYZRGBNormal PointRGBNormalType; /// resonstruction
typedef pcl::PointCloud<PointType> PointCloud; ///接收点云的”存储“格式
typedef pcl::PointCloud<NormalType> NormalCloud; ///接收点云法向量存储格式
typedef pcl::PointCloud<PointRGBNormalType> PointRGBNormalCloud;

typedef pcl::SHOT352 SHOT352;   /// shot32特征格式
typedef pcl::SHOT1344 SHOT1344; /// shot1344特征格式（color）
typedef pcl::FPFHSignature33 FPFH;

typedef pcl::PointCloud<SHOT352> DescriptorCloudShot352;
typedef pcl::PointCloud<SHOT1344> DescriptorCloudShot1344;
typedef pcl::PointCloud<FPFH> DescriptorCloudFPFH;

class qpcl
{
public:
  qpcl();

  //
  // filters
  //
  bool axisFilter(PointCloud::Ptr cloud, PointCloud::Ptr cloud_axis,
                  const float axis_size = 0.2f, bool keep_organized = false);
  bool gridFilter(PointCloud::Ptr cloud, PointCloud::Ptr cloud_grid,
                  float grid_size = 0.002f);
  bool planeFilter(PointCloud::Ptr cloud, PointCloud::Ptr cloud_extract,
                   double threshold_plane = 0.008, bool keep_organized = false);
  bool outlierFilter(PointCloud::Ptr cloud, PointCloud::Ptr cloud_outlier,
                     int outlier_meanK = 30, double outlier_Thresh = 0.1);
  bool backgroundFilter(PointCloud::Ptr cloud_g, PointCloud::Ptr cloud_extract,
                        PointCloud::Ptr cloud_extract, int noise_filter = 1,
                        double resolution = 0.015);
  //
  // keypoint
  //
  bool downSample(PointCloud::Ptr cloud,
                  pcl::PointCloud<int>::Ptr sampled_indices_cloud,
                  double sample_radius = 0.005);
  //
  // segmetation
  //
  bool
  rgbSegmentationIndex(PointCloud::Ptr cloud,
                       pcl::PointCloud<int>::Ptr sampled_indices_cloud,
                       std::vector<pcl::PointIndices>* clusters_rgb_deliver,
                       float dist_thre = 3, float point_color_thre = 13,
                       float region_color_thre = 30, int min_cluster = 350);

  bool rgbSegmentation(PointCloud::Ptr cloud,
                       std::vector<pcl::PointIndices>* clusters_rgb,
                       float dist_thre = 3, float point_color_thre = 13,
                       float region_color_thre = 30, int min_cluster = 350);
  //
  // surface reconstruction
  //
  bool mlsReconstruction(PointCloud::Ptr cloud,
                         PointRGBNormalCloud::Ptr cloud_pointRGBNormal,
                         double search_radius = 0.03);

  //
  // normal
  //
  bool normalEstimation(PointCloud::Ptr cloud, NormalCloud::Ptr cloud_normal);

  //
  // feature
  //
  bool shot352Estimation(PointCloud::Ptr cloud, PointCloud::Ptr cloud_keypoint,
                         NormalCloud::Ptr cloud_normal,
                         DescriptorCloudShot352::Ptr cloud_descriptors_shot352,
                         double descr_rad_352 = 0.03);
  bool
  shot1344Estimation(PointCloud::Ptr cloud, PointCloud::Ptr cloud_keypoint,
                     NormalCloud::Ptr cloud_normal,
                     DescriptorCloudShot1344::Ptr cloud_descriptors_shot1344,
                     double descr_rad_1344 = 0.03);
  bool fpfhEstimation(PointCloud::Ptr cloud, NormalCloud::Ptr cloud_normal,
                      DescriptorCloudFPFH::Ptr cloud_descriptors_FPFH);
};

#endif // QPCL_H
