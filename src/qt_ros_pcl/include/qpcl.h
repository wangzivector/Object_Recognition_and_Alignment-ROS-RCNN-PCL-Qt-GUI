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
#include <pcl/filters/approximate_voxel_grid.h>
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

/// registraction
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/sample_consensus_prerejective.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::PointXYZRGBNormal PointRGBNormalType; /// for resonstruction
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<NormalType> NormalCloud;
typedef pcl::PointCloud<PointRGBNormalType> PointRGBNormalCloud;

typedef pcl::SHOT352 SHOT352;   /// shot32
typedef pcl::SHOT1344 SHOT1344; /// shot1344 -- shot with color
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
                  const float axis_size = 0.2f, bool keep_organized = true);
  bool gridFilter(PointCloud::Ptr cloud, PointCloud::Ptr cloud_grid,
                  float grid_size = 0.002f);
  bool planeFilter(PointCloud::Ptr cloud, PointCloud::Ptr cloud_extract,
                   double threshold_plane = 0.008, bool keep_organized = true);
  bool outlierFilter(PointCloud::Ptr cloud, PointCloud::Ptr cloud_outlier,
                     int outlier_meanK = 30, double outlier_Thresh = 0.1);
  bool backgroundFilter(PointCloud::Ptr cloud_g, PointCloud::Ptr cloud_i,
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
  //
  // aligned
  //
  Eigen::Matrix4f RANSACRegistration(
      PointCloud::Ptr source_cloud_keypoint,
      DescriptorCloudFPFH::Ptr source_descriptors_FPFH,
      PointCloud::Ptr target_cloud_kepoint,
      DescriptorCloudFPFH::Ptr target_descriptors_FPFH,
      PointCloud::Ptr cloud_aligned, int max_iterations = 200,
      int number_samples = 20, int randomness = 10, float similar_thre = 0.9f,
      double max_corr_distance = 0.015, float min_sample_distance = 0.25);

  Eigen::Matrix4f RANSACRegistration(
      PointCloud::Ptr source_cloud_keypoint,
      DescriptorCloudShot352::Ptr source_descriptors_shot352,
      PointCloud::Ptr target_cloud_kepoint,
      DescriptorCloudShot352::Ptr target_descriptors_shot352,
      PointCloud::Ptr cloud_aligned, int max_iterations = 200,
      int number_samples = 20, int randomness = 10, float similar_thre = 0.9f,
      double max_corr_distance = 0.015, float min_sample_distance = 0.25);

  Eigen::Matrix4f
  SACIARegistration(PointCloud::Ptr source_cloud,
                    DescriptorCloudFPFH::Ptr source_descriptor_fpfh,
                    PointCloud::Ptr target_cloud,
                    DescriptorCloudFPFH::Ptr target_descriptor_fpfh,
                    PointCloud::Ptr cloud_aligned, int number_samples = 20,
                    int randomness = 10, float min_sample_distance = 0.25,
                    double max_correspondence_distance = 0.015,
                    int max_iterations = 200);

  Eigen::Matrix4f
  SACIARegistration(PointCloud::Ptr source_cloud,
                    DescriptorCloudShot352::Ptr source_descriptor_shot352,
                    PointCloud::Ptr target_cloud,
                    DescriptorCloudShot352::Ptr target_descriptor_shot352,
                    PointCloud::Ptr cloud_aligned, int number_samples = 20,
                    int randomness = 10, float min_sample_distance = 0.25,
                    double max_correspondence_distance = 0.015,
                    int max_iterations = 200);
  //
  // registration
  //
  Eigen::Matrix4f NDTRegistration(PointCloud::Ptr source_cloud_keypoint,
                                  PointCloud::Ptr target_cloud_keypoint,
                                  double ndt_transepsilon = 0.001,
                                  double ndt_stepsize = 0.1,
                                  float ndt_resolution = 1,
                                  int ndt_maxiteration = 20);
  Eigen::Matrix4f
  ICPRegistration(PointCloud::Ptr source_cloud, PointCloud::Ptr target_cloud,
                  PointCloud::Ptr cloud_icped, double max_corr_distance = 0.08,
                  int max_iter_icp = 100000, double transformation = 1.00,
                  double euclidean_Fitness = 0.001);
};

#endif // QPCL_H
