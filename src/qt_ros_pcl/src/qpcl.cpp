/*
 * qpcl.cpp
 *
 *  Created on: May 13, 2020
 *      Author: wangzivector
 * This cppfile mass with pointcloud basic process
 */
#include "qpcl.h"

qpcl::qpcl() {}

//===================================================
//  axisFilter
//  input pointcloud and output the inbox of axis size
//  keep_organized means keep the size of pointcloud
//  by setting its distance to infinite.
//===================================================
bool qpcl::axisFilter(PointCloud::Ptr cloud, PointCloud::Ptr cloud_axis,
                      const float axis_size = 0.2f, bool keep_organized = false)
{
  /// filter z values within a certain range set by parameters
  const float min = -axis_size;
  const float max = axis_size;

  /// set up the passthrongh filter for object save
  /// false means take the inlier instead of the outlier
  pcl::PassThrough<PointType> pt(false);

  /// set parameters for z axis filtering
  pt.setInputCloud(cloud);
  pt.setKeepOrganized(keep_organized);
  pt.setFilterFieldName("x");
  pt.setFilterLimits(min, max);
  pt.filter(*cloud_axis);
  pt.setInputCloud(cloud_axis);
  pt.setKeepOrganized(keep_organized);
  pt.setFilterFieldName("y");
  pt.setFilterLimits(min, max);
  pt.filter(*cloud_axis);
  std::cout << "after axis filter point size: " << cloud_axis->size() << endl;
  return true;
}

//===================================================
//  gridFilter
//  use for the grid /downsample
//  Create the filtering object and downsample dataset
//  using the parameter leaf size
//===================================================
bool qpcl::gridFilter(PointCloud::Ptr cloud, PointCloud::Ptr cloud_grid,
                      float grid_size = 0.002f)
{
  /// use for the grid /downsample
  pcl::VoxelGrid<PointType> sor_grid;
  /// Create the filtering object and downsample the dataset
  /// using the parameter leaf size
  sor_grid.setInputCloud(cloud);
  sor_grid.setLeafSize(grid_size, grid_size, grid_size);
  sor_grid.filter(*cloud_grid);
  std::cout << "after grid point size: " << cloud_grid->size() << endl;
  return true;
}

//===================================================
//  planeFilter
//  remove the main plane of pointcloud
//  with the max size of distance
//===================================================
bool qpcl::planeFilter(PointCloud::Ptr cloud, PointCloud::Ptr cloud_extract,
                       double threshold_plane = 0.008,
                       bool keep_organized = false)
{

  /// Declare the segmentation object for planes
  pcl::SACSegmentation<PointType> seg_plane;
  /// Declare the filtering object for planes
  pcl::ExtractIndices<PointType> extract_planes;

  /// Set up SAC parameters for plane segmentation
  seg_plane.setOptimizeCoefficients(true);
  seg_plane.setModelType(pcl::SACMODEL_PLANE);
  seg_plane.setMethodType(pcl::SAC_RANSAC);
  seg_plane.setMaxIterations(1000);
  extract_planes.setNegative(
      true); /// Extract the found plane to remove the table
  pcl::ModelCoefficients
      coefficients_plane; /// construct coefficients for plane
  pcl::PointIndices
      inliers_plane; /// constructor for point found as part of surface

  /// set maximal distance from point to surface to be identified as plane
  seg_plane.setDistanceThreshold(threshold_plane);
  /// maybe should set as the cloud_r with the remove part to keep mess
  seg_plane.setInputCloud(cloud);
  seg_plane.segment(inliers_plane, coefficients_plane);

  /// remove plane from point cloud
  extract_planes.setInputCloud(cloud);
  extract_planes.setIndices(pcl::PointIndices::Ptr(&inliers_plane));
  extract_planes.setKeepOrganized(keep_organized);

  extract_planes.filter(*cloud_extract);
  cout << "after noplane point size: " << cloud_extract->size() << endl;

  return true;
}

//===================================================
//  outlierFilter
//  remove the outlier of pointcloud
//  with mean_num in distance of outlier_Thresh
//===================================================
bool qpcl::outlierFilter(PointCloud::Ptr cloud, PointCloud::Ptr cloud_outlier,
                         int outlier_meanK = 30, double outlier_Thresh = 0.1)
{
  /// use for the outlier
  pcl::StatisticalOutlierRemoval<PointType> sor_outlier;
  sor_outlier.setInputCloud(cloud);
  sor_outlier.setMeanK(outlier_meanK);
  sor_outlier.setStddevMulThresh(outlier_Thresh);
  sor_outlier.filter(*cloud_outlier);
  std::cout << "after outlier point size: " << cloud_outlier->size() << endl;
  return true;
}

//===================================================
//  backgroundFilter - may still need modify
//  remove the outlier of pointcloud near background
//  with in distance of resolution
//  after it will apply noise_filter
//===================================================
bool qpcl::backgroundFilter(PointCloud::Ptr cloud_g, PointCloud::Ptr cloud_i,
                            PointCloud::Ptr cloud_extract, int noise_filter = 1,
                            double resolution = 0.015)
{
  pcl::octree::OctreePointCloudChangeDetector<PointType> octree(resolution);
  std::cout << "ground octree ... cloud_groud : " << cloud_g->size()
            << " resolution: " << resolution << endl;
  octree.setInputCloud(cloud_g);
  octree.addPointsFromInputCloud();
  octree.switchBuffers();
  std::cout << "compare change ... cloud_input : " << cloud_i->size() << endl;
  octree.setInputCloud(cloud_i);
  octree.addPointsFromInputCloud();
  std::vector<int> newPointIdxVector;
  /// Get vector of point indices from octree voxels which did not exist in
  /// previous buffer
  octree.getPointIndicesFromNewVoxels(newPointIdxVector, noise_filter);
  std::cout << "  newPointIdxVector : " << newPointIdxVector.size() << endl;

  if (newPointIdxVector.size() == 0)
  {
    std::cout << "background detected, ground deliver, please change input ...";
    pcl::copyPointCloud(*cloud_i, *cloud_extract);
  }
  else
  {
    cloud_extract.reset(new PointCloud);
    for (std::size_t i = 0; i < newPointIdxVector.size(); ++i)
      cloud_extract->points.push_back(cloud_i->points[newPointIdxVector[i]]);
    /// cloud_extract->width = cloud_extract->size ();
    /// cloud_extract->height = 1;
    cout << "after extra point size: " << cloud_extract->size() << endl;
  }

  return true;
}

//===================================================
//  downSample
//  downsample pointcloud with radius sample_radius
//  return as index of keypoints, if you want to get
//  the pointcloud , use the following code:
//  pcl::copyPointCloud(*cloud,
//    sampled_indices_cloud.points, *cloud_keypoints);
//  if you want to extract them one by one :
//  pcl::copyPointCloud(*cloud_descriptors_FPFH,
//  *clusters_rgb_deliver->at(current_num - 1),
//  *cloud_descriptors_FPFH);
//===================================================
bool qpcl::downSample(PointCloud::Ptr cloud,
                      pcl::PointCloud<int>::Ptr sampled_indices_cloud,
                      double sample_radius = 0.005)
{
  pcl::UniformSampling<PointType> uniform_sampling_cloud;
  uniform_sampling_cloud.setInputCloud(cloud);
  uniform_sampling_cloud.setRadiusSearch(sample_radius);
  uniform_sampling_cloud.compute(*sampled_indices_cloud);
  return true;
}

//===================================================
//  rgbSegmentationIndex
//  colour and region segmentation with keypoint index
//  if you want to extract them one by one :
//  pcl::copyPointCloud(*cloud_descriptors_FPFH,
//  *clusters_rgb_deliver->at(current_num - 1),
//  *cloud_descriptors_FPFH);
//===================================================

bool qpcl::rgbSegmentationIndex(
    PointCloud::Ptr cloud, pcl::PointCloud<int>::Ptr sampled_indices_cloud,
    std::vector<pcl::PointIndices>* clusters_rgb_deliver, float dist_thre = 3,
    float point_color_thre = 13, float region_color_thre = 30,
    int min_cluster = 350)
{
  pcl::RegionGrowingRGB<PointType> reg_segrgb;
  std::vector<pcl::PointIndices> clusters_rgb;
  pcl::search::Search<PointType>::Ptr tree_segrgb =
      boost::shared_ptr<pcl::search::Search<PointType>>(
          new pcl::search::KdTree<PointType>);
  reg_segrgb.setInputCloud(cloud);
  reg_segrgb.setSearchMethod(tree_segrgb);
  reg_segrgb.setDistanceThreshold(dist_thre); //距离的阀值
  reg_segrgb.setPointColorThreshold(
      point_color_thre); //点与点之间颜色容差 if points belong to the same
                         // region
  reg_segrgb.setRegionColorThreshold(
      region_color_thre); //区域之间容差 if regions can be merged.
  reg_segrgb.setMinClusterSize(min_cluster); //设置聚类的大小
  reg_segrgb.extract(clusters_rgb);
  //  seg_num = clusters_rgb.size ();
  cout << "uniform_sampling_cloudstart to redeliver..." << endl;
  clusters_rgb_deliver->clear();
  pcl::PointIndices empty_index;
  for (int i = 0; i < clusters_rgb.size(); i++)
    clusters_rgb_deliver->push_back(empty_index);
  bool being_colored = false;

  // core operation // using before-grid pointcloud to seg
  for (size_t i = 0; i < sampled_indices_cloud->points.size(); i++)
  {
    being_colored = false;
    for (size_t j = 0; j < clusters_rgb.size(); j++)
    {
      for (size_t k = 0; k < clusters_rgb[j].indices.size(); k++)
      {
        if (clusters_rgb[j].indices[k] == sampled_indices_cloud->points[i])
        {
          clusters_rgb_deliver->at(j).indices.push_back(
              clusters_rgb[j].indices[k]);
          being_colored = true;
        }
        if (being_colored)
          break;
      }
    }
  }
  return true;
}

//===================================================
//  rgbSegmentation
//  colour and region segmentation without keypoint
//===================================================
bool qpcl::rgbSegmentation(PointCloud::Ptr cloud,
                           std::vector<pcl::PointIndices>* clusters_rgb,
                           float dist_thre = 3, float point_color_thre = 13,
                           float region_color_thre = 30, int min_cluster = 350)
{
  pcl::RegionGrowingRGB<PointType> reg_segrgb;
  pcl::search::Search<PointType>::Ptr tree_segrgb =
      boost::shared_ptr<pcl::search::Search<PointType>>(
          new pcl::search::KdTree<PointType>);
  reg_segrgb.setInputCloud(cloud);
  reg_segrgb.setSearchMethod(tree_segrgb);
  reg_segrgb.setDistanceThreshold(
      dist_thre); //距离的阀值
                  /// 点与点之间颜色容差 if points belong to the same
  reg_segrgb.setPointColorThreshold(point_color_thre);
  /// 区域之间容差 if regions can be merged.
  reg_segrgb.setRegionColorThreshold(region_color_thre);
  reg_segrgb.setMinClusterSize(min_cluster); //设置聚类的大小
  reg_segrgb.extract(*clusters_rgb);
  //  seg_num = clusters_rgb.size ();
  cout << "uniform_sampling_cloudstart to redeliver..." << endl;
  return true;
}
