/*
 * objreco.h
 *
 *  Created on: May 13, 2020
 *      Author: wangzivector
 * This cppfile mass with pointcloud objection
 * recognition process
 */
#ifndef OBJRECO_H
#define OBJRECO_H
#include "qpcl.h"
#include "pcd_io.h"
#include <QSettings>

//
//  make a full arrangement in reco and
//  params management.
//
class ObjReco : public qpcl, public pcd_io
{
public:
  ObjReco();
  bool checkReconstruction();
  bool saveIni();
  bool loadIni(bool reset = false);
  bool pcdReadModel(std::string path);
  bool pcdReadWorld(std::string path);
  void reloadPointCloud(bool world, bool object);

  void reAxisFilter(bool is_do);
  void reGridFilter(bool is_do);
  void rePlaneFilter(bool is_do);
  void reOutlierFilter(bool is_do);
  void reBackGroundFilter(bool is_do);
  void reMlsRecoonstruction(bool is_do);
  void reKeypoint(bool is_do);
  bool reNormalEstimation();
  bool reSHOT352(bool is_do);
  bool reFPFH(bool is_do);
  bool reSACIAFPFH(bool is_do);
  bool reSACIASHOT352(bool is_do);
  bool reRANSACFPFH(bool is_do);
  bool reRANSACSHOT352(bool is_do);
  bool reNDT(bool is_do);
  bool reICP(bool is_do);

  bool deal_world;
  bool deal_object;
  bool deal_process;
  bool deal_fpfh;
  bool deal_shot352;

  /// pointcloud
  PointCloud::Ptr cloud_world;
  PointCloud::Ptr cloud_world_filter;
  PointCloud::Ptr cloud_world_keypoint;
  PointCloud::Ptr cloud_world_aligned;
  PointCloud::Ptr cloud_world_registrated;
  NormalCloud::Ptr cloud_world_normal;
  PointRGBNormalCloud::Ptr cloud_world_pointRGBNormal;

  PointCloud::Ptr cloud_object;
  PointCloud::Ptr cloud_object_filter;
  PointCloud::Ptr cloud_object_keypoint;
  PointCloud::Ptr cloud_object_aligned;
  PointCloud::Ptr cloud_object_registrated;
  NormalCloud::Ptr cloud_object_normal;
  PointRGBNormalCloud::Ptr cloud_object_pointRGBNormal;

  DescriptorCloudShot352::Ptr cloud_descr_shot352_world;
  DescriptorCloudShot1344::Ptr cloud_descr_shot1344_world;
  DescriptorCloudFPFH::Ptr cloud_descr_fpfh_world;

  DescriptorCloudShot352::Ptr cloud_descr_shot352_object;
  DescriptorCloudShot1344::Ptr cloud_descr_shot1344_object;
  DescriptorCloudFPFH::Ptr cloud_descr_fpfh_object;

  Eigen::Matrix4f trans_align;
  Eigen::Matrix4f trans_regi_ndt;
  Eigen::Matrix4f trans_regi_icp;
  Eigen::Matrix4f trans_filter_regi;


  /// param filters
  float axisFilter_axis_size;
  float gridFilter_grid_size;
  double planeFilter_threshold_plane;
  int outlierFilter_outlier_meanK;
  double outlierFilter_outlier_Thresh;
  int backgroundFilter_noise_filter;
  double backgroundFilter_resolution;

  /// param keypoints
  double downSample_sample_radius;

  /// param segmetation
  float rgbSegmentation_dist_thre;
  float rgbSegmentation_point_color_thre;
  float rgbSegmentation_region_color_thre;
  int rgbSegmentation_min_cluster;

  /// param reconstruction
  double mlsReconstruction_search_radius;

  /// param feature
  double shot352Estimation_descr_rad_352;
  double shot1344Estimation_descr_rad_1344;

  /// param registration
  int RANSA_max_iterations;
  int RANSA_number_samples;
  int RANSA_randomness;
  float RANSA_similar_thre;
  double RANSA_max_corr_distance ;
  float RANSA_min_sample_distance;

  double NDT_transepsilon;
  double NDT_stepsize;
  float NDT_resolution;
  int NDT_maxiteration;

  int SACIA_number_samples;
  int SACIA_randomness;
  float SACIA_min_sample_distance;
  double SACIA_max_correspondence_distance;
  int SACIA_max_iterations;

  double ICP_max_corr_distance;
  int ICP_max_iter_icp;
  double ICP_transformation;
  double ICP_euclidean_Fitness;

};

#endif // OBJRECO_H
