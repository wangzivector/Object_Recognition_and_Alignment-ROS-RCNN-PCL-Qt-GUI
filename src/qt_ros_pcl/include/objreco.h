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

  /// param filters
  float axisFilter_axis_size;
  float gridFilter_grid_size;
  double planeFilter_threshold_plane;
  int outlierFilter_outlier_meanK;
  double outlierFilter_outlier_Thresh;
  int outlierFilter_noise_filter;
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

  /// pointcloud
  PointCloud::Ptr cloud_world;
  PointRGBNormalCloud::Ptr cloud_pointRGBNormal;
};

#endif // OBJRECO_H
