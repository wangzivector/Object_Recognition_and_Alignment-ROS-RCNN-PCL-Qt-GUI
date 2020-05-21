/*
 * objreco.cpp
 *
 *  Created on: May 13, 2020
 *      Author: wangzivector
 * This cppfile mass with pointcloud objection
 * recognition process
 */
#include "objreco.h"

ObjReco::ObjReco()
{
  cloud_world = PointCloud::Ptr(new PointCloud());
  cloud_world_filter = PointCloud::Ptr(new PointCloud());
  cloud_world_keypoint = PointCloud::Ptr(new PointCloud());
  cloud_world_normal = NormalCloud::Ptr(new NormalCloud());
  cloud_world_pointRGBNormal =
      PointRGBNormalCloud::Ptr(new PointRGBNormalCloud());

  cloud_object = PointCloud::Ptr(new PointCloud());
  cloud_object_filter = PointCloud::Ptr(new PointCloud());
  cloud_object_keypoint = PointCloud::Ptr(new PointCloud());
  cloud_object_normal = NormalCloud::Ptr(new NormalCloud());
  cloud_object_pointRGBNormal =
      PointRGBNormalCloud::Ptr(new PointRGBNormalCloud());

  cloud_descr_shot352_world =
      DescriptorCloudShot352::Ptr(new DescriptorCloudShot352());
  cloud_descr_shot1344_world =
      DescriptorCloudShot1344::Ptr(new DescriptorCloudShot1344());
  cloud_descr_fpfh_world = DescriptorCloudFPFH::Ptr(new DescriptorCloudFPFH());

  cloud_descr_shot352_object =
      DescriptorCloudShot352::Ptr(new DescriptorCloudShot352());
  cloud_descr_shot1344_object =
      DescriptorCloudShot1344::Ptr(new DescriptorCloudShot1344());
  cloud_descr_fpfh_object = DescriptorCloudFPFH::Ptr(new DescriptorCloudFPFH());

  deal_world = false;
  deal_object = false;
  deal_process = false;

  //
  // filters param
  //
  axisFilter_axis_size = 0.2f;
  gridFilter_grid_size = 0.002f;
  planeFilter_threshold_plane = 0.008;
  outlierFilter_outlier_meanK = 30;
  outlierFilter_outlier_Thresh = 0.1;
  backgroundFilter_noise_filter = 1;
  backgroundFilter_resolution = 0.015;

  /// param keypoints
  downSample_sample_radius = 0.005;

  /// param segmetation
  rgbSegmentation_dist_thre = 3;
  rgbSegmentation_point_color_thre = 13;
  rgbSegmentation_region_color_thre = 30;
  rgbSegmentation_min_cluster = 350;

  /// param reconstruction
  mlsReconstruction_search_radius = 0.03;

  /// param feature
  shot352Estimation_descr_rad_352 = 0.03;
  shot1344Estimation_descr_rad_1344 = 0.03;

  /// param registration
  RANSA_max_iterations = 200;
  RANSA_number_samples = 20;
  RANSA_randomness = 10;
  RANSA_similar_thre = 0.9f;
  RANSA_max_corr_distance = 0.015;
  RANSA_min_sample_distance = 0.25;

  NDT_transepsilon = 0.001;
  NDT_stepsize = 0.1;
  NDT_resolution = 1;
  NDT_maxiteration = 20;

  SACIA_number_samples = 20;
  SACIA_randomness = 10;
  SACIA_min_sample_distance = 0.25;
  SACIA_max_correspondence_distance = 0.015;
  SACIA_max_iterations = 200;

  ICP_max_corr_distance = 0.08;
  ICP_max_iter_icp = 100000;
  ICP_transformation = 1;
  ICP_euclidean_Fitness = 0.002;
}

//===================================================
//  checkReconstruction
//  take a look whether recongstruction works well
//===================================================
bool ObjReco::checkReconstruction()
{
  if (cloud_object_filter->size() == 0)
  {
    std::cout << "processing pointcloud doest be loaded yet.";
    return false;
  }
  mlsReconstruction(cloud_object_filter, cloud_object_pointRGBNormal,
                    mlsReconstruction_search_radius);
  cloud_object_filter.reset(new PointCloud());
  std::cout << "before copy cloud_pointRGBNormal : "
            << cloud_object_pointRGBNormal->size()
            << "  cloud_object_filter: " << cloud_object_filter->size()
            << std::endl;
  copyPointRGBNormalToPointRGB(cloud_object_pointRGBNormal,
                               cloud_object_filter);
  std::cout << "after copy cloud_pointRGBNormal : "
            << cloud_object_pointRGBNormal->size()
            << "  cloud_object_filter: " << cloud_object_filter->size()
            << std::endl;
  return true;
}

void ObjReco::reAxisFilter(bool is_do)
{
  if (is_do)
    axisFilter(cloud_world_filter, cloud_world_filter, axisFilter_axis_size,
               false);
  else
    return;
}

void ObjReco::reGridFilter(bool is_do)
{
  if (!is_do)
    return;
  if (deal_world)
    gridFilter(cloud_world_filter, cloud_world_filter, gridFilter_grid_size);
  if (deal_object)
    gridFilter(cloud_object_filter, cloud_object_filter, gridFilter_grid_size);
}

void ObjReco::rePlaneFilter(bool is_do)
{
  if (is_do)
    planeFilter(cloud_world_filter, cloud_world_filter,
                planeFilter_threshold_plane, false);
  else
    return;
}

void ObjReco::reOutlierFilter(bool is_do)
{
  if (is_do)
    outlierFilter(cloud_world_filter, cloud_world_filter,
                  outlierFilter_outlier_meanK, outlierFilter_outlier_Thresh);
  else
    return;
}

void ObjReco::reBackGroundFilter(bool is_do)
{
  if (is_do)
    //    backgroundFilter(cloud_world_filter, cloud_ground,
    //    cloud_world_filter, backgroundFilter_noise_filter,
    //    backgroundFilter_resolution);
    //  else
    return;
}

void ObjReco::reMlsRecoonstruction(bool is_do)
{
  if (is_do)
  {
    if (deal_world)
    {
      mlsReconstruction(cloud_world_filter, cloud_world_pointRGBNormal,
                        mlsReconstruction_search_radius);
      cloud_world_filter.reset(new PointCloud());
      copyPointRGBNormalToPointRGB(cloud_world_pointRGBNormal,
                                   cloud_world_filter);
    }
    if (deal_object)
    {
      mlsReconstruction(cloud_object_filter, cloud_object_pointRGBNormal,
                        mlsReconstruction_search_radius);
      cloud_object_filter.reset(new PointCloud());
      copyPointRGBNormalToPointRGB(cloud_object_pointRGBNormal,
                                   cloud_object_filter);
    }
  }
  return;
}

void ObjReco::reKeypoint(bool is_do)
{
  deal_process = false;
  if (is_do)
  {
    if (deal_world)
    {
      pcl::PointCloud<int>::Ptr indies_int =
          pcl::PointCloud<int>::Ptr(new pcl::PointCloud<int>());
      downSample(cloud_world_filter, indies_int, downSample_sample_radius);
      pcl::copyPointCloud(*cloud_world_filter, indies_int->points,
                          *cloud_world_keypoint);
    }
    if (deal_object)
    {
      pcl::PointCloud<int>::Ptr indies_into =
          pcl::PointCloud<int>::Ptr(new pcl::PointCloud<int>());
      downSample(cloud_object_filter, indies_into, downSample_sample_radius);
      pcl::copyPointCloud(*cloud_object_filter, indies_into->points,
                          *cloud_object_keypoint);
    }
    if (deal_object && deal_world)
      deal_process = true;
  }
  else
    return;
}

bool ObjReco::reNormalEstimation()
{
  if (deal_process)
  {
    std::cout << "start calculating normal ..." << std::endl;
    normalEstimation(cloud_world_filter, cloud_world_normal);
    normalEstimation(cloud_object_filter, cloud_object_normal);
    std::cout << "finished calculate normal." << std::endl;
    return true;
  }
  else
    return false;
}

bool ObjReco::reSHOT352(bool is_do)
{
  if (!is_do)
    return false;
  if (reNormalEstimation())
  {
    std::cout << "start calculating feature 1 ..." << std::endl;
    shot352Estimation(cloud_world_filter, cloud_world_keypoint,
                      cloud_world_normal, cloud_descr_shot352_world,
                      shot352Estimation_descr_rad_352);
    std::cout << "start calculating feature 2 ..." << std::endl;
    shot352Estimation(cloud_object_filter, cloud_object_keypoint,
                      cloud_object_normal, cloud_descr_shot352_object,
                      shot352Estimation_descr_rad_352);
    std::cout << "## finished two feature. ## " << std::endl;
    return true;
  }
  else
    return false;
}

bool ObjReco::reFPFH(bool is_do)
{
  if (!is_do)
    return false;
  if (reNormalEstimation())
  {
    std::cout << "start calculating feature 1 ..." << std::endl;
    fpfhEstimation(cloud_world_filter, cloud_world_normal,
                   cloud_descr_fpfh_world);
    std::cout << "start calculating feature 2 ..." << std::endl;
    fpfhEstimation(cloud_object_filter, cloud_object_normal,
                   cloud_descr_fpfh_object);
    std::cout << "## finished two feature. ## " << std::endl;
    return true;
  }
  else
    return false;
}

bool ObjReco::pcdReadWorld(std::string path)
{
  bool isit = pcdRead(path, cloud_world);
  pcl::copyPointCloud(*cloud_world, *cloud_world_filter);
  return isit;
}

bool ObjReco::pcdReadModel(std::string path)
{
  bool isit = pcdRead(path, cloud_object);
  pcl::copyPointCloud(*cloud_object, *cloud_object_filter);
  return isit;
}

void ObjReco::reloadPointCloud(bool world, bool object)
{
  if (world)
    pcl::copyPointCloud(*cloud_world, *cloud_world_filter);
  if (object)
    pcl::copyPointCloud(*cloud_object, *cloud_object_filter);
}

//===================================================
//  saveIni
//  save the param of obj reco tasks
//===================================================
bool ObjReco::saveIni()
{
  QSettings* Inifile = new QSettings(
      "/home/wang/catkin_qtws/src/qt_ros_pcl/resources/config.ini",
      QSettings::IniFormat);

  /// param filters
  Inifile->setValue("axisFilter_axis_size", axisFilter_axis_size);
  Inifile->setValue("gridFilter_grid_size", gridFilter_grid_size);
  Inifile->setValue("planeFilter_threshold_plane", planeFilter_threshold_plane);
  Inifile->setValue("outlierFilter_outlier_meanK", outlierFilter_outlier_meanK);
  Inifile->setValue("outlierFilter_outlier_Thresh",
                    outlierFilter_outlier_Thresh);
  Inifile->setValue("outlierFilter_noise_filter",
                    backgroundFilter_noise_filter);
  Inifile->setValue("backgroundFilter_resolution", backgroundFilter_resolution);

  /// param keypoints
  Inifile->setValue("downSample_sample_radius", downSample_sample_radius);

  /// param segmetation
  Inifile->setValue("rgbSegmentation_dist_thre", rgbSegmentation_dist_thre);
  Inifile->setValue("rgbSegmentation_point_color_thre",
                    rgbSegmentation_point_color_thre);
  Inifile->setValue("rgbSegmentation_region_color_thre",
                    rgbSegmentation_region_color_thre);
  Inifile->setValue("rgbSegmentation_min_cluster", rgbSegmentation_min_cluster);

  /// param reconstruction
  Inifile->setValue("mlsReconstruction_search_radius",
                    mlsReconstruction_search_radius);

  /// param feature
  Inifile->setValue("shot352Estimation_descr_rad_352",
                    shot352Estimation_descr_rad_352);
  Inifile->setValue("shot1344Estimation_descr_rad_1344",
                    shot1344Estimation_descr_rad_1344);

  /// param registration
  Inifile->setValue("RANSA_max_iterations", RANSA_max_iterations);
  Inifile->setValue("RANSA_number_samples", RANSA_number_samples);
  Inifile->setValue("RANSA_randomness", RANSA_randomness);
  Inifile->setValue("RANSA_similar_thre", RANSA_similar_thre);
  Inifile->setValue("RANSA_max_corr_distance", RANSA_max_corr_distance);
  Inifile->setValue("RANSA_min_sample_distance", RANSA_min_sample_distance);

  Inifile->setValue("NDT_transepsilon", NDT_transepsilon);
  Inifile->setValue("NDT_stepsize", NDT_stepsize);
  Inifile->setValue("NDT_resolution", NDT_resolution);
  Inifile->setValue("NDT_maxiteration", NDT_maxiteration);

  Inifile->setValue("SACIA_number_samples", SACIA_number_samples);
  Inifile->setValue("SACIA_randomness", SACIA_randomness);
  Inifile->setValue("SACIA_min_sample_distance", SACIA_min_sample_distance);
  Inifile->setValue("SACIA_max_correspondence_distance",
                    SACIA_max_correspondence_distance);
  Inifile->setValue("SACIA_max_iterations", SACIA_max_iterations);

  Inifile->setValue("ICP_max_corr_distance", ICP_max_corr_distance);
  Inifile->setValue("ICP_max_iter_icp", ICP_max_iter_icp);
  Inifile->setValue("ICP_transformation", ICP_transformation);
  Inifile->setValue("ICP_euclidean_Fitness", ICP_euclidean_Fitness);

  std::cout << "saved params" << std::endl;
  delete Inifile;
  return true;
}

//===================================================
//  loadIni
//  load ini when start this object.
//===================================================
bool ObjReco::loadIni(bool reset)
{
  QSettings* Inifile;
  if (!reset)
    Inifile = new QSettings(
        "/home/wang/catkin_qtws/src/qt_ros_pcl/resources/config.ini",
        QSettings::IniFormat);
  else
    Inifile = new QSettings(
        "/home/wang/catkin_qtws/src/qt_ros_pcl/resources/config_base.ini",
        QSettings::IniFormat);
  //  int intLock = Inifile->value("lockTime").toInt();
  /// param filters
  axisFilter_axis_size = Inifile->value("axisFilter_axis_size").toFloat();
  gridFilter_grid_size = Inifile->value("gridFilter_grid_size").toFloat();
  planeFilter_threshold_plane =
      Inifile->value("planeFilter_threshold_plane").toDouble();
  outlierFilter_outlier_meanK =
      Inifile->value("outlierFilter_outlier_meanK").toInt();
  outlierFilter_outlier_Thresh =
      Inifile->value("outlierFilter_outlier_Thresh").toDouble();
  backgroundFilter_noise_filter =
      Inifile->value("outlierFilter_noise_filter").toInt();
  backgroundFilter_resolution =
      Inifile->value("backgroundFilter_resolution").toDouble();

  /// param keypoints
  downSample_sample_radius =
      Inifile->value("downSample_sample_radius").toDouble();

  /// param segmetation
  rgbSegmentation_dist_thre =
      Inifile->value("rgbSegmentation_dist_thre").toFloat();
  rgbSegmentation_point_color_thre =
      Inifile->value("rgbSegmentation_point_color_thre").toFloat();
  rgbSegmentation_region_color_thre =
      Inifile->value("rgbSegmentation_region_color_thre").toFloat();
  rgbSegmentation_min_cluster =
      Inifile->value("rgbSegmentation_min_cluster").toInt();

  /// param reconstruction
  mlsReconstruction_search_radius =
      Inifile->value("mlsReconstruction_search_radius").toDouble();

  /// param feature
  shot352Estimation_descr_rad_352 =
      Inifile->value("shot352Estimation_descr_rad_352").toDouble();
  shot1344Estimation_descr_rad_1344 =
      Inifile->value("shot1344Estimation_descr_rad_1344").toDouble();

  /// param registration
  RANSA_max_iterations = Inifile->value("RANSA_max_iterations").toInt();
  RANSA_number_samples = Inifile->value("RANSA_number_samples").toInt();
  RANSA_randomness = Inifile->value("RANSA_randomness").toInt();
  RANSA_similar_thre = Inifile->value("RANSA_similar_thre").toFloat();
  RANSA_max_corr_distance =
      Inifile->value("RANSA_max_corr_distance").toDouble();
  RANSA_min_sample_distance =
      Inifile->value("RANSA_min_sample_distance").toFloat();

  NDT_transepsilon = Inifile->value("NDT_transepsilon").toDouble();
  NDT_stepsize = Inifile->value("NDT_stepsize").toDouble();
  NDT_resolution = Inifile->value("NDT_resolution").toFloat();
  NDT_maxiteration = Inifile->value("NDT_maxiteration").toInt();

  SACIA_number_samples = Inifile->value("SACIA_number_samples").toInt();
  SACIA_randomness = Inifile->value("SACIA_randomness").toInt();
  SACIA_min_sample_distance =
      Inifile->value("SACIA_min_sample_distance").toFloat();
  SACIA_max_correspondence_distance =
      Inifile->value("SACIA_max_correspondence_distance").toDouble();
  SACIA_max_iterations = Inifile->value("SACIA_max_iterations").toInt();

  ICP_max_corr_distance = Inifile->value("ICP_max_corr_distance").toDouble();
  ICP_max_iter_icp = Inifile->value("ICP_max_iter_icp").toInt();
  ICP_transformation = Inifile->value("ICP_transformation").toDouble();
  ICP_euclidean_Fitness = Inifile->value("ICP_euclidean_Fitness").toDouble();

  //  std::cout << "ICP_euclidean_Fitness : " << ICP_euclidean_Fitness <<
  //  std::endl;
  delete Inifile;
  return true;
}
