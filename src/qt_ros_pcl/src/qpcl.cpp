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
                      const float axis_size, bool keep_organized)
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
  std::cout << "after axis filter point size: " << cloud_axis->size() << std::endl;
  return true;
}

//===================================================
//  gridFilter
//  use for the grid /downsample
//  Create the filtering object and downsample dataset
//  using the parameter leaf size
//===================================================
bool qpcl::gridFilter(PointCloud::Ptr cloud, PointCloud::Ptr cloud_grid,
                      float grid_size)
{
  /// use for the grid /downsample
  pcl::VoxelGrid<PointType> sor_grid;
  /// Create the filtering object and downsample the dataset
  /// using the parameter leaf size
  sor_grid.setInputCloud(cloud);
  sor_grid.setLeafSize(grid_size, grid_size, grid_size);
  sor_grid.filter(*cloud_grid);
  std::cout << "after grid point size: " << cloud_grid->size() << std::endl;
  return true;
}

//===================================================
//  planeFilter
//  remove the main plane of pointcloud
//  with the max size of distance
//===================================================
bool qpcl::planeFilter(PointCloud::Ptr cloud, PointCloud::Ptr cloud_extract,
                       double threshold_plane, bool keep_organized)
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
  extract_planes.setNegative(true);
  /// Extract the found plane to remove the table
  pcl::ModelCoefficients
      coefficients_plane; /// construct coefficients for plane
  pcl::PointIndices::Ptr inliers_plane =
      pcl::PointIndices::Ptr(new pcl::PointIndices);
  /// constructor for point found as part of surface

  /// set maximal distance from point to surface to be identified as plane
  seg_plane.setDistanceThreshold(threshold_plane);
  /// maybe should set as the cloud_r with the remove part to keep mess
  seg_plane.setInputCloud(cloud);
  seg_plane.segment(*inliers_plane, coefficients_plane);

  /// remove plane from point cloud
  extract_planes.setInputCloud(cloud);
  extract_planes.setIndices((inliers_plane));
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
                         int outlier_meanK, double outlier_Thresh)
{
  /// use for the outlier
  pcl::StatisticalOutlierRemoval<PointType> sor_outlier;
  sor_outlier.setInputCloud(cloud);
  sor_outlier.setMeanK(outlier_meanK);
  sor_outlier.setStddevMulThresh(outlier_Thresh);
  sor_outlier.filter(*cloud_outlier);
  std::cout << "after outlier point size: " << cloud_outlier->size() <<std::endl;
  return true;
}

//===================================================
//  backgroundFilter - may still need modify
//  remove the outlier of pointcloud near background
//  with in distance of resolution
//  after it will apply noise_filter
//===================================================
bool qpcl::backgroundFilter(PointCloud::Ptr cloud_g, PointCloud::Ptr cloud_i,
                            PointCloud::Ptr cloud_extract, int noise_filter,
                            double resolution)
{
  pcl::octree::OctreePointCloudChangeDetector<PointType> octree(resolution);
  std::cout << "ground octree ... cloud_groud : " << cloud_g->size()
            << " resolution: " << resolution << std::endl;
  octree.setInputCloud(cloud_g);
  octree.addPointsFromInputCloud();
  octree.switchBuffers();
  std::cout << "compare change ... cloud_input : " << cloud_i->size() << std::endl;
  octree.setInputCloud(cloud_i);
  octree.addPointsFromInputCloud();
  std::vector<int> newPointIdxVector;
  /// Get vector of point indices from octree voxels which did not exist in
  /// previous buffer
  octree.getPointIndicesFromNewVoxels(newPointIdxVector, noise_filter);
  std::cout << "  newPointIdxVector : " << newPointIdxVector.size() << std::endl;

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
    cout << "after extra point size: " << cloud_extract->size() << std::endl;
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
                      double sample_radius)
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
    std::vector<pcl::PointIndices>* clusters_rgb_deliver, float dist_thre,
    float point_color_thre, float region_color_thre, int min_cluster)
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
  cout << "uniform_sampling_cloudstart to redeliver..." << std::endl;
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
                           float dist_thre, float point_color_thre,
                           float region_color_thre, int min_cluster)
{
  pcl::RegionGrowingRGB<PointType> reg_segrgb;
  pcl::search::Search<PointType>::Ptr tree_segrgb =
      boost::shared_ptr<pcl::search::Search<PointType>>(
          new pcl::search::KdTree<PointType>);
  reg_segrgb.setInputCloud(cloud);
  reg_segrgb.setSearchMethod(tree_segrgb);
  ///距离的阀值
  reg_segrgb.setDistanceThreshold(dist_thre);
  /// 点与点之间颜色容差 if points belong to the same
  reg_segrgb.setPointColorThreshold(point_color_thre);
  /// 区域之间容差 if regions can be merged.
  reg_segrgb.setRegionColorThreshold(region_color_thre);
  ///设置聚类的大小
  reg_segrgb.setMinClusterSize(min_cluster);
  reg_segrgb.extract(*clusters_rgb);
  //  seg_num = clusters_rgb.size ();
  cout << "uniform_sampling_cloudstart to redeliver..." << std::endl;
  return true;
}

//===================================================
//  normalEstimation
//  may be alot time
// 	NormalCloud::Ptr cloud_normals=
//             NormalCloud::Ptr(new NormalCloud());
//===================================================
bool qpcl::normalEstimation(PointCloud::Ptr cloud,
                            NormalCloud::Ptr cloud_normal)
{

  pcl::NormalEstimationOMP<PointType, NormalType> norm_est_cloud;
  norm_est_cloud.setInputCloud(cloud);
  pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());;
  norm_est_cloud.setSearchMethod(tree);
  // norm_est_cloud.setRadiusSearch (0.03);
  norm_est_cloud.setKSearch(8); // maybe this can be adjust 10
  norm_est_cloud.setNumberOfThreads(4);
  // norm_est_cloud.setInputCloud (cloud_keypoints);
  norm_est_cloud.compute(*cloud_normal); // compute normals
  return true;
}

//===================================================
//  shot352Estimation
//  may failed because some reasons in pcl
//  output of shot 352 which not use color
//  DescriptorCloudShot352::Ptr cloud_descriptors_shot352
//  = DescriptorCloudShot352::Ptr
//  (new DescriptorCloudShot352());
//===================================================
bool qpcl::shot352Estimation(
    PointCloud::Ptr cloud, PointCloud::Ptr cloud_keypoint,
    NormalCloud::Ptr cloud_normal,
    DescriptorCloudShot352::Ptr cloud_descriptors_shot352, double descr_rad_352)
{
  pcl::SHOTEstimationOMP<PointType, NormalType, SHOT352> descr_est_shot352;
  descr_est_shot352.setInputCloud(cloud_keypoint);
  descr_est_shot352.setRadiusSearch(descr_rad_352);
  descr_est_shot352.setInputNormals(cloud_normal);
  descr_est_shot352.setSearchSurface(cloud);
  descr_est_shot352.compute(*cloud_descriptors_shot352);
  return true;
}

//===================================================
//  shot352Estimation
//  may failed because some reasons in pcl
//  this is with color compared with shot352,
//  i think need to make a justice which is better
//  DescriptorCloudShot1344::Ptr cloud_descriptors_shot1344
//  = DescriptorCloudShot1344::Ptr(new DescriptorCloudShot1344());
//===================================================
bool qpcl::shot1344Estimation(
    PointCloud::Ptr cloud, PointCloud::Ptr cloud_keypoint,
    NormalCloud::Ptr cloud_normal,
    DescriptorCloudShot1344::Ptr cloud_descriptors_shot1344,
    double descr_rad_1344)
{
  pcl::SHOTColorEstimationOMP<PointType, NormalType, SHOT1344>
      descr_est_shot1344;
  descr_est_shot1344.setInputCloud(cloud_keypoint);
  descr_est_shot1344.setRadiusSearch(descr_rad_1344);
  descr_est_shot1344.setInputNormals(cloud_normal);
  descr_est_shot1344.setSearchSurface(cloud);
  descr_est_shot1344.compute(*cloud_descriptors_shot1344);
  return true;
}

//===================================================
//  fpfhEstimation
//  DescriptorCloudFPFH::Ptr cloud_descriptors_FPFH
//  = DescriptorCloudFPFH::Ptr (new DescriptorCloudFPFH());
//===================================================
bool qpcl::fpfhEstimation(PointCloud::Ptr cloud, NormalCloud::Ptr cloud_normal,
                          DescriptorCloudFPFH::Ptr cloud_descriptors_FPFH)
{
  pcl::FPFHEstimationOMP<PointType, NormalType, FPFH> descr_est_fpfh;
  pcl::search::KdTree<PointType>::Ptr tree_fpfh(new pcl::search::KdTree<PointType>);

  descr_est_fpfh.setNumberOfThreads(4); ///指定4核计算
  descr_est_fpfh.setInputCloud(cloud);
  // descr_est_fpfh.setInputCloud(cloud_keypoints);
  descr_est_fpfh.setInputNormals(cloud_normal);
  descr_est_fpfh.setSearchMethod(tree_fpfh);
  descr_est_fpfh.setKSearch(10); /// 10

  descr_est_fpfh.compute(*cloud_descriptors_FPFH);
  return true;
}

//===================================================
//  mlsReconstruction
//  PointRGBNormalCloud::Ptr cloud_pointRGBNormal
//  = PointRGBNormalCloud::Ptr (new PointRGBNormalCloud());
//===================================================
bool qpcl::mlsReconstruction(PointCloud::Ptr cloud,
                             PointRGBNormalCloud::Ptr cloud_pointRGBNormal,
                             double search_radius)
{
  /// Smoothing object (we choose what point types we want as input and output).
  pcl::MovingLeastSquares<PointType, PointRGBNormalType> filter_re;
  std::cout << "origin size of cloud :" << cloud->size() << std::endl;
  std::vector<int> mapping;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);
  filter_re.setInputCloud(cloud);
  /// Use all neighbors in a radius of 3cm.
  filter_re.setSearchRadius(search_radius);
  filter_re.setPolynomialOrder(2); /// MLS拟合曲线的阶数，这个阶数在构造函数里默认是2
  /// If true, the surface and normal are approximated using a polynomial
  /// estimation (if false, only a tangent one).
  filter_re.setPolynomialFit(true);
  /// We can tell the algorithm to also compute smoothed normals (optional).
  filter_re.setComputeNormals(false);
  /// kd-tree object for performing searches.
  pcl::search::KdTree<PointType>::Ptr kdtree;
  //  (new pcl::search::KdTree<PointType>)
  // or pcl::search::KdTree<PointType>::Ptr kdtree (new
  //    pcl::search::KdTree<pcl::PointXYZ>);
  // or 1. pcl::search::KdTree<PointType> kdtree;
  //    2. pcl::search::KdTree<PointType>::Ptr (&kdtree)
  filter_re.setSearchMethod(kdtree);
  std::cout << "start mlsReconstruction process" << std::endl;
  filter_re.process(*cloud_pointRGBNormal);
  std::cout << "after reconstruction size = " << cloud_pointRGBNormal->size() << std::endl;
  return true;
}

//===================================================
//  RANSACRegistration
//  SampleConsensusPrerejective to get position
//  recognition to get the matrix transform the model
//===================================================
Eigen::Matrix4f
qpcl::RANSACRegistration(PointCloud::Ptr source_cloud_keypoint,
                         DescriptorCloudFPFH::Ptr source_descriptors_FPFH,
                         PointCloud::Ptr target_cloud_kepoint,
                         DescriptorCloudFPFH::Ptr target_descriptors_FPFH,
                         PointCloud::Ptr cloud_aligned, int max_iterations,
                         int number_samples, int randomness, float similar_thre,
                         double max_corr_distance, float min_sample_distance)
{
  pcl::SampleConsensusPrerejective<PointType, PointType, FPFH> SACPJ;
  SACPJ.setInputSource(source_cloud_keypoint);
  SACPJ.setSourceFeatures(source_descriptors_FPFH);
  SACPJ.setInputTarget(target_cloud_kepoint);
  SACPJ.setTargetFeatures(target_descriptors_FPFH);
  SACPJ.setMaximumIterations(max_iterations); // Number of RANSAC iterations
  SACPJ.setNumberOfSamples(number_samples);   // Number of points to sample for
                                              // generating/prerejecting a pose
  SACPJ.setCorrespondenceRandomness(
      randomness); // Number of nearest features to use
  SACPJ.setSimilarityThreshold(
      similar_thre); // Polygonal edge length similarity threshold
  SACPJ.setMaxCorrespondenceDistance(max_corr_distance); // Inlier threshold
  SACPJ.setInlierFraction(min_sample_distance); // Required inlier fraction for
                                                // accepting a pose hypothesis
  SACPJ.align(*cloud_aligned);
  Eigen::Matrix4f transformation = SACPJ.getFinalTransformation();
  // Print results
  printf("\n");
  pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0),
                           transformation(0, 1), transformation(0, 2));
  pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0),
                           transformation(1, 1), transformation(1, 2));
  pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0),
                           transformation(2, 1), transformation(2, 2));
  pcl::console::print_info("\n");
  pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n",
                           transformation(0, 3), transformation(1, 3),
                           transformation(2, 3));
  pcl::console::print_info("\n");
  pcl::console::print_info("Inliers: %i/%i\n", SACPJ.getInliers().size(),
                           target_cloud_kepoint->size());
  return transformation;
}

//===================================================
//  SACIARegistration
//  SACIA  method to get position recognition
//  to get the matrix transform the model into world
//===================================================
Eigen::Matrix4f
qpcl::SACIARegistration(PointCloud::Ptr source_cloud,
                        DescriptorCloudFPFH::Ptr source_descriptor_fpfh,
                        PointCloud::Ptr target_cloud,
                        DescriptorCloudFPFH::Ptr target_descriptor_fpfh,
                        PointCloud::Ptr cloud_aligned, int number_samples,
                        int randomness, float min_sample_distance,
                        double max_correspondence_distance, int max_iterations)
{
  printf("process source/target = %d/%d of points \n", (int)source_cloud->size(),
         (int)target_cloud->size());
  //
  //   sacia for a rough align
  //
  pcl::SampleConsensusInitialAlignment<PointType, PointType, FPFH> SACIA;
  SACIA.setInputSource(source_cloud);
  SACIA.setSourceFeatures(source_descriptor_fpfh);
  SACIA.setInputTarget(target_cloud);
  SACIA.setTargetFeatures(target_descriptor_fpfh);

  SACIA.setNumberOfSamples(number_samples);
  ///设置每次迭代计算中使用的样本数量（可省）,可节省时间
  SACIA.setCorrespondenceRandomness(randomness);
  ///设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
  SACIA.setMinSampleDistance(min_sample_distance);
  /// we’ve decided to truncate the error with an upper limit of 0.01 squared.
  SACIA.setMaxCorrespondenceDistance(max_correspondence_distance);
  /// maximum correspondence distance is actually specified as squared distance;
  SACIA.setMaximumIterations(max_iterations);
  SACIA.align(*cloud_aligned);
  float sacia_fitness_score =
      (float)SACIA.getFitnessScore(max_correspondence_distance);
  Eigen::Matrix4f matrix1 = SACIA.getFinalTransformation();

  printf("    | %6.3f %6.3f %6.3f | \n", matrix1(0, 0), matrix1(0, 1),
         matrix1(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", matrix1(1, 0), matrix1(1, 1),
         matrix1(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", matrix1(2, 0), matrix1(2, 1),
         matrix1(2, 2));
  std::cout << "with sacia_fitness_score : " << sacia_fitness_score << std::endl;
  return matrix1;
}


//===================================================
//  NDTRegistration
//  Normal Distributions Transform to get position
//  recognition to get the matrix transform the model
//===================================================
Eigen::Matrix4f qpcl::NDTRegistration(PointCloud::Ptr source_cloud_keypoint,
                                      PointCloud::Ptr target_cloud_keypoint,
                                      double ndt_transepsilon,
                                      double ndt_stepsize, float ndt_resolution,
                                      int ndt_maxiteration)
{
  cout << "start NDTRegistration ..." << std::endl;
  printf("NDTR source/target = %d/%d points",
         (int)source_cloud_keypoint->size(),
         (int)target_cloud_keypoint->size());
  /// Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<PointType, PointType> ndt;
  PointCloud::Ptr cloud_aligned = PointCloud::Ptr (new PointCloud());
  /// Setting scale dependent NDT parameters
  ndt.setTransformationEpsilon(ndt_transepsilon); /// converg condiction
  /// Setting minimum transformation difference for termination condition.
  ndt.setStepSize(ndt_stepsize);
  /// Setting maximum step size for More-Thuente line search.
  ndt.setResolution(ndt_resolution);
  /// Setting Resolution of NDT grid structure(VoxelGridCovariance).
  /// must be adjusted before use
  ndt.setMaximumIterations(ndt_maxiteration);
  /// Setting max number of registration iterations.
  ndt.setInputSource(source_cloud_keypoint);
  /// Setting point cloud to be aligned.
  ndt.setInputTarget(target_cloud_keypoint);
  /// Setting point cloud to be aligned to.

  /// Set initial alignment estimate found using robot odometry.
  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f result_transformation;
  // Calculating required rigid transform to align input cloud to target cloud.
  ndt.align(*cloud_aligned, init_guess);
  result_transformation = ndt.getFinalTransformation();
  double NDT_score = ndt.getFitnessScore();
  std::cout << "\n Normal Distributions Transform has converged:"
            << ndt.hasConverged() << "\n score: " << NDT_score << std::endl
            << "matrix : \n" << result_transformation << std::endl;
  return result_transformation;
}


//===================================================
//  ICPRegistration
//  ICP method to get position recognition
//  to get the matrix transform the model into world
//  pcl::transformPointCloud(*object_keypoints,
//         *inverse_pointcloud, matrix3.inverse());
//===================================================
Eigen::Matrix4f qpcl::ICPRegistration(PointCloud::Ptr source_cloud,
                                PointCloud::Ptr target_cloud,
                                PointCloud::Ptr cloud_icped,
                                double max_corr_distance, int max_iter_icp,
                                double transformation, double euclidean_Fitness)
{
  //
  //   icp for percise align
  //
  pcl::IterativeClosestPoint<PointType, PointType> ICP;
  ICP.setInputSource(source_cloud);
  /// how abaot icpwithnolinear and gerneral icp
  ICP.setInputTarget(target_cloud);
  ICP.setMaxCorrespondenceDistance(max_corr_distance);
  /// Set the max correspondence distance to 4cm
  /// (e.g.,correspondences with higher distances will be ignored)
  ICP.setMaximumIterations(max_iter_icp);
  /// 最大迭代次数
  ICP.setTransformationEpsilon(transformation); /// converg condiction
  /// 两次变化矩阵之间的差值//setTransformationEpsilon，
  /// 前一个变换矩阵和当前变换矩阵的差异小于阈值时，就认为已经收敛了，是一条收敛条件
  ICP.setEuclideanFitnessEpsilon(euclidean_Fitness); /// converg condiction
  ///收敛条件是均方误差和小于阈值， 停止迭代。
  ICP.align(*cloud_icped);
  Eigen::Matrix4f matrix2 = ICP.getFinalTransformation();
  double icp_fitness_score = ICP.getFitnessScore();
  printf(" icp score is : %f\n", icp_fitness_score);
  printf("    | %6.3f %6.3f %6.3f | \n", matrix2(0, 0), matrix2(0, 1),
         matrix2(0, 2));
  printf("R = | %6.3f %6.3f %6.3f |  of icp matrics \n", matrix2(1, 0),
         matrix2(1, 1), matrix2(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", matrix2(2, 0), matrix2(2, 1),
         matrix2(2, 2));
}
