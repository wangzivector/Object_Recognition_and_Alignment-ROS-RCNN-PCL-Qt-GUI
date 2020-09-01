#ifndef QALIGN_H
#define QALIGN_H

#include "pcd_io.h"
#include <iostream>
#include <pcl/registration/transformation_estimation_svd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class qalign
{
public:
  qalign();
  bool detectMatchpoints();
  bool searchMatchedInMasked(cv::Mat mask_img1, cv::Mat mask_img2);
  bool mapToPointCloudIndex();
  void qalignTest();
  bool setSingCloudImage(PointCloud::Ptr cloud_input, PointCloud::Ptr cloud_input_seg, cv::Mat Image,
                         cv::Mat Mask_img,
                         const rs2::texture_coordinate* Texture);
  bool compute(bool mask_apply);
  void createMaskSample();

  bool indexImplement();
  Eigen::Matrix4f computeSVD();

  std::vector<cv::Point2i> match_points1;
  std::vector<cv::Point2i> match_points2;
  PointCloud::Ptr input_cloud1;
  PointCloud::Ptr input_cloud2;
  PointCloud::Ptr input_cloud1_seg;
  PointCloud::Ptr input_cloud2_seg;

private:
  cv::Mat img_1;
  cv::Mat img_2;
  cv::Mat img_1_mask;
  cv::Mat img_2_mask;
  PointCloud::Ptr cloud_out1;
  PointCloud::Ptr cloud_out2;
//  PointCloud::Ptr input_cloud1;
//  PointCloud::Ptr input_cloud2;
  const rs2::texture_coordinate* Texture1;
  const rs2::texture_coordinate* Texture2;

  bool position;

  std::vector<cv::DMatch> good_matches;
  std::vector<cv::KeyPoint> kpts_01, kpts_02;
};

#endif // QALIGN_H
