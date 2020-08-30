#ifndef QALIGN_H
#define QALIGN_H

#include "pcd_io.h"
#include <iostream>
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
  bool mapToPointCloudIndex(const rs2::texture_coordinate* Texture);
  void qalignTest();

  std::vector<cv::Point2i> match_points1;
  std::vector<cv::Point2i> match_points2;
private:
  cv::Mat img_1;
  cv::Mat img_2;
  std::vector<cv::DMatch> good_matches;
  std::vector<cv::KeyPoint> kpts_01, kpts_02;
};

#endif // QALIGN_H
