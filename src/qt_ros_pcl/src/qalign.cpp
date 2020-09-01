#include "qalign.h"

//===================================================
//  qalign
//  setup and clear the store
//===================================================
qalign::qalign()
{
  kpts_01.clear();
  kpts_02.clear();

  match_points1.clear();
  match_points2.clear();

  good_matches.clear();

  input_cloud1 = PointCloud::Ptr(new PointCloud());
  input_cloud2 = PointCloud::Ptr(new PointCloud());

  cloud_out1 = PointCloud::Ptr(new PointCloud());
  cloud_out2 = PointCloud::Ptr(new PointCloud());

  img_1_mask = cv::Mat();
  img_2_mask = cv::Mat();

  img_1 = cv::Mat();
  img_2 = cv::Mat();
  position = false;
}

void qalign::qalignTest()
{
  img_1 = cv::imread("/home/wang/openCVtest/build/10.png", CV_LOAD_IMAGE_COLOR);
  img_2 = cv::imread("/home/wang/openCVtest/build/11.png", CV_LOAD_IMAGE_COLOR);

  int nRows = 480;
  int nCols = 640;
  int row = 480;
  int col = 640;
  int intensity = 255;

  resize(img_1, img_1, cv::Size(nCols, nRows), 0, 0, cv::INTER_NEAREST);
  resize(img_2, img_2, cv::Size(nCols, nRows), 0, 0, cv::INTER_NEAREST);

  cv::Mat img(row, col, CV_8UC3);
  for (int i = 0; i < row; i++)
    for (int j = 0; j < col; j++)
      if ((i > row / 3) && (j > col / 3) && (i < row / 1.5) && (j < col / 1.5))
      {
        img.at<cv::Vec3b>(i, j)[0] = intensity;
        img.at<cv::Vec3b>(i, j)[1] = intensity;
        img.at<cv::Vec3b>(i, j)[2] = intensity;
      }
      else
      {
        img.at<cv::Vec3b>(i, j)[0] = 0;
        img.at<cv::Vec3b>(i, j)[1] = 0;
        img.at<cv::Vec3b>(i, j)[2] = 0;
      }

  cv::Mat img_m1 = img.clone();
  cv::Mat img_m2 = img.clone();

  std::printf("prepared all the stuff for detect points...\n");
  detectMatchpoints();
  searchMatchedInMasked(img_m1, img_m2);
  //  mapToPointCloudIndex(Texture);
}

void qalign::createMaskSample()
{
  int row = 480;
  int col = 640;
  int intensity = 255;

  cv::Mat img(row, col, CV_8UC3);
  for (int i = 0; i < row; i++)
    for (int j = 0; j < col; j++)
      if ((i > row / 3) && (j > col / 3) && (i < row / 1.5) && (j < col / 1.5))
      {
        img.at<cv::Vec3b>(i, j)[0] = intensity;
        img.at<cv::Vec3b>(i, j)[1] = intensity;
        img.at<cv::Vec3b>(i, j)[2] = intensity;
      }
      else
      {
        img.at<cv::Vec3b>(i, j)[0] = 0;
        img.at<cv::Vec3b>(i, j)[1] = 0;
        img.at<cv::Vec3b>(i, j)[2] = 0;
      }

  img_1_mask = img.clone();
  img_2_mask = img.clone();

  std::printf("createMaskSample done...\n");
}

//===================================================
//  detectMatchpoints
//  operate the SIFT and match algrithm
//===================================================
bool qalign::detectMatchpoints()
{
  // 创建AKAZE
  auto akaze_detector = cv::AKAZE::create();
  cv::Mat descriptors1, descriptors2;
  kpts_01.clear();
  kpts_02.clear();
  akaze_detector->detectAndCompute(img_1, cv::Mat(), kpts_01, descriptors1);
  akaze_detector->detectAndCompute(img_2, cv::Mat(), kpts_02, descriptors2);

  // 定义描述子匹配 - 暴力匹配
  cv::Ptr<cv::DescriptorMatcher> matchera =
      cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
  std::vector<cv::DMatch> matchesa;
  matchera->match(descriptors1, descriptors2, matchesa);

  //-- 第四步:匹配点对筛选
  double min_dista = 10000, max_dista = 0;
  //找出所有匹配之间的最小距离和最大距离,
  //即是最相似的和最不相似的两组点之间的距离
  for (int i = 0; i < descriptors1.rows; i++)
  {
    double dist = matchesa[i].distance;
    if (dist < min_dista)
      min_dista = dist;
    if (dist > max_dista)
      max_dista = dist;
  }
  std::printf("-- Max dist : %f \n", max_dista);
  std::printf("-- Min dist : %f \n", min_dista);

  double threadhold = min_dista + (max_dista - min_dista) / 1.5;
  std::printf("-- threadhold dist : %f \n", threadhold);

  //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
  good_matches.clear();
  for (int i = 0; i < descriptors1.rows; i++)
  {
    // if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
    if (matchesa[i].distance <= threadhold)
    {
      good_matches.push_back(matchesa[i]);
    }
  }
  // 绘制匹配
  cv::Mat img_matches;
  cv::drawMatches(img_1, kpts_01, img_2, kpts_02, good_matches, img_matches);
  cv::imwrite("./img_matches.png", img_matches);
  //  cv::imshow("AKAZE-Matches", img_matches);
  std::printf("size of good_matches :%zu\n", good_matches.size());

  if (good_matches.size() > 3)
    return true;
  else
  {
    std::printf("FAILED TO GET enough size of good_matches :%zu\n",
                good_matches.size());
    return false;
  }
}

//===================================================
//  searchMatchedInMasked
//  find out the matched points in masked
//===================================================
bool qalign::searchMatchedInMasked(cv::Mat mask_img1, cv::Mat mask_img2)
{
  cv::imwrite("./mask_img1.png",mask_img1);
  cv::imwrite("./mask_img2.png",mask_img2);
  if ((mask_img1.size == 0) || (mask_img2.size == 0))
  {
    std::printf("failed searchMatchedInMasked BECAUSE mask size == 0.\n");
    return false;
  }

  std::printf("start searchMatchedInMasked...\n");
  std::printf("before masked select size : %zu \n\n", good_matches.size());

  std::vector<cv::DMatch> good_matches_temp;
  for (int num_matched = 0; num_matched < good_matches.size(); num_matched++)
  {
    if ((mask_img1.at<cv::Vec3b>(
             kpts_01[good_matches[num_matched].queryIdx].pt.y,
             kpts_01[good_matches[num_matched].queryIdx].pt.x)[0] == 255) &&
        (mask_img2.at<cv::Vec3b>(
             kpts_02[good_matches[num_matched].trainIdx].pt.y,
             kpts_02[good_matches[num_matched].trainIdx].pt.x)[0] == 255))
      good_matches_temp.push_back(good_matches[num_matched]);
//        std::printf("indexing %d\n", num_matched);
  }
  good_matches.clear();
  good_matches.insert(good_matches.end(), good_matches_temp.begin(),
                      good_matches_temp.end());
  std::printf("after masked select size of good_matches:%zu\n\n",
              good_matches.size());
  std::printf("searchMatchedInMasked...end \n");
  if (good_matches.size() > 3)
  {
    // 绘制匹配
    cv::Mat img_matches_mask;
    cv::drawMatches(img_1, kpts_01, img_2, kpts_02, good_matches,
                    img_matches_mask);
    cv::imwrite("./img_matches_mask.png", img_matches_mask);
    return true;
  }
  else
  {
    std::printf(
        "FAILED TO GET enough matched points in searchMatchedInMasked.\n");
    return false;
  }
}

//===================================================
//  mapToPointCloudIndex
//  map the origin points to the pointcloud domain
//===================================================
bool qalign::mapToPointCloudIndex()
{

  /// print the macted pairs before transform
//  for (int num_matched = 0; num_matched < good_matches.size(); num_matched++)
//  {
//    std::printf("#before trans point %d  : [%d,%d]-->[%d,%d]\n", num_matched,
//                int(kpts_01[good_matches[num_matched].queryIdx].pt.x),
//                int(kpts_01[good_matches[num_matched].queryIdx].pt.y),
//                int(kpts_02[good_matches[num_matched].trainIdx].pt.x),
//                int(kpts_02[good_matches[num_matched].trainIdx].pt.y));
//  }
  /// end of print

  int width = 640;
  int height = 480;
  std::vector<cv::Point> mark_trans;
  match_points1.clear();
  match_points2.clear();
  mark_trans.clear();

  mark_trans.resize(good_matches.size());
  std::vector<cv::Point2i> match_points1_temp(good_matches.size());
  std::vector<cv::Point2i> match_points2_temp(good_matches.size());

  for (int Text_Index = 0; Text_Index < (width * height);
       ++Text_Index) // 0 ~ 480*640
  {
    rs2::texture_coordinate Texture_XY1 =
        Texture1[Text_Index]; // index 0 ~ 480*640
    int x_value1 =
        std::min(std::max(int(Texture_XY1.u * width + .5f), 0), width - 1);
    int y_value1 =
        std::min(std::max(int(Texture_XY1.v * height + .5f), 0), height - 1);
    int i = Text_Index % width; // /640
    int j = Text_Index / width;

    rs2::texture_coordinate Texture_XY2 =
        Texture2[Text_Index]; // index 0 ~ 480*640
    int x_value2 =
        std::min(std::max(int(Texture_XY2.u * width + .5f), 0), width - 1);
    int y_value2 =
        std::min(std::max(int(Texture_XY2.v * height + .5f), 0), height - 1);

    for (int num_matched = 0; num_matched < good_matches.size(); num_matched++)
    {
      if ((abs(int(kpts_01[good_matches[num_matched].queryIdx].pt.x) -
               x_value1) <= 1) &&
          (abs(int(kpts_01[good_matches[num_matched].queryIdx].pt.y) -
               y_value1) <= 1))
      {
        match_points1_temp[num_matched].x = i;
        match_points1_temp[num_matched].y = j;
        mark_trans[num_matched].x = 1;
      }
      if ((abs(int(kpts_02[good_matches[num_matched].trainIdx].pt.x) -
               x_value2) <= 1) &&
          (abs(int(kpts_02[good_matches[num_matched].trainIdx].pt.y) -
               y_value2) <= 1))
      {
        match_points2_temp[num_matched].x = i;
        match_points2_temp[num_matched].y = j;
        mark_trans[num_matched].y = 1;
      }
    }
  }

  /// print the after trans.
//  for (int num_matched = 0; num_matched < match_points1_temp.size();
//       num_matched++)
//  {
//    std::printf(
//        "   ##match_points1_temp %d  : [%d,%d]-->[%d,%d]\n", num_matched,
//        match_points1_temp[num_matched].x, match_points1_temp[num_matched].y,
//        match_points2_temp[num_matched].x, match_points2_temp[num_matched].y);
//  }
  /// end of print

  for (int num_matched = 0; num_matched < good_matches.size(); num_matched++)
  {
    if ((mark_trans[num_matched].x != 0) && (mark_trans[num_matched].y != 0))
    {
      match_points1.push_back(match_points1_temp[num_matched]);
      match_points2.push_back(match_points2_temp[num_matched]);
    }
  }

  /// print the after trans.
//  for (int num_matched = 0; num_matched < match_points1.size(); num_matched++)
//  {
//    std::printf("        ###after match_points2 %d  : [%d,%d]-->[%d,%d]\n",
//                num_matched, match_points1[num_matched].x,
//                match_points1[num_matched].y, match_points2[num_matched].x,
//                match_points2[num_matched].y);
//  }
  /// end of print

  std::printf("final matched and transformed point pairs num: %zu , %zu\n",
              match_points1.size(), match_points2.size());
  return true;
}

bool qalign::setSingCloudImage(PointCloud::Ptr cloud_input, PointCloud::Ptr cloud_input_seg, cv::Mat Image,
                               cv::Mat Mask_img,
                               const rs2::texture_coordinate* Texture_input)
{
  kpts_01.clear();
  kpts_02.clear();

  match_points1.clear();
  match_points2.clear();

  good_matches.clear();

  if (!position)
  {
    position = true;
    pcl::copyPointCloud(*cloud_input, *input_cloud1);
    pcl::copyPointCloud(*cloud_input_seg, *input_cloud1_seg);
    img_1 = Image.clone();
    img_1_mask = Mask_img.clone();
    Texture1 = Texture_input;
    std::printf("firstly added second data in decet 2d.\n");
  }
  else
  {
    pcl::copyPointCloud(*input_cloud2, *input_cloud1);
    pcl::copyPointCloud(*input_cloud2_seg, *input_cloud1_seg);
    img_1 = img_2.clone();
    img_1_mask = img_2_mask.clone();
    Texture1 = Texture2;
    std::printf("added second data in decet 2d.\n");
  }
  pcl::copyPointCloud(*cloud_input, *input_cloud2);
  pcl::copyPointCloud(*cloud_input_seg, *input_cloud2_seg);
  img_2 = Image.clone();
  img_2_mask = Mask_img.clone();
  Texture2 = Texture_input;
  return true;
}

bool qalign::compute(bool mask_apply)
{
  if (detectMatchpoints())
  {
    //    createMaskSample(); // test
    //    mask_apply = true;  // test

    if (mask_apply)
    {
      if (searchMatchedInMasked(img_1_mask, img_2_mask))
        /// compute transformation.
        mapToPointCloudIndex();
    }
    else
      mapToPointCloudIndex();
    std::printf("computed decet 2d.\n");
  }
  return true;
}

//===================================================
//  indexImplement
//  crop the organized cloud based mask
//===================================================
bool qalign::indexImplement()
{
  /// if not organized cloud, you can't indese it by cloud[][]
  if ((input_cloud1->height > 1) && (input_cloud2->height > 1))
  {
    /// if mask img match cloud
    if ((480 == input_cloud1->height) && (640 == input_cloud1->width) &&
        (480 == input_cloud2->height) && (640 == input_cloud2->width))
      std::printf("indexImplement: this is a orgnized pointcloud\n");
    else
    {
      std::cout << "rows =/= height or cols =/= width: " << 480
                << input_cloud1->height << "  " << 640 << input_cloud1->width
                << std::endl;
      return false;
    }
  }
  else
  {
    std::cout << "indexImplement: this is not a organized pointcloud\n"
              << std::endl;
    return false;
  }
  cloud_out1 = PointCloud::Ptr(new PointCloud());
  cloud_out2 = PointCloud::Ptr(new PointCloud());

  std::cout << "indexImplement match_points1.size():" << match_points1.size()
            << std::endl;
  for (int indice = 0; indice < match_points1.size(); indice++)
  {
//    std::cout<< "indice:"<< match_points1[indice].x << " " << match_points1[indice].y<< std::endl;
    cloud_out1->push_back(
        input_cloud1->at(match_points1[indice].x, match_points1[indice].y));
    cloud_out2->push_back(
        input_cloud2->at(match_points2[indice].x, match_points2[indice].y));
  }

  std::cout << "out cloud1 size :" << cloud_out1->height << " " << cloud_out1->width
            << std::endl;
  std::cout << "out cloud2 size :" << cloud_out2->height << " " << cloud_out2->width
            << std::endl;
  return true;
}

Eigen::Matrix4f qalign::computeSVD()
{
  if (cloud_out1->size() < 4)
  {
    std::cout << "cloud_out1->size() < 4" << std::endl;
    Eigen::Matrix4f transformation4f;
    return transformation4f;
  }
  pcl::Correspondence* corr;
  pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());
  for (int num_ind = 0; num_ind < cloud_out1->size(); num_ind++)
  {
    corr = new pcl::Correspondence(num_ind, num_ind, 1.0);
    model_scene_corrs->push_back(*corr);
  }
  pcl::registration::TransformationEstimationSVD<PointType, PointType>
      SVDEstimation;
//  Eigen::Matrix4f transformation4f;
  pcl::registration::TransformationEstimationSVD<PointType, PointType>::Matrix4
      transformation;
  std::cout << "start estimateRigidTransformation\n";
  SVDEstimation.estimateRigidTransformation(*cloud_out1, *cloud_out2,
                                            *model_scene_corrs, transformation);
  std::cout << "here is the transformation of two matched point" << std::endl;
  std::cout << transformation;
  return transformation;
}
