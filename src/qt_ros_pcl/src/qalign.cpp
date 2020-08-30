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

  img_1 = cv::Mat();
  img_2 = cv::Mat();
}

void qalign::qalignTest()
{
  img_1 = cv::imread ( "/home/wang/openCVtest/build/10.png", CV_LOAD_IMAGE_COLOR );
  img_2 = cv::imread ( "/home/wang/openCVtest/build/11.png", CV_LOAD_IMAGE_COLOR );

  int nRows = 480;
  int nCols =640;
  int row = 480;
  int col = 640;
  int intensity = 255;

  resize(img_1, img_1, cv::Size(nCols,nRows), 0, 0, cv::INTER_NEAREST);
  resize(img_2, img_2, cv::Size(nCols,nRows), 0, 0, cv::INTER_NEAREST);

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
  searchMatchedInMasked(img_m1,img_m2);
//  mapToPointCloudIndex(Texture);

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

  double threadhold = min_dista + (max_dista - min_dista) / 3;
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
  cv::imwrite("./img_matches.png",img_matches);
//  cv::imshow("AKAZE-Matches", img_matches);
  std::printf("size of good_matches :%zu\n", good_matches.size());
  return true;
}

//===================================================
//  searchMatchedInMasked
//  find out the matched points in masked
//===================================================
bool qalign::searchMatchedInMasked(cv::Mat mask_img1, cv::Mat mask_img2)
{
  std::printf("start searchMatchedInMasked...\n");
  std::vector<cv::DMatch> good_matches_temp;
  std::printf("searchMatchedInMasked...1\n");
  for (int num_matched = 0; num_matched < good_matches.size(); num_matched++)
  {
    if ((mask_img1.at<cv::Vec3b>(
             kpts_01[good_matches[num_matched].queryIdx].pt.x,
             kpts_01[good_matches[num_matched].queryIdx].pt.y)[0] == 255) &&
        (mask_img2.at<cv::Vec3b>(
             kpts_02[good_matches[num_matched].trainIdx].pt.x,
             kpts_02[good_matches[num_matched].trainIdx].pt.y)[0] == 255))
      good_matches_temp.push_back(good_matches[num_matched]);
    std::printf("indexing %d\n", num_matched);
  }
  std::printf("searchMatchedInMasked...2\n");
  good_matches.clear();
  good_matches.insert(good_matches.end(), good_matches_temp.begin(),
                      good_matches_temp.end());
  std::printf("size of good_matches:%zu", good_matches.size());
  std::printf("searchMatchedInMasked...end \n");
  return true;
}

//===================================================
//  mapToPointCloudIndex
//  map the origin points to the pointcloud domain
//===================================================
bool qalign::mapToPointCloudIndex(const rs2::texture_coordinate* Texture)
{
  int width = 640;
  int height = 480;

  for (int Text_Index = 0; Text_Index < (width * height);
       ++Text_Index) // 0 ~ 480*640
  {
    rs2::texture_coordinate Texture_XY =
        Texture[Text_Index]; // index 0 ~ 480*640
    int x_value =
        std::min(std::max(int(Texture_XY.u * width + .5f), 0), width - 1);
    int y_value =
        std::min(std::max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int i = Text_Index % width; // /640
    int j = Text_Index / width;

    match_points1.clear();
    match_points2.clear();
    match_points1.resize(good_matches.size());
    match_points2.resize(good_matches.size());
    for (int num_matched = 0;
         (num_matched < good_matches.size()) && (num_matched < 100);
         num_matched++)
    {
      if ((int(kpts_01[good_matches[num_matched].queryIdx].pt.x) == x_value) &&
          (int(kpts_01[good_matches[num_matched].queryIdx].pt.y) == y_value))
      {
        match_points1.at(num_matched).x = i;
        match_points1.at(num_matched).y = j;

//        match_pointcloud[0][num_matched][0] = i;
//        match_pointcloud[0][num_matched][1] = j;
      }
      if ((int(kpts_02[good_matches[num_matched].trainIdx].pt.x) == x_value) &&
          (int(kpts_02[good_matches[num_matched].trainIdx].pt.y) == y_value))
      {

        match_points2.at(num_matched).x = i;
        match_points2.at(num_matched).y = j;
//        match_pointcloud[1][num_matched][0] = i;
//        match_pointcloud[1][num_matched][1] = j;
      }
    }
  }

  std::printf("final matched and transformed point pairs num: %zu , %zu", match_points1.size(), match_points2.size());
  return true;
}
