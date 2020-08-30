/*
 * gui_node.cpp
 *
 *  Created on: May 13, 2020
 *      Author: wangzivector
 * pcd file realated and io , realsense
 */
#include "pcd_io.h"

pcd_io::pcd_io()
{
  mask = cv::Mat(640, 480, CV_8UC3);
  image_origin = cv::Mat(640, 480, CV_8UC3);
  mask_color = std::tuple<uchar, uchar, uchar>(255, 255, 255);
}

bool pcd_io::pcdRead(std::string pcd_path, PointCloud::Ptr cloud)
{
  /// read cloud from pcd_path deliver from param
  if (pcl::io::loadPCDFile(pcd_path, *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file %s \n", pcd_path.c_str());
    return false;
  }
  return true;
}

//===================================================
//  pcdSave
//===================================================
bool pcd_io::pcdSave(std::string pcd_path, PointCloud::Ptr cloud)
{
  if (cloud->size() == 0)
    std::cout << "saving cloud is 0 size ! failed saveed." << std::endl;
  if (pcl::io::savePCDFileASCII(pcd_path, *cloud) == -1)
  {
    PCL_ERROR("Couldn't save file %s \n", pcd_path.c_str());
    return false;
  }
  return true;
}

//===================================================
//  maskImplement
//  crop the organized cloud based mask
//===================================================
bool pcd_io::maskImplement(PointCloud::Ptr input_cloud,
                           PointCloud::Ptr out_cloud, cv::Mat mask_img,
                           std::tuple<uchar, uchar, uchar> mask_rgb)
{
  /// operate this could make the mask avalible
  RGB_Texture_mask(mask_img, Texture);
  /// if not organized cloud, you can't indese it by cloud[][]
  if (input_cloud->height > 1)
  {

    /// if mask img match cloud
    if ((mask_img.rows == input_cloud->height) &&
        (mask_img.cols == input_cloud->width))
    {
      std::cout << "image size is : " << mask_img.rows << "/" << mask_img.cols
                << std::endl;
    }
    else
    {
      std::cout << "rows =/= height or cols =/= width: " << mask_img.rows
                << input_cloud->height << "  " << mask_img.cols
                << input_cloud->width << std::endl;
      return false;
    }
  }
  else
  {
    std::cout << "this is not a organized pointcloud\n" << std::endl;
    return false;
  }

  cv::Mat img = mask.clone();
  /// mask is the transformed maskimg
  PointCloud::Ptr output_cloud = PointCloud::Ptr(new PointCloud());

  /// mask_rgb is mask color, if match ,then take the point in same position
  uchar b = std::get<2>(mask_rgb);
  uchar g = std::get<1>(mask_rgb);
  uchar r = std::get<0>(mask_rgb);
  for (int i = 0; i < img.rows; i++)
  {
    for (int j = 0; j < img.cols; j++)
    {
      if ((img.at<cv::Vec3b>(i, j)[0] == b) &&
          (img.at<cv::Vec3b>(i, j)[1] == g) &&
          (img.at<cv::Vec3b>(i, j)[2] == r))
      {
        output_cloud->push_back(input_cloud->at(j, i));
//        output_cloud->points[output_cloud->size() - 1].r = 255;
//        output_cloud->points[output_cloud->size() - 1].g = 0;
//        output_cloud->points[output_cloud->size() - 1].b = 0;
      }
    }
  }
  pcl::copyPointCloud(*output_cloud, *out_cloud);
  std::cout << "cloud size :" << out_cloud->height << out_cloud->width
            << std::endl;
  return true;
}

//===================================================
//  maskExample
//  create an example mask img
//===================================================
cv::Mat pcd_io::maskExample(int row, int col, uchar intensity)
{
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
        img.at<cv::Vec3b>(i, j)[0] = 255;
        img.at<cv::Vec3b>(i, j)[1] = 255;
        img.at<cv::Vec3b>(i, j)[2] = 255;
      }
  //  cv::imwrite("/home/wang/catkin_qtws/img.jpg", img);
  mask = img.clone();
  mask_color = std::tuple<uchar, uchar, uchar>(intensity, intensity, intensity);
  return img;
}


//===================================================
//  realsenseInit
//  create a pipeline for frame read from kinetic
//===================================================
bool pcd_io::realsenseInit()
{
  try
  {
    /// Declare pointcloud object, for calculating pointclouds and texture
    /// mappings
    pc_pointer = new rs2::pointcloud;
    /// We want the points object to be persistent so we can display the last
    /// cloud when a frame drops
    points_pointer = new rs2::points;
    /// Declare RealSense pipeline, encapsulating the actual device
    pipe_point = new rs2::pipeline();
    /// Start streaming with default recommended configuration
    /// Create a configuration for configuring the pipeline with a non default
    /// profile
    rs2::config cfg;
    ///======================
    /// Stream configuration
    ///======================
    int frame_width = 640;
    int frame_height = 480;
    int frame_rate = 10;

    /// config sensor
    cfg.enable_stream(RS2_STREAM_COLOR, frame_width, frame_height,
                      RS2_FORMAT_BGR8, frame_rate);
    cfg.enable_stream(RS2_STREAM_INFRARED, frame_width, frame_height,
                      RS2_FORMAT_Y8, frame_rate);
    cfg.enable_stream(RS2_STREAM_DEPTH, frame_width, frame_height,
                      RS2_FORMAT_Z16, frame_rate);

    pipe_point->start(cfg);

    for (int i = 0; i < 10; i++)
      auto frames =
          pipe_point
              ->wait_for_frames(); // Drop several frames for auto-exposure
  }
  /// if there error, catch and don't crash the exe
  /// like no kinetic connected yet or not detach well
  catch (std::exception& e)
  {
    std::cout << "open realsense error occurs : " << e.what() << std::endl;
    return false;
  }
  std::cout << "have wait for frame settle. got sensor.\n";
  return true;
}

//===================================================
//  readFrameRS
//  form kinetic, note that get cloud and RGB img
//===================================================
bool pcd_io::readFrameRS(PointCloud::Ptr cloud, cv::Mat& img)
{
  try
  {
    auto frames = pipe_point->wait_for_frames();
    auto color = frames.get_color_frame();
    if (!color)
      color = frames.get_infrared_frame();

    /// Tell pointcloud object to map to this color frame
    pc_pointer->map_to(color);
    auto depth = frames.get_depth_frame();

    /// Generate the pointcloud and texture mappings
    *points_pointer = pc_pointer->calculate(depth);
    PointCloud::Ptr cloud_get =
        PCL_Conversion(*points_pointer, color); /// get PCL pointcloud type
    pcl::copyPointCloud(*cloud_get, *cloud);

    //
    // get img frame
    //
    img =
        cv::Mat(cloud->height, cloud->width, CV_8UC3, (void*)color.get_data());
    cv::cvtColor(img, img, CV_RGB2BGR);
  }
  /// if there error, catch and don't crash the exe
  /// like no kinetic connected yet or not detach well
  catch (std::exception& e)
  {
    std::cout << "open realsense error occurs : " << e.what() << std::endl;
    return false;
  }
  std::cout << "have wait for frame settle. got sensor.\n";
  return true;
}

//===================================================
//  releasePipe
//  release sensor if no use.
//===================================================
void pcd_io::releasePipe() { pipe_point->stop(); }

using namespace std;

//===================================================
//  PCL_Conversion
// - Function is utilized to fill a point cloud
//  object with depth and RGB data from a single
//  frame captured using the Realsense.
//===================================================
PointCloud::Ptr pcd_io::PCL_Conversion(const rs2::points& points,
                                       const rs2::video_frame& color)
{
  /// input param is points and RGB ,combine it to return

  /// Object Declaration (Point Cloud)
  PointCloud::Ptr cloud = PointCloud::Ptr(new PointCloud());

  /// Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
  std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

  //================================
  // PCL Cloud Object Configuration
  //================================
  /// Convert data captured from Realsense camera to Point Cloud
  auto sp = points.get_profile().as<rs2::video_stream_profile>();

  cloud->width = static_cast<uint32_t>(sp.width());
  cloud->height = static_cast<uint32_t>(sp.height());
  cloud->is_dense = false;
  cloud->points.resize(points.size());

  auto Texture_Coord = points.get_texture_coordinates();
  Texture = Texture_Coord;
  auto Vertex = points.get_vertices();

  /// Iterating through all points and setting XYZ coordinates
  /// and RGB values
  for (int i = 0; i < points.size(); i++)
  {
    //===================================
    // Mapping Depth Coordinates
    // - Depth data stored as XYZ values
    //===================================
    cloud->points[i].x = Vertex[i].x;
    cloud->points[i].y = Vertex[i].y;
    cloud->points[i].z = Vertex[i].z;

    /// Obtain color texture for specific point
    RGB_Color = RGB_Texture(color, Texture_Coord[i]);

    /// Mapping Color (BGR due to Camera Model)
    cloud->points[i].r = std::get<2>(RGB_Color); // Reference tuple<2>
    cloud->points[i].g = std::get<1>(RGB_Color); // Reference tuple<1>
    cloud->points[i].b = std::get<0>(RGB_Color); // Reference tuple<0>
  }
  return cloud;
}

//======================================================
// RGB Texture
// - Function is utilized to extract the RGB data from
// a single point return R, G, and B values.
// Normals are stored as RGB components and
// correspond to the specific depth (XYZ) coordinate.
// By taking these normals and converting them to
// texture coordinates, the RGB components can be
// "mapped" to each individual point (XYZ).
//======================================================
std::tuple<int, int, int>
pcd_io::RGB_Texture(rs2::video_frame texture,
                    rs2::texture_coordinate Texture_XY)
{

  /// Get Width and Height coordinates of texture
  int width = texture.get_width();   // Frame width in pixels
  int height = texture.get_height(); // Frame height in pixels

  /// Normals to Texture Coordinates conversion
  int x_value = min(max(int(Texture_XY.u * width + .5f), 0), width - 1);
  int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

  int bytes =
      x_value * texture.get_bytes_per_pixel(); // Get # of bytes per pixel
  int strides =
      y_value * texture.get_stride_in_bytes(); // Get line width in bytes
  int Text_Index = (bytes + strides);

  const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());

  /// RGB components to save in tuple
  int NT1 = New_Texture[Text_Index];
  int NT2 = New_Texture[Text_Index + 1];
  int NT3 = New_Texture[Text_Index + 2];

  return std::tuple<int, int, int>(NT1, NT2, NT3);
}

//======================================================
// RGB Texture_mask
// modify for color
//======================================================
cv::Mat pcd_io::RGB_Texture_mask(cv::Mat origin,
                                 const rs2::texture_coordinate* Texture)
{
  /// Get Width and Height coordinates of texture
  int width = origin.cols;//640                            // Frame width in pixels
  int height = origin.rows;//480                           // Frame height in pixels
  cv::Mat mask_img = cv::Mat(height, width, CV_8UC3); //(480 640)
  for (int Text_Index = 0; Text_Index < (width * height); ++Text_Index)// 0 ~ 480*640
  {
    rs2::texture_coordinate Texture_XY = Texture[Text_Index];// index 0 ~ 480*640
    int x_value = min(max(int(Texture_XY.u * width + .5f), 0), width - 1);
    int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int i = Text_Index % width; // /640
    int j = Text_Index / width;
    mask_img.at<cv::Vec3b>(j, i)[0] = origin.at<cv::Vec3b>(y_value, x_value)[2];// NEW(Text_Index_xy) = OLD(xy_value)
    mask_img.at<cv::Vec3b>(j, i)[1] = origin.at<cv::Vec3b>(y_value, x_value)[1];
    mask_img.at<cv::Vec3b>(j, i)[2] = origin.at<cv::Vec3b>(y_value, x_value)[0];
    //    if(i==j*2) printf("#debug#:Text_Index(%d) (%d,%d): x_value/y_value:%d
    //    / %d \n",Text_Index ,j ,i ,y_value,x_value);
  }
  /// Normals to Texture Coordinates conversion
  //  int x_value = min(max(int(Texture_XY.u * width + .5f), 0), width - 1);
  //  int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

  //  const auto New_Texture = reinterpret_cast<const
  //  uint8_t*>(origin.get_data());
  mask = mask_img.clone();

  return mask_img;
}

//===================================================
//  copyPointRGBNormalToPointRGB
//  special fun for extract cloud from this type
//===================================================
bool pcd_io::copyPointRGBNormalToPointRGB(PointRGBNormalCloud::Ptr cloud_in,
                                          PointCloud::Ptr cloud_out)
{
  cloud_out->points.resize(cloud_in->size());
  std::cout << "start copyPointRGBNormalToPointRGB" << std::endl;
  for (size_t i = 0; i < cloud_in->points.size(); i++)
  {
    cloud_out->points[i].x = cloud_in->points[i].x;
    cloud_out->points[i].y = cloud_in->points[i].y;
    cloud_out->points[i].z = cloud_in->points[i].z;
    cloud_out->points[i].r = cloud_in->points[i].r;
    cloud_out->points[i].g = cloud_in->points[i].g;
    cloud_out->points[i].b = cloud_in->points[i].b;
  }
  cloud_out->width = cloud_in->width;
  cloud_out->height = cloud_in->height;
  std::cout << "done copyPointRGBNormalToPointRGB : " << cloud_out->size()
            << "  width/height :" << cloud_out->width << cloud_out->height
            << std::endl;
  return true;
}
