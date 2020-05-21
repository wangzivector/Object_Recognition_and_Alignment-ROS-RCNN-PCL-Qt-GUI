/*
 * gui_node.cpp
 *
 *  Created on: May 13, 2020
 *      Author: wangzivector
 * pcd file realated and io , realsense
 */
#include "pcd_io.h"

pcd_io::pcd_io() {}

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
    cfg.enable_stream(RS2_STREAM_COLOR, frame_width, frame_height,
                      RS2_FORMAT_BGR8, frame_rate);
    cfg.enable_stream(RS2_STREAM_INFRARED, frame_width, frame_height,
                      RS2_FORMAT_Y8, frame_rate);
    cfg.enable_stream(RS2_STREAM_DEPTH, frame_width, frame_height,
                      RS2_FORMAT_Z16, frame_rate);

    // rs2::pipeline_profile selection = pipe.start(cfg);
    pipe_point->start(cfg);
  }
  catch (std::exception& e)
  {
    std::cout << "open realsense error occurs : " << e.what() << std::endl;
    return false;
  }
  std::cout << "** wait for frame settle **   \n";
  for (int i = 0; i < 10; i++)
    auto frames =
        pipe_point->wait_for_frames(); // Drop several frames for auto-exposure
  return true;
}

PointCloud::Ptr pcd_io::readFrameRS()
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
  return PCL_Conversion(*points_pointer, color); // get PCL pointcloud type
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

bool pcd_io::pcdSave(std::string pcd_path, PointCloud::Ptr cloud)
{
  if (pcl::io::savePCDFileASCII(pcd_path, *cloud) == -1)
  {
    PCL_ERROR("Couldn't save file %s \n", pcd_path.c_str());
    return false;
  }
  return true;
}

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
  PointCloud::Ptr cloud(new PointCloud);

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

  return cloud; /// PCL RGB Point Cloud generated
}

bool pcd_io::copyPointRGBNormalToPointRGB(PointRGBNormalCloud::Ptr cloud_in, PointCloud::Ptr cloud_out)
{
  cloud_out->points.resize(cloud_in->size());
  std::cout << "start copyPointRGBNormalToPointRGB" << std::endl;
  for (size_t i = 0; i < cloud_in->points.size(); i++) {
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
             << "  width/height :" << cloud_out->width << cloud_out->height << std::endl;
  return true;
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
