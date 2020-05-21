/*
 * gui_node.cpp
 *
 *  Created on: May 13, 2020
 *      Author: wangzivector
 *  pcd realated
 */
#ifndef PCD_IO_H
#define PCD_IO_H

/// Include RealSense Cross Platform API
#include <librealsense2/rs.hpp>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZRGB PointType;            ///接收点云的格式
typedef pcl::Normal NormalType;                ///点云法向量格式
typedef pcl::PointCloud<PointType> PointCloud; ///接收点云的”存储“格式
typedef pcl::PointCloud<NormalType> NormalCloud; ///接收点云法向量存储格式
typedef pcl::PointXYZRGBNormal PointRGBNormalType; /// resonstruction
typedef pcl::PointCloud<PointRGBNormalType> PointRGBNormalCloud;


class pcd_io
{
public:
  pcd_io();
  bool realsenseInit();
  PointCloud::Ptr readFrameRS();
  bool copyPointRGBNormalToPointRGB(PointRGBNormalCloud::Ptr cloud_in, PointCloud::Ptr cloud_out);
  bool pcdRead(std::string path, PointCloud::Ptr);
  bool pcdSave(std::string pcd_path, PointCloud::Ptr cloud);

private:
  PointCloud::Ptr PCL_Conversion(const rs2::points& points,
                                 const rs2::video_frame& color);
  std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture,
                                        rs2::texture_coordinate Texture_XY);
  /// Declare pointcloud object, for calculating pointclouds and texture
  /// mappings
  rs2::pointcloud* pc_pointer;
  /// We want the points object to be persistent so we can display the last
  /// cloud when a frame drops
  rs2::points* points_pointer;
  rs2::pipeline* pipe_point;
};

#endif // PCD_IO_H
