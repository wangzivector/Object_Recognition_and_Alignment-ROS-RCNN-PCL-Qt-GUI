/*
 * qpcl.h
 *
 *  Created on: May 8, 2020
 *      Author: wangzivector
 * pointcloud process in this file, including create/read downsample and so on
 */

#ifndef QPCL_H
#define QPCL_H
/// pointcloud headfile
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class qpcl
{
public:
  qpcl();
};

#endif // QPCL_H
