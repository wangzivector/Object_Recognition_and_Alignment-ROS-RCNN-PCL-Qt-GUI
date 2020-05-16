#include "objreco.h"

ObjReco::ObjReco()
{
  cloud = PointCloud::Ptr(new PointCloud());
  cloud_pointRGBNormal = PointRGBNormalCloud::Ptr(new PointRGBNormalCloud());
}

//===================================================
//  checkReconstruction
//  take a look whether recongstruction works well
//===================================================
bool ObjReco::checkReconstruction()
{
  if (cloud->size() == 0)
  {
    std::cout << "processing pointcloud doest be loaded yet.";
    return false;
  }
  return mlsReconstruction(cloud, cloud_pointRGBNormal);
}
