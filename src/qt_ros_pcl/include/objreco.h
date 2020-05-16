#ifndef OBJRECO_H
#define OBJRECO_H
#include "qpcl.h"
#include "pcd_io.h"

class ObjReco : public qpcl, public pcd_io
{
public:
  ObjReco();
  bool checkReconstruction();

  PointCloud::Ptr cloud;
  PointRGBNormalCloud::Ptr cloud_pointRGBNormal;
};

#endif // OBJRECO_H
