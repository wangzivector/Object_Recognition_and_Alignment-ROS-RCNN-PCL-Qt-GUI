#include <string>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>		  
#include <vector>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
 
using namespace std;
typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
 
int main(int argc,char **argv)
{
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr cloud_normal(new pcl::PointCloud<NormalType>());
 
	std::string fileName(argv[1]);
	pcl::io::loadPCDFile(fileName, *cloud);
 
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	//eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
 
	//std::cout << eigenValuesPCA << std::endl;
	//std::cout << eigenVectorsPCA << std::endl;
 
	//初始位置时的主方向
	PointType c;
	c.x = pcaCentroid(0);
	c.y = pcaCentroid(1);
	c.z = pcaCentroid(2);
	PointType pcZ;
	pcZ.x = eigenVectorsPCA(0, 0);
	pcZ.y = eigenVectorsPCA(1, 0);
	pcZ.z = eigenVectorsPCA(2, 0);
	PointType pcY;
	pcY.x = eigenVectorsPCA(0, 1);
	pcY.y = eigenVectorsPCA(1, 1);
	pcY.z = eigenVectorsPCA(2, 1);
	PointType pcX;
	pcX.x = eigenVectorsPCA(0, 2);
	pcX.y = eigenVectorsPCA(1, 2);
	pcX.z = eigenVectorsPCA(2, 2);

	std::cout << "point c :\n" << c << std::endl;
	std::cout << pcX << std::endl;
	std::cout << pcY << std::endl;
	std::cout << pcZ << std::endl;

	//x axis
	for(int count=0;count<60;count++)
	{
		pcl::PointXYZRGB Point;
		Point.x=c.x+pcX.x*0.005*count;
		Point.y=c.y+pcX.y*0.005*count;
		Point.z=c.z+pcX.z*0.005*count;
		Point.r=255;
		Point.g=0;
		Point.b=0;
		//std::cout << Point << std::endl;
		cloud->points.push_back(Point);
		cloud->width ++;
	}

	//y axis
	for(int count=0;count<40;count++)
	{
		pcl::PointXYZRGB Point;
		Point.x=c.x+pcY.x*0.005*count;
		Point.y=c.y+pcY.y*0.005*count;
		Point.z=c.z+pcY.z*0.005*count;
		Point.r=0;
		Point.g=255;
		Point.b=0;
		//std::cout << Point << std::endl;
		cloud->points.push_back(Point);
		cloud->width ++;
	}

	//z axis
	for(int count=0;count<40;count++)
	{
		pcl::PointXYZRGB Point;
		Point.x=c.x+pcZ.x*0.005*count;
		Point.y=c.y+pcZ.y*0.005*count;
		Point.z=c.z+pcZ.z*0.005*count;
		Point.r=0;
		Point.g=0;
		Point.b=255;
		//std::cout << Point << std::endl;
		cloud->points.push_back(Point);
		cloud->width ++;
	}
    //save pc
	pcl::io::savePCDFileASCII("pcdsave.pcd",*cloud);
    std::cout << "done save file" << std::endl;
	

	return 0;
}
