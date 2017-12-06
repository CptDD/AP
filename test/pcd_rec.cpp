#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>


using namespace std;

int main(int argc,char**argv)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>());

	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normal_estimator;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr vfh_tree(new pcl::search::KdTree<pcl::PointXYZ>);



	if(reader.read(argv[1],*cloud)==-1)
	{
		cout<<"An error has occured!"<<endl;
		return -1;
	}

	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);

	cout<<"The cloud has :"<<cloud->points.size()<<endl;
	cout<<"The cloud has :"<<normals->points.size()<<endl;

	pcl::VFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::VFHSignature308> vfh;
	vfh.setInputCloud(cloud);
	vfh.setInputNormals(normals);
	vfh.setSearchMethod(vfh_tree);

	cout<<"Computing the vfh descriptors . . ."<<endl;
	vfh.compute(*vfhs);
	cout<<"Features computed!"<<endl;

	cout<<vfhs->points.size()<<endl;

	writer.write("go.pcd",*vfhs);


	return 0;
}


/*{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
  	pcl::PCDReader reader;

  	if(reader.read(argv[1],*cloud)==-1)
  	{
  		cout<<"There is an error while reading the cloud!"<<endl;
  		return -1;
  	}

  	
  	pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  	vfh.setInputCloud (cloud);
  	vfh.setInputNormals (normals);

	return 0;
}*/