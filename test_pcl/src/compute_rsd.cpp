#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/features/rsd.h>
#include <pcl/features/normal_3d.h>


void
computeRSD (const boost::filesystem::path &path)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
  pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr descriptors (new pcl::PointCloud<pcl::PrincipalRadiiRSD>);

  pcl::io::loadPCDFile (path.string (), *cloud);

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.01);

  // Compute the features
  ne.compute (*normals);

  pcl::PrincipalRadiiRSD r;
  

  /*pcl::RSDEstimation<pcl::PointXYZ,pcl::Normal,pcl::PrincipalRadiiRSD> rsd_estimator;

  rsd_estimator.setInputCloud(cloud);
  rsd_estimator.setInputNormals(normals);

  rsd_estimator.compute(*descriptors);*/



  // Output datasets
  /*pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr rsds(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());
  

  std::string filename = std::string(path.string ());
  boost::replace_all(filename, ".pcd", "");
  filename = filename + std::string("_rsd.pcd");

  // Save the features
  pcl::io::savePCDFile (filename, *rsds);*/
}


int main(int argc,char**argv)
{ 
	computeRSD(argv[1]);
  return 0;
}

