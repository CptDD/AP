#include <iostream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Geometry>
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/common_headers.h>
#include <pcl/segmentation/sac_segmentation.h>




using namespace std;



int main(int argc,char**argv)
{
	cout<<"We are here to make some noise!"<<endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr current_outliers(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::Normal>::Ptr sphere_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr sphere_coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr sphere_inliers(new pcl::PointIndices);

	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normal_estimator;
	pcl::ExtractIndices<pcl::Normal> normal_extractor;
  	pcl::ExtractIndices<pcl::PointXYZ> point_extractor;

  	pcl::SACSegmentationFromNormals<pcl::PointXYZ,pcl::Normal> segmentor;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr sphere_tree(new pcl::search::KdTree<pcl::PointXYZ>());
 
	pcl::PCDReader reader;
	pcl::PCDWriter writer;

	if(reader.read(argv[1],*cloud)==-1)
	{
		cout<<"An error has occured while reading the point cloud!"<<endl;
		return -1;
	}

	cout<<"Cloud has :"<<cloud->points.size()<<endl;


	normal_estimator.setSearchMethod(sphere_tree);
	normal_estimator.setKSearch(50);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.compute(*sphere_normals);

    segmentor.setOptimizeCoefficients(true);
	segmentor.setModelType(pcl::SACMODEL_SPHERE);
	segmentor.setMethodType(pcl::SAC_RANSAC);
	segmentor.setNormalDistanceWeight (0.01);
	segmentor.setMaxIterations (10000);
    segmentor.setDistanceThreshold (0.03);
    segmentor.setRadiusLimits (0.05, 0.15);
	segmentor.setInputCloud(cloud);
	segmentor.setInputNormals (sphere_normals);
	segmentor.segment(*sphere_inliers,*sphere_coefficients);

	point_extractor.setInputCloud(cloud);
	point_extractor.setIndices(sphere_inliers);
	point_extractor.setNegative(false);
	point_extractor.filter(*sphere_cloud);

	point_extractor.setNegative(true);
	point_extractor.filter(*cloud);


	writer.write("result.pcd",*cloud);




	return 0;

}