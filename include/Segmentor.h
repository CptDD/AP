#ifndef H_SEGMENTOR
#define H_SEGMENTOR

#include "CloudNode.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>



class Segmentor
{
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr main_plane;
	pcl::PointCloud<pcl::PointXYZ>::Ptr current_outliers;
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud;


	pcl::PointCloud<pcl::Normal>::Ptr main_plane_normals;
	pcl::PointCloud<pcl::Normal>::Ptr sphere_normals;
	pcl::PointCloud<pcl::Normal>::Ptr cylinder_normals;

	pcl::PointIndices::Ptr main_plane_inliers;
	pcl::PointIndices::Ptr sphere_inliers;
	pcl::PointIndices::Ptr cylinder_indices;

	pcl::ModelCoefficients::Ptr main_plane_coefficients;
	pcl::ModelCoefficients::Ptr sphere_coefficients;
	pcl::ModelCoefficients::Ptr cylinder_coefficients;

	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normal_estimator;
	pcl::ExtractIndices<pcl::Normal> normal_extractor;
  	pcl::ExtractIndices<pcl::PointXYZ> point_extractor;


  	pcl::SACSegmentationFromNormals<pcl::PointXYZ,pcl::Normal> segmentor;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr sphere_tree;	

	void filter_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

public:
	Segmentor(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud):
	main_plane(new pcl::PointCloud<pcl::PointXYZ>),
	cylinder_cloud(new pcl::PointCloud<pcl::PointXYZ>),
	current_outliers(new pcl::PointCloud<pcl::PointXYZ>),
	filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>),
	sphere_cloud(new pcl::PointCloud<pcl::PointXYZ>),
	main_plane_normals(new pcl::PointCloud<pcl::Normal>),
	sphere_normals(new pcl::PointCloud<pcl::Normal>),
	cylinder_normals(new pcl::PointCloud<pcl::Normal>),
	main_plane_inliers(new pcl::PointIndices),
	sphere_inliers(new pcl::PointIndices),
	cylinder_indices(new pcl::PointIndices),
	main_plane_coefficients(new pcl::ModelCoefficients),
	sphere_coefficients(new pcl::ModelCoefficients),
	cylinder_coefficients(new pcl::ModelCoefficients),
	tree(new pcl::search::KdTree<pcl::PointXYZ>()),
	sphere_tree(new pcl::search::KdTree<pcl::PointXYZ>())
	{
		this->segment_main_plane(input_cloud);
	};

	void segment_main_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
	void segment_sphere();
	void segment_cylinder();
	std::vector<CloudNode> segment_cylinders();
	std::vector<CloudNode> segment_cylinders(CloudNode node,double distance_thr);


	CloudNode get_main_plane();
	CloudNode get_filtered_cloud();
	CloudNode get_sphere_cloud();
	CloudNode get_current_outliers();
	CloudNode get_cylinder_cloud();

};
#endif