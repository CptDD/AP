#include "CloudNode.h"
#include "Segmentor.h"
#include "Viewer.h"

#include <cmath>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/sac_model_sphere.h>



void Segmentor::filter_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
	pcl::PassThrough<pcl::PointXYZ> pass_filter;
	pass_filter.setInputCloud(input_cloud);
	pass_filter.setFilterFieldName("x");
	pass_filter.setFilterLimits(-2.4,0.2);
	pass_filter.filter(*filtered_cloud);

	pass_filter.setInputCloud(filtered_cloud);
	pass_filter.setFilterFieldName("y");
	pass_filter.setFilterLimits(1.5,4.8);
	pass_filter.filter(*filtered_cloud);

	normal_estimator.setSearchMethod(tree);
  	normal_estimator.setInputCloud(filtered_cloud);
  	normal_estimator.setKSearch(50);
  	normal_estimator.compute(*main_plane_normals);

  	 segmentor.setOptimizeCoefficients(true);
  	 segmentor.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  	 segmentor.setMethodType(pcl::SAC_RANSAC);
  	 segmentor.setNormalDistanceWeight(0.01);
     segmentor.setDistanceThreshold(0.03);
	 segmentor.setInputCloud(filtered_cloud);
     segmentor.setInputNormals(main_plane_normals);
	 segmentor.segment(*main_plane_inliers,*main_plane_coefficients);


	 point_extractor.setInputCloud(filtered_cloud);
	 point_extractor.setNegative(false);
	 point_extractor.setIndices(main_plane_inliers);
	 point_extractor.filter(*main_plane);

	 point_extractor.setNegative(true);
	 point_extractor.filter(*current_outliers);
}


void Segmentor::segment_main_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
	this->filter_cloud(input_cloud);
}

void Segmentor::segment_sphere()
{
	normal_estimator.setSearchMethod(sphere_tree);
	normal_estimator.setKSearch(50);
    normal_estimator.setInputCloud(current_outliers);
    normal_estimator.compute(*sphere_normals);

    segmentor.setOptimizeCoefficients(true);
	segmentor.setModelType(pcl::SACMODEL_SPHERE);
	segmentor.setMethodType(pcl::SAC_RANSAC);
	segmentor.setNormalDistanceWeight (0.01);
	segmentor.setMaxIterations (10000);
    segmentor.setDistanceThreshold (0.03);
    segmentor.setRadiusLimits (0.05, 0.8);
	segmentor.setInputCloud(current_outliers);
	segmentor.setInputNormals (sphere_normals);
	segmentor.segment(*sphere_inliers,*sphere_coefficients);

	point_extractor.setInputCloud(current_outliers);
	point_extractor.setIndices(sphere_inliers);
	point_extractor.setNegative(false);
	point_extractor.filter(*sphere_cloud);

	point_extractor.setNegative(true);
	point_extractor.filter(*current_outliers);

}

void Segmentor::segment_cylinder()
{
	normal_estimator.setSearchMethod(tree);
  	current_outliers->points.size()>0?normal_estimator.setInputCloud(current_outliers):normal_estimator.setInputCloud(filtered_cloud);
  	normal_estimator.setKSearch(50);
  	normal_estimator.compute(*cylinder_normals);

  	segmentor.setOptimizeCoefficients(true);
  	segmentor.setModelType (pcl::SACMODEL_CYLINDER);
  	segmentor.setMethodType (pcl::SAC_RANSAC);
  	segmentor.setNormalDistanceWeight (0.01);
  	segmentor.setMaxIterations (10000);
  	segmentor.setDistanceThreshold(0.02);
  	segmentor.setRadiusLimits(0.05,0.15);
  	segmentor.setInputNormals(cylinder_normals);
  	current_outliers->points.size()>0?segmentor.setInputCloud(current_outliers):segmentor.setInputCloud(filtered_cloud);
  	segmentor.segment(*cylinder_indices,*cylinder_coefficients);

  	current_outliers->points.size()>0?point_extractor.setInputCloud(current_outliers):point_extractor.setInputCloud(filtered_cloud);
  	point_extractor.setIndices(cylinder_indices);
	point_extractor.setNegative(false);
	point_extractor.filter(*cylinder_cloud);

	point_extractor.setNegative(true);
	point_extractor.filter(*current_outliers);

}	


std::pair<pcl::PointXYZ,pcl::PointXYZ> Segmentor::get_segment_heads(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	std::pair<pcl::PointXYZ,pcl::PointXYZ> heads;

	heads.first=cloud->points[0];
	heads.second=cloud->points[0];

	for(int i=1;i<cloud->points.size();i++)
	{
		if(cloud->points[i].x<heads.first.x)
		{
			heads.first=cloud->points[i];
		}

		if(cloud->points[i].x>heads.second.x)
		{
			heads.second=cloud->points[i];
		}
	}
	return heads;
}

double Segmentor::compute_distance(CloudNode node,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	std::pair<pcl::PointXYZ,pcl::PointXYZ> heads=this->get_segment_heads(cloud);

	double xdiff=heads.second.x-heads.first.x;
	double ydiff=heads.second.y-heads.first.y;
	double zdiff=heads.second.z-heads.first.z;

	double distance=sqrt(xdiff*xdiff+ydiff*ydiff+zdiff*zdiff);

	CloudNodeViewer viewer;
	viewer.view_line_test(node,heads.first,heads.second);

	return distance;
}

double Segmentor::compute_length(CloudNode node)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::ModelCoefficients::Ptr cylinder_coefficients(new pcl::ModelCoefficients);

	cylinder_coefficients->values.push_back(node.get_point_axis().x());	
	cylinder_coefficients->values.push_back(node.get_point_axis().y());
	cylinder_coefficients->values.push_back(node.get_point_axis().z());	
	cylinder_coefficients->values.push_back(node.get_axis_direction().x());
	cylinder_coefficients->values.push_back(node.get_axis_direction().y());
	cylinder_coefficients->values.push_back(node.get_axis_direction().z());

	pcl::ProjectInliers<pcl::PointXYZ> projections;
	projections.setModelType(pcl::SACMODEL_LINE);
	projections.setInputCloud(node.get_cloud());
	projections.setModelCoefficients(cylinder_coefficients);	
	projections.filter(*cloud);

	double distance=this->compute_distance(node,cloud);


	return distance;
}



std::vector<CloudNode> Segmentor::segment_cylinders()
{

	int nr_of_points;
	bool current_filtered=false;

	std::vector<CloudNode> cylinders;

	if(current_outliers->points.size()>0)
	{
		current_filtered=true;
		nr_of_points=current_outliers->points.size();
	}else
	{
		nr_of_points=filtered_cloud->points.size();
	}

	CloudNodeViewer v;

	if(current_filtered)
	{
		while(current_outliers->points.size()>0.2*nr_of_points)
		{
			normal_estimator.setSearchMethod(tree);
  			normal_estimator.setInputCloud(current_outliers);
  			normal_estimator.setKSearch(50);
  			normal_estimator.compute(*cylinder_normals);

  			segmentor.setOptimizeCoefficients (true);
  			segmentor.setModelType (pcl::SACMODEL_CYLINDER);
  			segmentor.setMethodType (pcl::SAC_RANSAC);
  			segmentor.setNormalDistanceWeight (0.01);
  			segmentor.setMaxIterations (10000);
  			segmentor.setDistanceThreshold(0.02);
  			segmentor.setRadiusLimits(0.05,0.15);
  			segmentor.setInputCloud (current_outliers);
    		segmentor.setInputNormals (cylinder_normals);
    		segmentor.segment(*cylinder_indices,*cylinder_coefficients);

    		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cylinder(new pcl::PointCloud<pcl::PointXYZ>);

    		point_extractor.setIndices(cylinder_indices);
    		point_extractor.setInputCloud(current_outliers);
			point_extractor.setNegative(false);
			point_extractor.filter(*temp_cylinder);

			CloudNode temp_node(temp_cylinder);
			temp_node.set_radius(cylinder_coefficients->values[6]);

			cylinders.push_back(temp_node);
		
			v.view(temp_node);

			pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>);
			point_extractor.setNegative(true);
			point_extractor.filter(*input_cloud_outliers);
			current_outliers.swap(input_cloud_outliers);
		}

	}else
	{
		while(filtered_cloud->points.size()>0.4*nr_of_points)
		{
			normal_estimator.setSearchMethod(tree);
  			normal_estimator.setInputCloud(filtered_cloud);
  			normal_estimator.setKSearch(50);
  			normal_estimator.compute(*cylinder_normals);

  			segmentor.setOptimizeCoefficients (true);
  			segmentor.setModelType (pcl::SACMODEL_CYLINDER);
  			segmentor.setMethodType (pcl::SAC_RANSAC);
  			segmentor.setNormalDistanceWeight (0.01);
  			segmentor.setMaxIterations (10000);
  			segmentor.setDistanceThreshold(0.02);
  			segmentor.setRadiusLimits(0.05,0.15);
  			segmentor.setInputCloud (filtered_cloud);
    		segmentor.setInputNormals (cylinder_normals);
    		segmentor.segment(*cylinder_indices,*cylinder_coefficients);

    		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cylinder(new pcl::PointCloud<pcl::PointXYZ>);

    		point_extractor.setIndices(cylinder_indices);
			point_extractor.setNegative(false);
			point_extractor.setInputCloud(filtered_cloud);
			point_extractor.filter(*temp_cylinder);

			CloudNode temp_node(temp_cylinder);
			temp_node.set_radius(cylinder_coefficients->values[6]);

			cylinders.push_back(temp_node);

			pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>);
			point_extractor.setNegative(true);
			point_extractor.filter(*input_cloud_outliers);
			filtered_cloud.swap(input_cloud_outliers);
		}
	}

	return cylinders;	
}

std::vector<CloudNode> Segmentor::segment_cylinders(CloudNode node,double min_r,double max_r,double distance_thr)
{
	std::vector<CloudNode> cylinders;

	int nr_of_points=node.get_cloud_points();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud=node.get_cloud();

	CloudNodeViewer v;

	int segmented_cylinder_count=0;

	while(cloud->points.size()>0.2*nr_of_points)
	{
		std::stringstream ss;
		ss<<node.get_description()<<"_seg_cylinder_"<<segmented_cylinder_count;

		std::cout<<cloud->points.size()<<std::endl;
		normal_estimator.setSearchMethod(tree);
  		normal_estimator.setInputCloud(cloud);
  		normal_estimator.setKSearch(50);
  		normal_estimator.compute(*cylinder_normals);

  		segmentor.setOptimizeCoefficients (true);
  		segmentor.setModelType (pcl::SACMODEL_CYLINDER);
  		segmentor.setMethodType (pcl::SAC_RANSAC);
  		segmentor.setNormalDistanceWeight (0.01);
  		segmentor.setMaxIterations (10000);
  		segmentor.setDistanceThreshold(distance_thr);
  		segmentor.setRadiusLimits(min_r,max_r);
  		segmentor.setInputCloud (cloud);
    	segmentor.setInputNormals (cylinder_normals);
    	segmentor.segment(*cylinder_indices,*cylinder_coefficients);

    	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cylinder(new pcl::PointCloud<pcl::PointXYZ>);

    	point_extractor.setIndices(cylinder_indices);
		point_extractor.setNegative(false);
		point_extractor.setInputCloud(cloud);
		point_extractor.filter(*temp_cylinder);

		CloudNode temp_node(temp_cylinder);
		temp_node.set_axis_direction(cylinder_coefficients->values[3],cylinder_coefficients->values[4],cylinder_coefficients->values[5]);
		temp_node.set_point_axis(cylinder_coefficients->values[0],cylinder_coefficients->values[1],cylinder_coefficients->values[2]);
		temp_node.set_radius(cylinder_coefficients->values[6]);
		temp_node.set_description(ss.str());
		temp_node.set_length(this->compute_length(temp_node));


		segmented_cylinder_count++;
		
		//v.view(CloudNode(cloud),temp_node,temp_node.get_cloud_center());

		cylinders.push_back(temp_node);

		pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>);
		point_extractor.setNegative(true);
		point_extractor.filter(*input_cloud_outliers);
		cloud.swap(input_cloud_outliers);


	}
	return cylinders;
};


CloudNode Segmentor::get_main_plane()
{
	return CloudNode(this->main_plane);
}

CloudNode Segmentor::get_filtered_cloud()
{
	return CloudNode(this->filtered_cloud);
}


CloudNode Segmentor::get_sphere_cloud()
{
	return CloudNode(this->sphere_cloud);
}

CloudNode Segmentor::get_current_outliers()
{
	return CloudNode(this->current_outliers);
}

CloudNode Segmentor::get_cylinder_cloud()
{
	CloudNode node(cylinder_cloud);
	node.set_radius(cylinder_coefficients->values[6]);

	return CloudNode(this->cylinder_cloud);
}



