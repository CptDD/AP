#ifndef SEGMENTOR
#define SEGMENTOR

#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Segmentor
{
public:

	static void pass_filter_elongated_model(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{

		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("y");
		pass_filter.setFilterLimits(0.3,1);
		pass_filter.filter(*filtered_cloud);
	}


	static void pass_filter_mushroom_model(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{

		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("y");
		pass_filter.setFilterLimits(0.2,1);
		pass_filter.filter(*filtered_cloud);
	}


	static void pass_filter_model(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("y");
		pass_filter.setFilterLimits(3.2,4);
		pass_filter.filter(*filtered_cloud);
	}

	static void pass_filter_scene(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("y");
		pass_filter.setFilterLimits(0.8,4);
		pass_filter.filter(*filtered_cloud);
	}

	static void pass_filter_cup(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("y");
		pass_filter.setFilterLimits(0.6,1);
		pass_filter.filter(*filtered_cloud);
	}


	static void pass_filter_up(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("z");
		pass_filter.setFilterLimits(-4,0);
		pass_filter.filter(*filtered_cloud);
	}
	

	static void pass_filter_side(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("y");
		pass_filter.setFilterLimits(-0.5,0.5);
		pass_filter.filter(*filtered_cloud);
	}


	static void pass_filter_scene_2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("y");
		pass_filter.setFilterLimits(-0.6,0);
		pass_filter.filter(*filtered_cloud);
	}

	static void pass_filter_scene_3(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("y");
		pass_filter.setFilterLimits(-0.1,1);
		pass_filter.filter(*filtered_cloud);
	}

	static void pass_filter_scene_4(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("y");
		pass_filter.setFilterLimits(0,1);
		pass_filter.filter(*filtered_cloud);
	}

	static void pass_filter_single(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("x");
		pass_filter.setFilterLimits(-1,0.5);
		pass_filter.filter(*filtered_cloud);
	}


	static void pass_filter_single_2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("x");
		pass_filter.setFilterLimits(-1,0.5);
		pass_filter.filter(*filtered_cloud);
	}


	static void pass_filter_single_3(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("x");
		pass_filter.setFilterLimits(-0.5,0.4);
		pass_filter.filter(*filtered_cloud);
	}


	static void pass_filter_single_4(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("x");
		pass_filter.setFilterLimits(-1,0.5);
		pass_filter.filter(*filtered_cloud);
	}

	static void pass_filter_conveyor_cup(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("x");
		pass_filter.setFilterLimits(-0.3,0.2);
		pass_filter.filter(*filtered_cloud);
	}



	static void pass_filter_scaled_first(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("y");
		//pass_filter.setFilterLimits(-0.06,1);
		pass_filter.setFilterLimits(-0.01,1);
		pass_filter.filter(*filtered_cloud);
	}

	static void pass_filter_scaled_first_side(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("x");
		//pass_filter.setFilterLimits(-0.4,-0.1);
		//pass_filter.setFilterLimits(-0.1,0); //small bulb
		//pass_filter.setFilterLimits(0,0.2); //mushroom_bulb
		//pass_filter.setFilterLimits(-0.4,-0.3); //new model bulb
		pass_filter.setFilterLimits(0.2,0.4); //elongated bulb
		pass_filter.filter(*filtered_cloud);
	}


	static void pass_filter_scaled_second(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("y");
		pass_filter.setFilterLimits(-0.06,1);
		pass_filter.filter(*filtered_cloud);
	}

	static void pass_filter_scaled_second_side(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("x");
		pass_filter.setFilterLimits(-0.7,-0.5);
		pass_filter.filter(*filtered_cloud);
	}

	static void pass_filter_scaled_third(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("y");
		pass_filter.setFilterLimits(-0.06,1);
		pass_filter.filter(*filtered_cloud);
	}

	static void pass_filter_scaled_third_side(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("x");
		pass_filter.setFilterLimits(0,0.2);
		pass_filter.filter(*filtered_cloud);
	}


	static void pass_filter_scaled_fourth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("z");
		//pass_filter.setFilterLimits(-1,1);
		pass_filter.setFilterLimits(-1.4,-1.3);
		pass_filter.filter(*filtered_cloud);
	}

	static void pass_filter_scaled_fourth_side(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("y");
		//pass_filter.setFilterLimits(-0.05,0.1); //mushroom bulb
		//pass_filter.setFilterLimits(0.4,0.5); //modern bulb
		//pass_filter.setFilterLimits(-0.3,-0.05); //elongated bulb
		pass_filter.setFilterLimits(0.1,0.2); //standard bulb
		pass_filter.filter(*filtered_cloud);
	}


	static void pass_filter_scaled_fifth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("y");
		pass_filter.setFilterLimits(-0.2,0);
		pass_filter.filter(*filtered_cloud);
	}

	static void pass_filter_scaled_fifth_side(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("z");
		pass_filter.setFilterLimits(-0.9,-0.8);
		pass_filter.filter(*filtered_cloud);
	}


	static void pass_filter_scaled_model(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("y");
		//pass_filter.setFilterLimits(0.65,1);
		pass_filter.setFilterLimits(0.39,1);
		pass_filter.filter(*filtered_cloud);
	}

	static void pass_filter_scaled_model_side(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("x");
		pass_filter.setFilterLimits(-1,1);
		pass_filter.filter(*filtered_cloud);
	}

	static void pass_filter_mush_2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("y");
		pass_filter.setFilterLimits(4.3,5);
		pass_filter.filter(*filtered_cloud);
	}

	static void downsample_model(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::VoxelGrid<pcl::PointXYZ> grid;
		grid.setInputCloud(cloud);
		grid.setLeafSize(0.001f,0.001f,0.001f);
		grid.filter(*filtered_cloud);
	}

	static void downsample_model_big(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::VoxelGrid<pcl::PointXYZ> grid;
		grid.setInputCloud(cloud);
		grid.setLeafSize(0.01f,0.01f,0.01f);
		grid.filter(*filtered_cloud);
	}


	static void pass_filter_first_big(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("y");
		pass_filter.setFilterLimits(-1.19,0);
		pass_filter.filter(*filtered_cloud);
	}


	static void pass_filter_first_big_side(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("x");
		//pass_filter.setFilterLimits(0,2); //elongated_bulb
		//pass_filter.setFilterLimits(-1,0); //mushroom_bulb
		pass_filter.setFilterLimits(0.5,3); //standard_bulb -2 -1
		pass_filter.filter(*filtered_cloud);
	}



	static void pass_filter_fourth_big(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("z");
		pass_filter.setFilterLimits(-4,5);
		pass_filter.filter(*filtered_cloud);
	}


	static void pass_filter_fourth_big_side(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("y");
		pass_filter.setFilterLimits(0,3);
		pass_filter.filter(*filtered_cloud);
	}


	static void pass_filter_model_big(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("y");
		pass_filter.setFilterLimits(3.2,5);
		pass_filter.filter(*filtered_cloud);
	}




};

#endif