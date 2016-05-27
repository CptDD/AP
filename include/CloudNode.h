#ifndef H_CLOUD_NODE
#define H_CLOUD_NODE

#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>


class CloudNode
{
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points;
	Eigen::Vector4f center_point;
	std::vector<int> connected_components_indices;

	double radius;
	bool visited;
	void compute_cloud_center();

public:
	CloudNode(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud):
	radius(0.0),
	visited(false)
	{
		this->cloud_points=cloud;
		this->compute_cloud_center();
	}

	void set_radius(double radius){this->radius=radius;}
	void set_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){this->cloud_points=cloud;}
	void info_center_point();
	void show(){std::cout<<"We are here to make some noise!"<<std::endl;}
	void add_connected(int index){this->connected_components_indices.push_back(index);}
	void set_visited(){this->visited=true;}
	void set_unvisited(){this->visited=false;}
	

	Eigen::Vector4f get_cloud_center(){return this->center_point;}
	pcl::PointCloud<pcl::PointXYZ>::Ptr get_cloud(){return this->cloud_points;}
	std::vector<int> get_connected_indices(){return this->connected_components_indices;}

	int get_cloud_points(){return this->cloud_points->points.size();}
	double get_radius(){return radius;}
	double compute_distance(CloudNode node);
	bool is_index_present(int index);
	bool is_visited(){return visited;}
};

#endif