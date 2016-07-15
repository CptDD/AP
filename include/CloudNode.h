#ifndef H_CLOUD_NODE
#define H_CLOUD_NODE

#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>


class CloudNode
{
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points;
	Eigen::Vector4f center_point;
	Eigen::Vector3f axis_direction;
	Eigen::Vector3f point_axis;
	std::vector<int> connected_components_indices;
	std::string node_description;

	double radius;
	double length;
	bool visited;

	void compute_cloud_center();


public:
	CloudNode():
	radius(0.0),
	length(0.0),
	visited(false){}

	CloudNode(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud):
	radius(0.0),
	length(0.0),
	visited(false)
	{
		this->cloud_points=cloud;
		this->compute_cloud_center();
	}

	void set_radius(double radius){this->radius=radius;}
	void set_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){this->cloud_points=cloud;}
	void set_center_point(Eigen::Vector4f center_point){this->center_point=center_point;}
	void set_connected_indices(std::vector<int>connected_indices){this->connected_components_indices=connected_indices;}
	void info_center_point();
	void show(){std::cout<<"We are here to make some noise!"<<std::endl;}
	void add_connected(int index){this->connected_components_indices.push_back(index);}
	void set_visited(){this->visited=true;}
	void set_unvisited(){this->visited=false;}
	void set_visited(bool visit_state){this->visited=visit_state;}
	void set_axis_direction(double x,double y,double z);
	void set_axis_direction(Eigen::Vector3f axis_direction){this->axis_direction=axis_direction;}
	void show_axis_direction(){std::cout<<"X :"<<axis_direction.x()<<" Y :"<<axis_direction.y()<<" Z :"<<axis_direction.z()<<std::endl;}
	void transform_cloud(Eigen::Affine3f transform_matrix);
	void set_description(std::string node_description){this->node_description=node_description;}
	void set_point_axis(double x,double y,double z);
	void set_length(double length){this->length=length;}
	

	Eigen::Vector4f get_cloud_center(){return this->center_point;}
	pcl::PointCloud<pcl::PointXYZ>::Ptr get_cloud(){return this->cloud_points;}
	std::vector<int> get_connected_indices(){return this->connected_components_indices;}
	Eigen::Vector3f get_axis_direction(){return this->axis_direction;}
	Eigen::Vector3f get_point_axis(){return this->point_axis;}
	std::string get_description(){return this->node_description;}

	int get_cloud_points(){return this->cloud_points->points.size();}
	double get_radius(){return radius;}
	double compute_distance(CloudNode node);
	double compute_distance_eff(CloudNode node);
	bool is_index_present(int index);
	bool is_visited(){return visited;}
	double get_length(){return length;}
};

#endif