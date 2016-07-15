#ifndef H_CLOUD2SERIALIZE
#define H_CLOUD2SERIALIZE

#include <vector>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>

#include "CloudNode.h"

class CloudNode2Serialize
{
private:
	friend class boost::serialization::access;


	template<class Archive>void serialize(Archive &ar,const unsigned int version)
	{
		ar&radius;
		ar&visited;
		ar&length;
		ar&connected_component_indices;
		ar&cp.x&cp.y&cp.z;
		ar&ad.x&ad.y&ad.z;
		ar&pcd_filename;
	};

	struct center_point
	{
		double x,y,z;
	};

	struct axis_direction
	{
		double x,y,z;
	};

	double radius;
	double length;
	bool visited;
	std::vector<int> connected_component_indices;
	std::string pcd_filename;

	center_point cp;
	axis_direction ad;

public:
	CloudNode2Serialize(){};
	CloudNode2Serialize(CloudNode cloud_node)
	{
		connected_component_indices=cloud_node.get_connected_indices();
		visited=cloud_node.is_visited();		
		radius=cloud_node.get_radius();
		length=cloud_node.get_length();

		Eigen::Vector3f axis_direction=cloud_node.get_axis_direction();
		ad.x=axis_direction.x();
		ad.y=axis_direction.y();
		ad.z=axis_direction.z();

		Eigen::Vector4f center_point=cloud_node.get_cloud_center();
		cp.x=center_point.x();
		cp.y=center_point.y();
		cp.z=center_point.z();
	}

	void show_info();
	void set_filename(std::string pcd_filename){this->pcd_filename=pcd_filename;}

	double get_radius(){return this->radius;}
	double get_length(){return this->length;}
	bool is_visited(){return this->visited;}

	std::string get_filename(){return this->pcd_filename;}
	std::vector<int> get_connected_indices(){return this->connected_component_indices;}
	Eigen::Vector4f get_center_point();
	Eigen::Vector3f get_axis_direction();

};


#endif