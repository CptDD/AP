#include "Cloud2Serialize.h"
#include "CloudNode.h"


void CloudNode2Serialize::show_info()
{
		std::cout<<"Radius :"<<radius<<std::endl;
		std::cout<<"===Visited==="<<visited<<std::endl;
		std::cout<<"Center point :"<<cp.x<<" "<<cp.y<<" "<<cp.z<<std::endl;
}

Eigen::Vector4f CloudNode2Serialize::get_center_point()
{
	Eigen::Vector4f center_point;

	center_point.x()=this->cp.x;
	center_point.y()=this->cp.y;
	center_point.z()=this->cp.z;

	return center_point;
}

Eigen::Vector3f CloudNode2Serialize::get_axis_direction()
{
	return Eigen::Vector3f(ad.x,ad.y,ad.z);
}