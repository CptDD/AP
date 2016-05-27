#include "CloudNode.h"
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <cmath>

void CloudNode::compute_cloud_center()
{
	pcl::compute3DCentroid(*this->cloud_points,center_point);
}



void CloudNode::info_center_point()
{
	std::cout<<"===Center Point==="<<std::endl;
	std::cout<<"x :"<<center_point.x()<<" y :"<<center_point.y()<<" z :"<<center_point.z()<<std::endl;
    std::cout<<"=================="<<std::endl;
}


double CloudNode::compute_distance(CloudNode node)
{
	Eigen::Vector4f n_c=node.get_cloud_center();

	double xdiff=n_c.x()-center_point.x();
	double ydiff=n_c.y()-center_point.y();
	double zdiff=n_c.z()-center_point.z();

    return sqrt(xdiff*xdiff+ydiff*ydiff+zdiff*zdiff);
}

bool CloudNode::is_index_present(int index)
{
	for(int i=0;i<connected_components_indices.size();i++)
	{
		if(index==connected_components_indices[i])
		{
			return true;
		}
	}
	return false;
}
