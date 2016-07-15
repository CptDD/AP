#include "CloudNode.h"
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <cmath>

void CloudNode::compute_cloud_center()
{
	pcl::compute3DCentroid(*this->cloud_points,center_point);
}


void CloudNode::set_axis_direction(double x,double y,double z)
{
	axis_direction.x()=x;
	axis_direction.y()=y;
	axis_direction.z()=z;
}

void CloudNode::set_point_axis(double x,double y,double z)
{
	point_axis.x()=x;
	point_axis.y()=y;
	point_axis.z()=z;
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




double CloudNode::compute_distance_eff(CloudNode node)
{
	Eigen::Vector4f n_c=node.get_cloud_center();
	double xdiff=n_c.x()-center_point.x();
	double ydiff=n_c.y()-center_point.y();
	double zdiff=n_c.z()-center_point.z();

	return (xdiff*xdiff+ydiff*ydiff+zdiff*zdiff);

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


void CloudNode::transform_cloud(Eigen::Affine3f transform_matrix)
{
	std::cout<<"Transforming!"<<std::endl;
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			std::cout<<transform_matrix(i,j)<<" ";
		}
		std::cout<<std::endl;
	}

	pcl::transformPointCloud(*cloud_points,*cloud_points,transform_matrix);
}