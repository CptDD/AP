#include <iostream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Geometry>
#include <pcl/registration/transformation_estimation_svd.h>

#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include "CloudNode.h"
#include "Viewer.h"
#include "Segmentor.h"
#include "Euclid.h"
#include "GraphUtil.h"
#include "GraphSearcher.h"
#include "Cloud2Serialize.h"
#include "Serializer.h"


using namespace std;

Eigen::Affine3f compute_transform_matrix(Eigen::Vector3f u,Eigen::Vector3f v);
Eigen::Affine3f compute_transform_matrix_v2(Eigen::Vector3f u,Eigen::Vector3f v);



int main(int argc,char**argv)
{

	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCDReader reader;
	CloudNodeViewer viewer;
	EuclideanExtractor extractor;
	GraphBuilder gb;

	CloudNode model_node(cloud_model);

	if(reader.read(argv[1],*cloud_model)==-1)
	{
		PCL_ERROR("Could not load the model cloud!\n");
		return -1;
	}
	cout<<"Model cloud loaded!"<<endl;

	if(reader.read(argv[2],*cloud_target)==-1)
	{
		PCL_ERROR("Could not load the target cloud!\n");
		return -1;
	}
	cout<<"Target cloud loaded!"<<endl;

	Segmentor s(cloud_model);
	Segmentor s2(cloud_target);

	vector<CloudNode> model_objects=extractor.extract_objects(s.get_current_outliers(),1);
	cout<<"Model objects!"<<endl;
	viewer.view(model_objects);

	vector<CloudNode> target_objects=extractor.extract_objects(s2.get_current_outliers(),1,"Target");
	cout<<"Target objects!"<<endl;
	viewer.view(target_objects);

	vector<vector<CloudNode> >model_graph;
	vector<vector<CloudNode> >target_graph;

	cout<<"Extracting the model cylinders!"<<endl;

	Serializer serializer("ModelGraph");

	for(int i=0;i<model_objects.size();i++)
	{
		vector<CloudNode> temp_cylinders=s.segment_cylinders(model_objects[i],0.05,0.15,0.04);
		gb.build_graph(model_graph,temp_cylinders);
	}

	serializer.save_graph(model_graph);
	gb.scene_graph_info();

	


	
	cout<<"Extracting the target cylinders!"<<endl;


	for(int i=0;i<target_objects.size();i++)
	{
		vector<CloudNode> temp_cylinders=s2.segment_cylinders(target_objects[i],0.05,0.15,0.04);
		gb.build_graph(target_graph,temp_cylinders);
	}
	gb.scene_graph_info();
		
	serializer.set_filename("TargetGraph");
	serializer.save_graph(target_graph);

	*/
	CloudNodeViewer viewer;
	vector<vector<CloudNode> >model_graph;
	vector<vector<CloudNode> >target_graph;

	Serializer serializer("ModelGraph");
	model_graph=serializer.load_graph();
	serializer.set_filename("TargetGraph");
	target_graph=serializer.load_graph();

	cout<<"Model!"<<endl;
	viewer.view_graph_test(model_graph[1]);
	
	cout<<"Targetter!"<<target_graph.size()<<endl;
	viewer.view_graph_test(target_graph[0]);

	GraphMatcher matcher(model_graph);
	matcher.search(target_graph[0]);

	/*Eigen::Affine3f transform_matrix=compute_transform_matrix(model_graph[4][0].get_axis_direction(),target_graph[4][0].get_axis_direction());
	Eigen::Affine3f transform_matrix_2=compute_transform_matrix_v2(target_graph[4][0].get_axis_direction(),model_graph[4][0].get_axis_direction());

	model_graph[4][0].transform_cloud(transform_matrix);

	viewer.view(model_graph[4][0],target_graph[4][0]);	

	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> svd;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 transformation;

	svd.estimateRigidTransformation(*model_graph[4][0].get_cloud(),*target_graph[4][0].get_cloud(),transformation);


	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			cout<<transformation(i,j)<<" ";
		}
		cout<<endl;
	}*/

	return 0;
}


Eigen::Affine3f compute_transform_matrix(Eigen::Vector3f u,Eigen::Vector3f v)
{
	double x1=u.x();
	double y1=u.y();
	double z1=u.z();

	double x2=v.x();
	double y2=v.y();
	double z2=v.z();

	double angle=acos((x1*x2+y1*y2+z1*z2)/(sqrt(x1*x1+y1*y1+z1*z1)*sqrt(x2*x2+y2*y2+z2*z2)));

	
	cout<<"Info angle :"<<angle<<endl;

	double xoff=x2-x1;
	double yoff=y2-y1;
	double zoff=z2-z1;

	Eigen::Affine3f transform_matrix=Eigen::Affine3f::Identity();

	transform_matrix(0,0)=cos(angle);
	transform_matrix(0,1)=sin(angle);
	transform_matrix(1,0)=-sin(angle);
	transform_matrix(1,1)=cos(angle);

	transform_matrix(0,3)=0;
	transform_matrix(1,3)=0;
	transform_matrix(2,3)=0;
	transform_matrix(3,3)=1;

	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			cout<<transform_matrix(i,j)<<" ";
		}
		cout<<endl;
	}

	return transform_matrix;
}

Eigen::Affine3f compute_transform_matrix_v2(Eigen::Vector3f u,Eigen::Vector3f v)
{
	double x1=u.x();
	double y1=u.y();
	double z1=u.z();

	double x2=v.x();
	double y2=v.y();
	double z2=v.z();

	double cos_a=(x1*x2+y1*y2+z1*z2)/sqrt((x1*x1+y1*y1+z1*z1)*(x2*x2+y2*y2+z2*z2));

	double half_cos=sqrt((1+cos_a)/2);
	double half_sin=sqrt((1-cos_a)/2);

	double sin_a=2*half_sin*half_cos;

	Eigen::Affine3f transform_matrix=Eigen::Affine3f::Identity();

	double x_t=x2-x1;
	double y_t=y2-y1;
	double z_t=z2-z1;

	transform_matrix(0,0)=cos_a;
	transform_matrix(0,1)=-sin_a;
	transform_matrix(1,0)=sin_a;
	transform_matrix(1,1)=cos_a;
	transform_matrix(0,3)=x_t;
	transform_matrix(1,3)=y_t;
	transform_matrix(2,3)=z_t;
	transform_matrix(3,3)=1;

	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			cout<<transform_matrix(i,j)<<" ";
		}
		cout<<endl;
	}

	return transform_matrix;

}
