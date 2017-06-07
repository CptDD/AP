#include <iostream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Geometry>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/common.h>

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
#include "ProbComp.h"


using namespace std;

Eigen::Affine3f compute_transform_matrix(Eigen::Vector3f u,Eigen::Vector3f v);
Eigen::Affine3f compute_transform_matrix_v2(Eigen::Vector3f u,Eigen::Vector3f v);
vector<pcl::PointXYZ> project_cloud(CloudNode node);


int main(int argc,char**argv)
{


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCDReader reader;
	CloudNodeViewer viewer;
	EuclideanExtractor extractor;
	GraphBuilder gb;

	if(reader.read(argv[1],*cloud_model)==-1)
	{
		PCL_ERROR("Could not load the model cloud!\n");
		return -1;
	}
	cout<<"Model cloud loaded!"<<endl;

	CloudNode mm(cloud_model);

	viewer.view(mm);

	if(reader.read(argv[2],*cloud_target)==-1)
	{
		PCL_ERROR("Could not load the target cloud!\n");
		return -1;
	}

	cout<<"Target cloud loaded!"<<endl;

	Segmentor s(cloud_model);
	

	vector<CloudNode> model_objects=extractor.extract_objects(s.get_current_outliers(),1);
	cout<<"Model objects!"<<endl;
	viewer.view(model_objects);


	Segmentor s2(cloud_target);

	vector<CloudNode> target_objects=extractor.extract_objects(s2.get_current_outliers(),1,"Target");
	cout<<"Target objects!"<<endl;
	viewer.view(target_objects);



	vector<vector<CloudNode> >model_graph;
	vector<vector<CloudNode> >target_graph;

	cout<<"Extracting the model cylinders!"<<endl;

	//Serializer serializer("ModelGraph");

	


	for(int i=0;i<model_objects.size();i++)
	{
		vector<CloudNode> temp_cylinders=s.segment_cylinders(model_objects[i],0.05,0.15,0.04);
		gb.build_graph(model_graph,temp_cylinders);
	}

	//serializer.save_graph(model_graph);
	gb.scene_graph_info();

	
	cout<<"Extracting the target cylinders!"<<endl;


	for(int i=0;i<target_objects.size();i++)
	{
		vector<CloudNode> temp_cylinders=s2.segment_cylinders(target_objects[i],0.05,0.15,0.04);
		gb.build_graph(target_graph,temp_cylinders);
	}
	gb.scene_graph_info();
		
	cout<<"Showing the objects to be searched!"<<endl;
	viewer.view(target_graph[2]);

	//serializer.set_filename("TargetGraph");
	//serializer.save_graph(target_graph);



	//cout<<"Model!"<<endl;
	//viewer.view_graph_test(model_graph[1]);
	
	//cout<<"Targetter!"<<target_graph.size()<<endl;
	//viewer.view_graph_test(target_graph[0]);

	GraphMatcher matcher(model_graph);
	matcher.search(target_graph[2],1);
	//vector<pair<int,int> >similars=matcher.search(target_graph[0]);

	//ProbComp prob_comp;
	//prob_comp.compute_probability(similars,target_graph[1].size());


	//Eigen::Affine3f transform_matrix=compute_transform_matrix(model_graph[1][0].get_axis_direction(),target_graph[0][0].get_axis_direction());
	
	/*Eigen::Affine3f transform_matrix_2=compute_transform_matrix_v2(target_graph[4][0].get_axis_direction(),model_graph[4][0].get_axis_direction());*/

	//CloudNode test_node=target_graph[0][0];
	//test_node.transform_cloud(transform_matrix.inverse());

	//model_graph[1][0].transform_cloud(transform_matrix);

	//viewer.view(test_node,model_graph[1][0]);	

	/*
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> svd;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 transformation;

	svd.estimateRigidTransformation(*model_graph[4][0].get_cloud(),*target_graph[4][0].get_cloud(),transformation);*/


	/*for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			cout<<transform_matrix(i,j)<<" ";
		}
		cout<<endl;
	}*/

	return 0;
}


vector<pcl::PointXYZ> project_cloud(CloudNode node)
{
	pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);

	coeff->values.push_back(node.get_point_axis().x());
	coeff->values.push_back(node.get_point_axis().y());
	coeff->values.push_back(node.get_point_axis().z());
	coeff->values.push_back(node.get_axis_direction().x());
	coeff->values.push_back(node.get_axis_direction().y());
	coeff->values.push_back(node.get_axis_direction().z());

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType (pcl::SACMODEL_LINE);
    proj.setInputCloud (node.get_cloud());
    proj.setModelCoefficients (coeff);
    proj.filter(*cloud); 

    CloudNodeViewer viewer;
    CloudNode test(cloud);
    viewer.view(node,test);


    pcl::PointXYZ min,max;

    pcl::getMinMax3D(*cloud,min,max);

    double xdiff=max.x-min.x;
    double ydiff=max.y-min.y;
    double zdiff=max.z-min.z;

    cout<<"Radius :"<<node.get_radius()<<endl;
    cout<<"Length :"<<sqrt(xdiff*xdiff+ydiff*ydiff+zdiff*zdiff)<<endl;


   cout<<"Points"<<endl;

   pcl::PointXYZ mn=cloud->points[0];
   pcl::PointXYZ mx=cloud->points[0];

   for(int i=1;i<cloud->points.size();i++)
   {
   		if(cloud->points[i].x<mn.x)
   		{
   			mn=cloud->points[i];
   		}

   		if(cloud->points[i].x>mx.x)
   		{
   			mx=cloud->points[i];
   		}
   }

  
   cout<<"===Values for test!==="<<endl;

   cout<<mn.x<<" "<<mn.y<<" "<<mn.z<<endl;
   cout<<mx.x<<" "<<mx.y<<" "<<mx.z<<endl;

    viewer.view_line_test(node,min,max);

    viewer.view_line_test(node,mn,mx);

    viewer.view_line_test(test,mn,mx);

    vector<pcl::PointXYZ> vec;
    vec.push_back(min);
    vec.push_back(max);

    return vec;

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
