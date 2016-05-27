#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include "CloudNode.h"
#include "Viewer.h"
#include "Segmentor.h"
#include "Euclid.h"
#include "GraphUtil.h"
#include "GraphSearcher.h"


using namespace std;

int main(int argc,char**argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model(new pcl::PointCloud<pcl::PointXYZ>);
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


	Segmentor s(cloud_target);


	vector<CloudNode> model_objects=extractor.extract_objects(model_node,1);
	cout<<"Model objects!"<<endl;
	viewer.view(model_objects);

	vector<CloudNode> target_objects=extractor.extract_objects(s.get_current_outliers(),1);
	cout<<"Target objects!"<<endl;
	viewer.view(target_objects);

	vector<vector<CloudNode> >model_graph;
	vector<vector<CloudNode> >target_graph;

	for(int i=0;i<model_objects.size();i++)
	{
		vector<CloudNode> temp_cylinders=s.segment_cylinders(model_objects[i],0.04);
		gb.build_graph(model_graph,temp_cylinders);
	}
	gb.scene_graph_info();



	for(int i=0;i<target_objects.size();i++)
	{
		vector<CloudNode> temp_cylinders=s.segment_cylinders(target_objects[i],0.04);
		gb.build_graph(target_graph,temp_cylinders);
	}
	gb.scene_graph_info();

	cout<<"Targetter!"<<target_graph.size()<<endl;
	viewer.view(target_graph[4],true);


	GraphMatcher matcher(model_graph);
	matcher.search(target_graph[4],true);

	viewer.view_graph(target_graph[4]);

	return 0;
}	