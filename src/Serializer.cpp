#include "CloudNode.h"
#include "Cloud2Serialize.h"
#include "Serializer.h"

#include <sstream>
#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

#include <pcl/io/pcd_io.h>


std::vector<CloudNode2Serialize> Serializer::nodes_2_srl_nodes(std::vector<CloudNode> objects)
{
	std::vector<CloudNode2Serialize> clouds_2_serialize;

	pcl::PCDWriter writer;
	for(int i=0;i<objects.size();i++)
	{
		std::stringstream ss;
		ss<<"Clouds/"<<objects[i].get_description()<<".pcd";

		CloudNode2Serialize temp_cloud(objects[i]);
		temp_cloud.set_filename(ss.str());

		clouds_2_serialize.push_back(temp_cloud);
		writer.write(ss.str(),*objects[i].get_cloud());
	}

	return clouds_2_serialize;
}


std::vector<CloudNode> Serializer::srl_nodes_2_nodes(std::vector<CloudNode2Serialize> objects)
{
	std::vector<CloudNode> nodes;
	pcl::PCDReader reader;

	for(int i=0;i<objects.size();i++)
	{
		CloudNode temp_node;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		if(reader.read(objects[i].get_filename(),*cloud)==-1)
		{
			PCL_ERROR("Could not read the point cloud!\n");
		}else
		{
			temp_node.set_cloud(cloud);
		}

		temp_node.set_radius(objects[i].get_radius());
		temp_node.set_axis_direction(objects[i].get_axis_direction());
		temp_node.set_center_point(objects[i].get_center_point());
		objects[i].is_visited()?temp_node.set_visited():temp_node.set_unvisited();
		temp_node.set_connected_indices(objects[i].get_connected_indices());

		nodes.push_back(temp_node);
	}

	return nodes;
}

std::vector<CloudNode2Serialize> Serializer::convert_2_serializable(std::vector<CloudNode> nodes)
{
	std::vector<CloudNode2Serialize> srl_nodes;

	pcl::PCDWriter writer;
	for(int i=0;i<nodes.size();i++)
	{
		std::stringstream ss;
		ss<<"Clouds/"<< nodes[i].get_description()<<".pcd";

		CloudNode2Serialize temp_cloud(nodes[i]);
		temp_cloud.set_filename(ss.str());

		srl_nodes.push_back(temp_cloud);
		writer.write(ss.str(),*nodes[i].get_cloud());
	}

	return srl_nodes;
}

std::vector<std::vector<CloudNode2Serialize> >Serializer::graph_2_serialize(std::vector<std::vector<CloudNode> >graph)
{
	std::vector<std::vector<CloudNode2Serialize> >graph_2_serialize;

	for(int i=0;i<graph.size();i++)
	{
		std::vector<CloudNode2Serialize> temp_srl_vector=this->convert_2_serializable(graph[i]);

		graph_2_serialize.push_back(temp_srl_vector);
	}

	return graph_2_serialize;
}

std::vector<std::vector<CloudNode> >Serializer::srl_graph_2_graph(std::vector<std::vector<CloudNode2Serialize> >srl_graph)
{
	std::vector<std::vector<CloudNode> >graph;

	for(int i=0;i<srl_graph.size();i++)
	{
		std::vector<CloudNode> temp_nodes=this->srl_nodes_2_nodes(srl_graph[i]);
		graph.push_back(temp_nodes);
	}

	return graph;
}

void Serializer::save_objects(std::vector<CloudNode> objects)
{
	std::cout<<"Saving the objects . . ."<<std::endl;
	std::vector<CloudNode2Serialize> srl_objects;
	srl_objects=this->nodes_2_srl_nodes(objects);

	std::ofstream output_stream(this->filename.c_str());
	boost::archive::text_oarchive output_archive(output_stream);
	output_archive<<srl_objects;

	output_stream.close();

}

std::vector<CloudNode> Serializer::load_objects()
{
	std::cout<<"Loading the objects . . . "<<std::endl;
	std::vector<CloudNode2Serialize> temp_srl_nodes;

	std::ifstream input_stream(this->filename.c_str());
	boost::archive::text_iarchive input_archive(input_stream);

	input_archive>>temp_srl_nodes;
	input_stream.close();

	return this->srl_nodes_2_nodes(temp_srl_nodes);
}

void Serializer::save_graph(std::vector<std::vector<CloudNode> >graph)
{
	std::cout<<"Saving the graph . . . "<<std::endl;

	std::vector<std::vector<CloudNode2Serialize> >srl_graph;

	srl_graph=this->graph_2_serialize(graph);

	std::ofstream output_stream(this->filename.c_str());
	boost::archive::text_oarchive output_archive(output_stream);
	output_archive<<srl_graph;

	output_stream.close();
}

std::vector<std::vector<CloudNode> >Serializer::load_graph()
{
	std::cout<<"Loading the graph . . . "<<std::endl;
	std::vector<std::vector<CloudNode2Serialize> >srl_graph;

	std::ifstream input_stream(this->filename.c_str());
	boost::archive::text_iarchive input_archive(input_stream);

	input_archive>>srl_graph;

	input_stream.close();

	return this->srl_graph_2_graph(srl_graph);
}