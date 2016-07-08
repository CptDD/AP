#include "CloudNode.h"
#include "Cloud2Serialize.h"
#include "Serializer.h"

#include <sstream>
#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

#include <pcl/io/pcd_io.h>


void Serializer::nodes_2_srl_nodes(std::vector<CloudNode> objects)
{
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
}

std::vector<CloudNode> Serializer::srl_nodes_2_nodes()
{
	std::vector<CloudNode> nodes;
	pcl::PCDReader reader;

	for(int i=0;i<clouds_2_serialize.size();i++)
	{
		CloudNode temp_node;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		if(reader.read(clouds_2_serialize[i].get_filename(),*cloud)==-1)
		{
			PCL_ERROR("Could not read the point cloud!\n");
		}else
		{
			temp_node.set_cloud(cloud);
		}


		temp_node.set_radius(clouds_2_serialize[i].get_radius());
		temp_node.set_axis_direction(clouds_2_serialize[i].get_axis_direction());
		temp_node.set_center_point(clouds_2_serialize[i].get_center_point());
		clouds_2_serialize[i].is_visited()?temp_node.set_visited():temp_node.set_unvisited();
		temp_node.set_connected_indices(clouds_2_serialize[i].get_connected_indices());

		nodes.push_back(temp_node);
	}

	return nodes;
}

void Serializer::save_objects(std::vector<CloudNode> objects)
{
	std::cout<<"Saving the objects . . ."<<std::endl;
	this->nodes_2_srl_nodes(objects);

	std::ofstream output_stream(this->filename.c_str());
	boost::archive::text_oarchive output_archive(output_stream);
	output_archive<<clouds_2_serialize;

	output_stream.close();

}

std::vector<CloudNode> Serializer::load_objects()
{
	std::cout<<"Loading the objects . . . "<<std::endl;

	std::ifstream input_stream(this->filename.c_str());
	boost::archive::text_iarchive input_archive(input_stream);

	input_archive>>this->clouds_2_serialize;
	input_stream.close();

	return this->srl_nodes_2_nodes();
}