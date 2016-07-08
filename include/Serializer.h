#ifndef H_SERIALIZER
#define H_SERIALIZER

#include <cstring>
#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

#include "CloudNode.h"
#include "Cloud2Serialize.h"

class Serializer
{

private:
	std::string filename;

	std::vector<CloudNode2Serialize> nodes_2_srl_nodes(std::vector<CloudNode> objects);
	std::vector<CloudNode> srl_nodes_2_nodes(std::vector<CloudNode2Serialize> objects); 
	std::vector<CloudNode2Serialize> convert_2_serializable(std::vector<CloudNode> nodes);
	std::vector<std::vector<CloudNode2Serialize> >graph_2_serialize(std::vector<std::vector<CloudNode> >graph);
	std::vector<std::vector<CloudNode> >srl_graph_2_graph(std::vector<std::vector<CloudNode2Serialize> >srl_graph);
public:
	Serializer(){};
	Serializer(std::string filename):filename(filename){};
	void save_objects(std::vector<CloudNode>objects);
	void show_filename(){std::cout<<"The filename is :"<<filename<<std::endl;}
	void set_filename(std::string filename){this->filename=filename;}
	void save_graph(std::vector<std::vector<CloudNode> >graph);

	std::vector<CloudNode> load_objects();	
	std::vector<std::vector<CloudNode> >load_graph();
};

#endif