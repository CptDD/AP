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
	friend class boost::serialization::access;
	template<class Archive>void serialize(Archive &ar,const unsigned int version)
	{
		ar&clouds_2_serialize;
	}
	std::vector<CloudNode2Serialize> clouds_2_serialize;
	std::string filename;
	void nodes_2_srl_nodes(std::vector<CloudNode> objects);
	std::vector<CloudNode> srl_nodes_2_nodes(); 
public:
	Serializer(){};
	Serializer(std::string filename):filename(filename){};
	void save_objects(std::vector<CloudNode>objects);
	void show_filename(){std::cout<<"The filename is :"<<filename<<std::endl;}
	void set_filename(std::string filename){this->filename=filename;clouds_2_serialize.clear();}

	std::vector<CloudNode> load_objects();
};

#endif