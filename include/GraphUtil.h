#ifndef H_GRAPH
#define H_GRAPH

#include "CloudNode.h"

/**
 * Name: GraphMatcher
 * Info: Class used to build a graph out of the segmented CloudNodes
 * 
 */

class GraphBuilder
{
private:
	std::vector<std::vector<CloudNode> >scene_graph;
	std::pair<double,int> get_min_pair_for_node(std::vector<CloudNode> nodes,CloudNode target_node);
	void sort_length(std::vector<CloudNode> &nodes);
	void sort_volume(std::vector<CloudNode> &nodes);

public:
	void build_graph(std::vector<CloudNode> cloud_nodes);
	void build_graph(std::vector<std::vector<CloudNode> >&scene_graph,std::vector<CloudNode> cloud_nodes);
	void build_graph_length(std::vector<std::vector<CloudNode> >&scene_graph,std::vector<CloudNode> cloud_nodes);
	void build_graph_volume(std::vector<std::vector<CloudNode> >&scene_graph,std::vector<CloudNode> cloud_nodes);
	std::vector<std::vector<CloudNode> >get_scene_graph(){return this->scene_graph;}
	int get_scene_graph_size(){return this->scene_graph.size();}
	void scene_graph_info();
};
#endif