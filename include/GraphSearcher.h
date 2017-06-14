#ifndef H_SEARCHER
#define H_SEARCHER

#include "CloudNode.h"


/**
 * Name: GraphMatcher
 * Info: Class that computes similarities between two graphs
 * Constructor: The model of the objects to be queried for in the scene in the form of a matrix 
 *              of CloudNode objects
 *
 */
class GraphMatcher
{
private:
	std::vector<std::vector<CloudNode> >scene_graph;
	std::vector<CloudNode> target_graph;// Temporary graph vector used for object query purposes
	std::pair<int,double> get_most_similar_for_index(std::vector<CloudNode> target_graph,std::vector<int> target_index,
		CloudNode query_node);
	std::pair<int,double> get_most_similar(std::vector<CloudNode> target_graph,CloudNode query_node);
	void getter(std::vector<CloudNode> target_graph,std::vector<CloudNode> query_graph,int target_ind,int query_ind,
	std::vector<int> &where_query,std::vector<int> &where_target,std::vector<int>visited_query,std::vector<int>visited_target);

	double **build_adjacency_matrix(std::vector<CloudNode>graph);
	bool is_present(std::vector<int> indices,int index);
	double distance_for_index(std::vector<CloudNode> c_node,int start,int end);


public:
	GraphMatcher(std::vector<std::vector<CloudNode> >scene_graph)
	{
		this->scene_graph=scene_graph;
	};

	std::vector<std::pair<int,int> > search(std::vector<CloudNode> query_graph,bool screenshot=false);
	void search(std::vector<CloudNode> query_graph,int g);

	void show_level(std::vector<CloudNode> nodes);
};
#endif