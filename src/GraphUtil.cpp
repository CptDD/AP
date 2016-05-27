#include "CloudNode.h"
#include "GraphUtil.h"

std::pair<double,int> GraphBuilder::get_min_pair_for_node(std::vector<CloudNode> nodes,CloudNode target_node)
{
	std::pair<double,int> min_index;
	double min_distance=target_node.compute_distance(nodes[0]);

	min_index.first=min_distance;
	min_index.second=0;

	double temp_distance;

	for(int i=1;i<nodes.size();i++)
	{
		temp_distance=target_node.compute_distance(nodes[i]);

		if(temp_distance<min_distance)
		{
			min_distance=temp_distance;
			min_index.first=min_distance;
			min_index.second=i;
		}
	}

	return min_index;
}


void GraphBuilder::build_graph(std::vector<CloudNode> cloud_nodes)
{
	std::vector<CloudNode> processed_nodes;

	for(int i=0;i<cloud_nodes.size();i++)
	{
		if(processed_nodes.size()>0)
		{
			std::pair<double,int> pair=get_min_pair_for_node(processed_nodes,cloud_nodes[i]);
         	std::cout<<"Minimum distance for :"<<i+1<<" is :"<<pair.first<<" and is between :"
           	<<i+1<<" and :"<<pair.second+1<<std::endl;

           	cloud_nodes[pair.second].add_connected(i);

           	if(!cloud_nodes[i].is_index_present(pair.second))
           	{
           		
           		cloud_nodes[i].add_connected(pair.second);
           	}
		}
		processed_nodes.push_back(cloud_nodes[i]);
	}
	  scene_graph.push_back(cloud_nodes);
}

void GraphBuilder::build_graph(std::vector<std::vector<CloudNode> >&scene_graph,std::vector<CloudNode> cloud_nodes)
{
	std::vector<CloudNode> processed_nodes;

	for(int i=0;i<cloud_nodes.size();i++)
	{
		if(processed_nodes.size()>0)
		{
			std::pair<double,int> pair=get_min_pair_for_node(processed_nodes,cloud_nodes[i]);
         	std::cout<<"Minimum distance for :"<<i+1<<" is :"<<pair.first<<" and is between :"
           	<<i+1<<" and :"<<pair.second+1<<std::endl;

           	cloud_nodes[pair.second].add_connected(i);

           	if(!cloud_nodes[i].is_index_present(pair.second))
           	{
           		
           		cloud_nodes[i].add_connected(pair.second);
           	}
		}
		processed_nodes.push_back(cloud_nodes[i]);
	}
	  scene_graph.push_back(cloud_nodes);
}



void GraphBuilder::scene_graph_info()
{
	for(int i=0;i<scene_graph.size();i++)
            {
            	std::vector<CloudNode> c_nodes=scene_graph[i];
            	std::cout<<"===Scene graph element :"<<i<<"==="<<std::endl;
            	for(int j=0;j<c_nodes.size();j++)
           		{
           			std::vector<int> con_comp=c_nodes[j].get_connected_indices();
           			std::cout<<"Connected to :"<<j<<std::endl;
           			for(int k=0;k<con_comp.size();k++)
           			{
           				std::cout<<con_comp[k]<<" ";
           			}
           			std::cout<<"======================"<<std::endl;
           		}
           	}
}