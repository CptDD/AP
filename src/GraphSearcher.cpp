#include "CloudNode.h"
#include "GraphSearcher.h"
#include "Viewer.h"
#include "ProbComp.h"

#define MAX_REPETITIONS 3


std::pair<int,double> GraphMatcher::get_most_similar_for_index(std::vector<CloudNode> target_graph,std::vector<int> target_index,
		CloudNode query_node)
{
	double radius_thr=0.03;
	double length_thr=0.08;

	//double radius_thr=0.01;
	//double radius_thr=0.06;

	std::pair<int,double> index(-1,100);

	for(int i=0;i<target_index.size();i++)
	{
		double radius_diff=std::abs(query_node.get_radius()-target_graph[target_index[i]].get_radius());
		double length_diff=std::abs(query_node.get_length()-target_graph[target_index[i]].get_length());

		std::cout<<"Radius diff :"<<radius_diff<<" Length diff :"<<length_diff<<std::endl;
		if(radius_diff<=radius_thr && length_diff<=length_thr)
		{
			std::cout<<"The start node is similar to node :"<<i<<" from the target graph!"<<std::endl;

			if(radius_diff<index.second)
			{
				index.first=i;
				index.second=radius_diff;
			}			
		}
		std::cout<<"Most similar node is :"<<index.first<<" with radius :"<<index.second<<std::endl;
	}
	return index;
}



bool GraphMatcher::is_present(std::vector<int> indices,int index)
{
	for(int i=0;i<indices.size();i++)
	{
		if(index==indices[i])
		{
			return true;
		}
	}
	return false;
}


double GraphMatcher::distance_for_index(std::vector<CloudNode> c_node,int start,int end)
{
	double distance=c_node[start].compute_distance(c_node[end]);
	return distance;
}


double **GraphMatcher::build_adjacency_matrix(std::vector<CloudNode>graph)
{
	double **adj_mat=new double*[graph.size()];

	for(int i=0;i<graph.size();i++)
	{
		adj_mat[i]=new double[graph.size()];

		for(int j=0;j<graph.size();j++)
		{
			adj_mat[i][j]=-1.0;	
		}
	}

	for(int i=0;i<graph.size();i++)
	{
		std::vector<int> indices=graph[i].get_connected_indices();

		for(int j=0;j<indices.size();j++)
		{	
			adj_mat[i][indices[j]]=distance_for_index(graph,i,indices[j]);
		}
	}

	/*
	for(int i=0;i<graph.size();i++)
	{
		for(int j=0;j<graph.size();j++)
		{
			cout<<adj_mat[i][j]<<" ";
		}
		cout<<endl;
	}*/

	return adj_mat;
}


std::pair<int,double> GraphMatcher::get_most_similar(std::vector<CloudNode> target_graph,CloudNode query_node)
{
	double radius_thr=0.03;
	double length_thr=0.08;

	//double radius_thr=0.08;
	//double radius_thr=0.02;

	std::pair<int,double> min_index;
	min_index.first=-1;
	min_index.second=100;



	for(int i=0;i<target_graph.size();i++)
	{
		double radius_diff=std::abs(query_node.get_radius()-target_graph[i].get_radius());
		double length_diff=std::abs(query_node.get_length()-target_graph[i].get_length());

		std::cout<<"Radius diff :"<<radius_diff<<std::endl;
		std::cout<<"Length diff :"<<length_diff<<std::endl;

		if(radius_diff<=radius_thr && length_diff<=length_thr)
		{
			//cout<<"The start node is similar to node :"<<i<<" from the target graph!"<<endl;

			if(radius_diff<min_index.second)
			{
				min_index.first=i;
				min_index.second=radius_diff;
			}			
		}
		std::cout<<"Most similar node is :"<<min_index.first<<" with radius :"<<min_index.second<<std::endl;
	}

	return min_index;
}


void GraphMatcher::getter(std::vector<CloudNode> target_graph,std::vector<CloudNode> query_graph,int target_ind,int query_ind,
	std::vector<int> &where_query,std::vector<int> &where_target,std::vector<int>visited_query,std::vector<int>visited_target)
{
	//double distance_thr=0.04;
	double distance_thr=0.03;

	//double distance_thr=0.01;
	
	double **adj_target=build_adjacency_matrix(target_graph);
	double **adj_query=build_adjacency_matrix(query_graph);


	for(int i=0;i<query_graph.size();i++)
	{
		double query_dist=adj_query[query_ind][i];

		for(int j=0;j<target_graph.size();j++)
		{
			double target_dist=adj_target[target_ind][j];

			if(target_dist>-1 && query_dist>-1)
			{
				double distance_diff=std::abs(target_dist-query_dist);

				std::cout<<"Distance Q :"<<i<<" "<<j<<" "<<distance_diff<<std::endl;

				if(distance_diff<=distance_thr)
				{
					//cout<<"Minium distance found for :"<<i<<" "<<j<<" "<<distance_diff<<endl;

					if(!is_present(visited_query,i))
					{
						where_query.push_back(i);
					}

					if(!is_present(visited_target,j))
					{
						where_target.push_back(j);
					}
				}
			}
		}
	}


	cout<<"We go forward for the query !"<<endl;
	for(int i=0;i<where_query.size();i++)
	{
		cout<<where_query[i]<<" ";
	}
	cout<<endl<<"For the target !"<<endl;
	for(int i=0;i<where_target.size();i++)
	{
		cout<<where_target[i]<<" ";
	}
	cout<<endl;

	/*
	 * Delete the adjacency matrices after usage
	 */
	for(int i=0;i<query_graph.size();i++)
	{
		delete [] adj_query[i];
	}
	delete [] adj_query;

	for(int i=0;i<target_graph.size();i++)
	{
		delete [] adj_target[i];
	}
	delete [] adj_target;
}

std::vector<std::pair<int,int> > GraphMatcher::search(std::vector<CloudNode> query_graph,bool screenshot)
{
	std::cout<<"Searching for the subraph!"<<std::endl;
		
	    target_graph=scene_graph[1];

		CloudNodeViewer viewer;

		int start_index;

		std::vector<CloudNode> similar_nodes;
		std::pair<int,double> min_index;

		std::vector<int> visited_query;
		std::vector<int> visited_target;

		std::vector<int> where_query;
		std::vector<int> where_target;

		std::vector<std::pair<int,int> >similars;

		/*
		 * Pick a random node and find the most similar node to it!
		 */

		do{
		 	start_index=rand()%query_graph.size();
			std::cout<<"Random node index is :"<<start_index<<std::endl;

			min_index=get_most_similar(target_graph,query_graph[start_index]);

			std::cout<<"Query node :"<<start_index<<" most similar to "<<min_index.first<<std::endl;
			
		}while(min_index.first==-1);



		visited_query.push_back(start_index);
		visited_target.push_back(min_index.first);
		query_graph[start_index].set_visited();
		target_graph[min_index.first].set_visited();

		similars.push_back(std::pair<int,int>(min_index.first,start_index));
	
		similar_nodes.push_back(target_graph[min_index.first]);

		viewer.view(query_graph[start_index]);
		viewer.view(target_graph[min_index.first]);

		/*
		 * Where to go next from current nodes, provided that distance threshold is respected
		 */
		getter(target_graph,query_graph,min_index.first,start_index,where_query,where_target,visited_query,visited_target);

		while(!where_query.empty())
		{

			int start_index=where_query.front();
			where_query.erase(where_query.begin());

			visited_query.push_back(start_index);
			query_graph[start_index].set_visited();

			min_index=get_most_similar_for_index(target_graph,where_target,query_graph[start_index]);

			if(min_index.first!=-1)
			{

				int target_index=where_target[min_index.first];
				where_target.erase(where_target.begin()+min_index.first);

				std::cout<<"Query node :"<<start_index<<" most similar to "<<target_index<<std::endl;

				target_graph[target_index].set_visited();
				visited_target.push_back(target_index);
				similar_nodes.push_back(target_graph[target_index]);

				similars.push_back(std::pair<int,int>(target_index,start_index));
			

				viewer.view(query_graph[start_index]);
				viewer.view(target_graph[target_index]);

				/*
				 * Update where to go next!
				 */
				 getter(target_graph,query_graph,target_index,start_index,where_target,where_query,visited_query,visited_target);

			}


		}
		
		std::cout<<"Similar size :"<<similar_nodes.size()<<std::endl;

		std::cout<<"The similar pairs are :"<<std::endl;

		for(int i=0;i<similars.size();i++)
		{
			std::cout<<"Target :"<<similars[i].first<<" with Query :"<<similars[i].second<<std::endl;
		}
		
		viewer.view(target_graph,query_graph,similars,screenshot);

		return similars;

}



void GraphMatcher::search(std::vector<CloudNode> query_graph,int g)
{
	std::cout<<"We are here to query some node!"<<std::endl;
	std::cout<<"The query graph has :"<<query_graph.size()<<" components!"<<std::endl;	
	std::cout<<"Scene--model graph has :"<<this->scene_graph.size()<<" components!"<<std::endl;

	CloudNodeViewer viewer;
	ProbComp prob_comp;

	std::vector<double> probabilities;

	for(int i=0;i<1;i++)
	{
		i=0;
		target_graph.clear();
		target_graph=scene_graph[i];

		std::cout<<"The "<<std::endl;
		viewer.view(target_graph);

		int start_index;
		std::vector<CloudNode> similar_nodes;
		std::pair<int,double> min_index;

		std::vector<int> visited_query;
		std::vector<int> visited_target;

		std::vector<int> where_query;
		std::vector<int> where_target;
		std::vector<std::pair<int,int> >similars;

		int dead_end=0;
		bool go=true;

		do
		{	
			start_index=rand()%query_graph.size();
			std::cout<<"Random node index is :"<<start_index<<std::endl;

			min_index=get_most_similar(target_graph,query_graph[start_index]);
			std::cout<<"Query node :"<<start_index<<" most similar to "<<min_index.first<<std::endl;

			if(min_index.first==-1)
			{
				dead_end++;
			}else
			{
				go=false;
			}

		}while(dead_end<MAX_REPETITIONS && go);

		if(min_index.first!=-1)
		{
			visited_query.push_back(start_index);
			visited_target.push_back(min_index.first);
			query_graph[start_index].set_visited();
			target_graph[min_index.first].set_visited();

			similars.push_back(std::pair<int,int>(min_index.first,start_index));
	
			similar_nodes.push_back(target_graph[min_index.first]);

			viewer.view(query_graph[start_index]);
			viewer.view(target_graph[min_index.first]);

			/*
			 * Where to go next from current nodes, provided that distance threshold is respected
		 	*/
			getter(target_graph,query_graph,min_index.first,start_index,where_query,where_target,visited_query,visited_target);

			while(!where_query.empty())
			{

				int start_index=where_query.front();
				where_query.erase(where_query.begin());

				visited_query.push_back(start_index);
				query_graph[start_index].set_visited();

				min_index=get_most_similar_for_index(target_graph,where_target,query_graph[start_index]);

				if(min_index.first!=-1)
				{

					int target_index=where_target[min_index.first];
					where_target.erase(where_target.begin()+min_index.first);

					std::cout<<"Query node :"<<start_index<<" most similar to "<<target_index<<std::endl;

					target_graph[target_index].set_visited();
					visited_target.push_back(target_index);
					similar_nodes.push_back(target_graph[target_index]);

					similars.push_back(std::pair<int,int>(target_index,start_index));
			

					viewer.view(query_graph[start_index]);
					viewer.view(target_graph[target_index]);

					/*
				 	* Update where to go next!
				 	*/
				 	getter(target_graph,query_graph,target_index,start_index,where_target,where_query,visited_query,visited_target);

				}


			}
		
			std::cout<<"Similar size :"<<similar_nodes.size()<<std::endl;
			std::cout<<"The similar pairs are :"<<std::endl;

			for(int i=0;i<similars.size();i++)
			{
				std::cout<<"Target :"<<similars[i].first<<" with Query :"<<similars[i].second<<std::endl;
			}
		
			viewer.view(target_graph,query_graph,similars);
		}

		prob_comp.set_model_nodes(target_graph);
		prob_comp.compute_level_probability(similars,target_graph.size());

		probabilities.push_back(prob_comp.compute_probability(similars,target_graph.size()));
	}
	std::cout<<"Probabilities are :"<<std::endl;
	for(int i=0;i<probabilities.size();i++)
	{
		std::cout<<probabilities[i]<<" ";
	}
	std::cout<<std::endl;

}


void GraphMatcher::show_level(std::vector<CloudNode> nodes)
{
	for(int i=0;i<nodes.size();i++)
	{
		std::cout<<i<<"-->";
		std::vector<int> connected=nodes[i].get_connected_indices();

		for(int j=0;j<connected.size();j++)
		{
			std::cout<<connected[j]<<" ";
		}
		std::cout<<std::endl;
	}
}