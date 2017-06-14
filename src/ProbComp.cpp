#include "ProbComp.h"



void ProbComp::setup_probabilities(int model_length)
{

	std::cout<<"Setting up probabilities for model with size :"<<model_length<<std::endl;
	if(model_length==2)
	{
		this->level_probs[0]=0.6;
		this->level_probs[1]=0.4;
	}
}


double ProbComp::compute_probability(std::vector<std::pair<int,int> >similarities,int model_length)
{
	std::cout<<"Similarities size :"<<similarities.size()<<std::endl;
	std::cout<<"Model graph length :"<<model_length<<std::endl;


	double single_prob=1/(double)model_length;

	double prob=single_prob*similarities.size();

	std::cout<<"Single prob :"<<single_prob<<std::endl;

	for(int i=0;i<similarities.size();i++)
	{
		std::cout<<"Model :"<<similarities[i].first<<" Target :"<<similarities[i].second<<std::endl;
	}

	std::cout<<"The probabilities are :"<<prob<<std::endl;

	if(similarities.size()>0)
	{
		return prob;
	}else
	{
		return 0;
	}
}


int compute_level(std::vector<CloudNode> nodes,int similar)
{
	std::cout<<"Computing the level!"<<std::endl;

	std::vector<int> connected_indices=nodes[similar].get_connected_indices();

	int level=0;

	std::cout<<"The similar node is :"<<similar<<std::endl;

	for(int i=0;i<connected_indices.size();i++)
	{
		std::cout<<"Connected index :"<<connected_indices[i]<<" ";
		if(connected_indices[i]<similar)
		{
			level=i+1;
		}
	}
	std::cout<<std::endl<<"The level of :"<<similar<<" is :"<<level<<std::endl;

	return level;
}


bool is_present(std::vector<int> indices,int index)
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

int compute_levels(std::vector<CloudNode> nodes)
{
	std::vector<int>levels;
	for(int i=0;i<nodes.size();i++)
	{
		int level=compute_level(nodes,i);
		if(!is_present(levels,level))
		{
			levels.push_back(level);
		}

	}

	return levels.size();
}

double ProbComp::compute_level_probability(std::vector<std::pair<int,int> >similarities,int model_length)
{

	std::cout<<"Model has: "<<compute_levels(this->get_model_nodes())<<" levels!"<<std::endl;

	return 1;
	/*std::cout<<"==============================================================================="<<std::endl;
	this->setup_probabilities(model_length);
	std::cout<<"Computing level probabilities!"<<std::endl;
	std::cout<<"There are :"<<similarities.size()<<" similarities between the model and target!"<<std::endl;
	std::cout<<"Model graph length :"<<model_length<<std::endl;
	
	double prob=0;

	for(int i=0;i<similarities.size();i++)
	{
		int level=compute_level(this->get_model_nodes(),similarities[i].first);

		std::cout<<"The level is :"<<level<<std::endl;

		prob+=this->level_probs[level];
	}


	std::cout<<"The probabilities are :"<<prob<<std::endl;
	std::cout<<"==============================================================================="<<std::endl;

	/*double single_prob=1/(double)model_length;

	double prob=0;

	for(int i=0;i<similarities.size();i++)
	{
		int level=compute_level(this->get_model_nodes(),similarities[i].first);
		double frac=(double)model_length/(level+1);
		prob+=(double)single_prob*frac;

		std::cout<<"Frac :"<<frac<<std::endl;

		std::cout<<"The level is :"<<level<<std::endl;
	}

	for(int i=0;i<similarities.size();i++)
	{
		std::cout<<"Model :"<<similarities[i].first<<" Target :"<<similarities[i].second<<std::endl;
	}

	std::cout<<"The probabilities are :"<<prob<<std::endl;

	std::cout<<"==============================================================================="<<std::endl;*/

	return 1;
}