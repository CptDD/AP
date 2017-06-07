#include "ProbComp.h"


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