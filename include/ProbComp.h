#ifndef H_PROB
#define H_PROB

#include "CloudNode.h"

class ProbComp
{

private:
	std::vector<CloudNode> model_nodes;
	std::map<int,double> level_probs;

	void setup_probabilities(int model_length);

public:
	ProbComp(std::vector<CloudNode> model_nodes):model_nodes(model_nodes){};
	ProbComp(){}


	void set_model_nodes(std::vector<CloudNode> model_nodes){this->model_nodes=model_nodes;}
	std::vector<CloudNode> get_model_nodes(){return this->model_nodes;}

	double compute_probability(std::vector<std::pair<int,int> >similarities,int model_length);
	double compute_level_probability(std::vector<std::pair<int,int> >similarities,int model_length);


};



#endif