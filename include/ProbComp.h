#ifndef H_PROB
#define H_PROB

#include "CloudNode.h"

class ProbComp
{
public:
	double compute_probability(std::vector<std::pair<int,int> >similarities,int model_length);

};



#endif