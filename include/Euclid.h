#ifndef H_EUCLID
#define H_EUCLID

#include "CloudNode.h"

class EuclideanExtractor
{
public:
	std::vector<CloudNode> extract_objects(CloudNode node,int model=0);
};
#endif