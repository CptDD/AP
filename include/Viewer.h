#ifndef H_VIEWER
#define H_VIEWER

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include "CloudNode.h"


class CloudNodeViewer
{
	private:
		boost::shared_ptr<pcl::visualization::PCLVisualizer> visualisation_handler(CloudNode node);
		boost::shared_ptr<pcl::visualization::PCLVisualizer> visualisation_handler(std::vector<CloudNode> nodes);
		boost::shared_ptr<pcl::visualization::PCLVisualizer> graph_vis_handler(std::vector<CloudNode> graph);
		boost::shared_ptr<pcl::visualization::PCLVisualizer> graph_vis_handler_test(std::vector<CloudNode> graph);
		boost::shared_ptr<pcl::visualization::PCLVisualizer> visualisation_handler(CloudNode node,CloudNode node2);
		boost::shared_ptr<pcl::visualization::PCLVisualizer> visualisation_handler(std::vector<CloudNode> target,
			std::vector<CloudNode> query,std::vector<std::pair<int,int> >similars);
		boost::shared_ptr<pcl::visualization::PCLVisualizer> visualisation_handler(CloudNode node,
		CloudNode node2,Eigen::Vector4f center);


	public:
		void view(CloudNode node,bool screenshot=false);
		void view(std::vector<CloudNode> nodes,bool screenshot=false);
		void view(CloudNode node,CloudNode node2,bool screenshot=false);
		void view(CloudNode node,CloudNode node2, Eigen::Vector4f center,bool screenshot=false);
		void view(std::vector<CloudNode> target,std::vector<CloudNode> query, std::vector<std::pair<int,int> >similars,bool screenshot=false);
		void view_graph(std::vector<CloudNode> graph,bool screenshot=false);
		void view_graph_test(std::vector<CloudNode> graph,bool screenshot=false);

};
#endif