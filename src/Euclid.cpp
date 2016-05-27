#include "CloudNode.h"
#include "Euclid.h"

#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>





std::vector<CloudNode> EuclideanExtractor::extract_objects(CloudNode node,int model)
{
	std::vector<CloudNode> objects;
	std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extractor;
    pcl::ExtractIndices<pcl::PointXYZ> point_extractor;
	
  if(model==0)
  {
	  cluster_extractor.setClusterTolerance(0.02);
	  cluster_extractor.setMinClusterSize(150);
  }else
  {
    cluster_extractor.setClusterTolerance(0.03);
    cluster_extractor.setMinClusterSize(3000);
  }
	cluster_extractor.setInputCloud(node.get_cloud());
    cluster_extractor.extract(cluster_indices);

    std::cout<<"Cluster indices size :"<<cluster_indices.size()<<std::endl;

    for(int i=0;i<cluster_indices.size();i++)
    {
    	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_extracted_object(new pcl::PointCloud<pcl::PointXYZ>);
        point_extractor.setInputCloud(node.get_cloud());
        point_extractor.setNegative(false);
		point_extractor.setIndices(boost::make_shared<pcl::PointIndices>(cluster_indices[i]));
     	point_extractor.filter(*temp_extracted_object);
          		//cout<<"Cloud :"<<i+1<<" has size :"<<temp_extracted_object->points.size()<<endl;
          		//stringstream stream;
          		//stream<<"final_cloud_"<<i+1<<".pcd";
          		//writer.write(stream.str(),*temp_extracted_object,false);
          		//view(temp_extracted_object);
        objects.push_back(temp_extracted_object);
    }	
    std::cout<<"Objects size :"<<objects.size()<<std::endl;
    return objects;
}