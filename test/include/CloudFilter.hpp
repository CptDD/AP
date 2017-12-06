#ifndef CLOUD_FILTER
#define CLOUD_FILTER

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/correspondence.h>

#include <pcl/features/vfh.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>

#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_cloud.h>

class CloudFilter
{
public:

	static void cleanCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		std::cout<<"===CLEANING THE CLOUD==="<<std::endl;
		std::vector<int> indices;
		std::cout<<"Before :"<<cloud->points.size()<<std::endl;
		pcl::removeNaNFromPointCloud(*cloud,*cloud,indices);
		std::cout<<"After  :"<<cloud->points.size()<<std::endl;
		std::cout<<"========================"<<std::endl;

	}

	static void computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals)
	{
		std::cout<<"===COMPUTING NORMALS==="<<std::endl;
		pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normal_estimator;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;

		normal_estimator.setSearchMethod(tree);
		normal_estimator.setInputCloud(cloud);
		normal_estimator.setKSearch(50);
		normal_estimator.compute(*normals);

		std::cout<<normals->points.size()<<" computed!"<<std::endl;
		std::cout<<"======================="<<std::endl;
	}

	/*static void cloudDownsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoints)
	{
		std::cout<<"===DOWNSAMPLING THE CLOUD==="<<std::endl;

		double cloud_ss=0.01;

		pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
		uniform_sampling.setInputCloud(cloud);
		uniform_sampling.setRadiusSearch(cloud_ss);

		pcl::PointCloud<int> keypointIndices;
		uniform_sampling.compute(keypointIndices);
		pcl::copyPointCloud(*cloud,keypointIndices.points,*cloud_keypoints);

		std::cout<<"Out of :"<<cloud->points.size()<<" keypoints sampled :"<<cloud_keypoints->points.size()<<std::endl;
		std::cout<<"============================"<<std::endl;
	}*/

	/*static void compute_shot_features(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoints,pcl::PointCloud<pcl::SHOT352>::Ptr cloud_descriptors)
	{
		std::cout<<"===COMPUTING SHOT DESCRIPTORS==="<<std::endl;

		double descr_rad=0.02;

		pcl::SHOTEstimationOMP<pcl::PointXYZ,pcl::Normal,pcl::SHOT352> descriptor_estimator;

		descriptor_estimator.setRadiusSearch(descr_rad);
		descriptor_estimator.setInputCloud(cloud_keypoints);
		descriptor_estimator.setInputNormals(cloud_normals);
		descriptor_estimator.setSearchSurface(cloud);
		descriptor_estimator.compute(*cloud_descriptors);

		std::cout<<cloud_descriptors->points.size()<<" descriptors computed!"<<std::endl;
		std::cout<<"==============================="<<std::endl;

	}	

	static void compute_vfh_features(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals,
		pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors)
	{

		std::cout<<"===COMPUTING VFH==="<<std::endl;
		pcl::VFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::VFHSignature308> vfh;

		vfh.setInputCloud(cloud);
		vfh.setInputNormals(normals);

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  		
  		vfh.setSearchMethod (tree);
  		vfh.setNormalizeBins(true);
		vfh.setNormalizeDistance(false);
		vfh.compute (*descriptors);
		std::cout<<"=================="<<std::endl;
	}*/

	static void compute_vfh_correspondences(pcl::PointCloud<pcl::VFHSignature308>::Ptr model_descriptors,
		pcl::PointCloud<pcl::VFHSignature308>::Ptr scene_descriptors,pcl::CorrespondencesPtr model_scene_corrs)
	{
		std::cout<<"===COMPUTING VFHS CORRESPONDENCES==="<<std::endl;
		pcl::KdTreeFLANN<pcl::VFHSignature308> match_search;
  		match_search.setInputCloud (model_descriptors);

  	
  		
  	
  		/*for(size_t i=0;i<308;i++)
  		{
  			std::vector<int> neigh_indices (1);
    		std::vector<float> neigh_sqr_dists (1);

    		/*if(!pcl_isfinite(scene_descriptors->points[i]))
    		{
    			continue;
    		}


  			int found_neighs=match_search.nearestKSearch(scene_descriptors->points[i],1,neigh_indices,neigh_sqr_dists);
  			std::cout<<"Distance :"<<neigh_sqr_dists[0]<<std::endl;

  			if(found_neighs == 1 && neigh_sqr_dists[0]<1000)
  			{
  				pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      			model_scene_corrs->push_back (corr);
  			}
  		}*/

  		/*for(size_t i=0;i<scene_descriptors->size();i++)
  		{
  			std::vector<int> neigh_indices (1);
    		std::vector<float> neigh_sqr_dists (1);
    		if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0]))
    		{
      			continue;
    		}
    		int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    		if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)
    		{
      			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      			model_scene_corrs->push_back (corr);
    		}
  		}
  		std::cout << "Found :" << model_scene_corrs->size () <<" correspondences!"<<std::endl;
		std::cout<<"===================================="<<std::endl;*/
	}

	static void compute_correspondences(pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors,
		pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors,pcl::CorrespondencesPtr model_scene_corrs)
	{
		std::cout<<"===COMPUTING CORRESPONDENCES==="<<std::endl;

		pcl::KdTreeFLANN<pcl::SHOT352> match_search;
  		match_search.setInputCloud (model_descriptors);


  		for (size_t i = 0; i < scene_descriptors->size (); ++i)
  		{
  		
    		std::vector<int> neigh_indices (1);
    		std::vector<float> neigh_sqr_dists (1);
    		if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0]))
    		{
      			continue;
    		}
    		int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    		if(found_neighs == 1 && neigh_sqr_dists[0] < 0.1)
    		{
      			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      			model_scene_corrs->push_back (corr);
    		}
    	}
  			std::cout << "Found :" << model_scene_corrs->size () <<" correspondences!"<<std::endl;
			std::cout<<"==============================="<<std::endl;
	}

	static void hough_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr model,pcl::PointCloud<pcl::Normal>::Ptr model_normals,
		pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints,pcl::PointCloud<pcl::PointXYZ>::Ptr scene,
		pcl::PointCloud<pcl::Normal>::Ptr scene_normals,pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints,
		pcl::CorrespondencesPtr model_scene_corrs,
		std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &rototranslations,
		std::vector<pcl::Correspondences> clustered_corrs)
	{
		std::cout<<"===HOUGH CLUSTERING==="<<std::endl;

  		double ref_rad=0.015;
  		double cluster_size=0.01;
  		double cluster_thr=5;

  		pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf (new pcl::PointCloud<pcl::ReferenceFrame>);
    	pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf (new pcl::PointCloud<pcl::ReferenceFrame>);

    	pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZ,pcl::Normal,pcl::ReferenceFrame> reference_estimator;
    	reference_estimator.setFindHoles(true);
    	reference_estimator.setRadiusSearch(ref_rad);

    	reference_estimator.setInputCloud (model_keypoints);
    	reference_estimator.setInputNormals (model_normals);
    	reference_estimator.setSearchSurface (model);

    	reference_estimator.compute (*model_rf);

    	reference_estimator.setInputCloud (scene_keypoints);
    	reference_estimator.setInputNormals (scene_normals);
    	reference_estimator.setSearchSurface (scene);
    	reference_estimator.compute (*scene_rf);

    	pcl::Hough3DGrouping<pcl::PointXYZ, pcl::PointXYZ, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
    	clusterer.setHoughBinSize (cluster_size);
    	clusterer.setHoughThreshold (cluster_thr);
    	clusterer.setUseInterpolation (true);
    	clusterer.setUseDistanceWeight (false);

    	clusterer.setInputCloud (model_keypoints);
    	clusterer.setInputRf (model_rf);
    	clusterer.setSceneCloud (scene_keypoints);
    	clusterer.setSceneRf (scene_rf);
    	clusterer.setModelSceneCorrespondences (model_scene_corrs);

    	clusterer.recognize (rototranslations, clustered_corrs);
    	std::cout << "Model instances found: " << rototranslations.size () << std::endl;
    	std::cout<<"======================"<<std::endl;
	}

	static void compute_pose(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &rototranslations)
		//std::vector<pcl::Correspondences> clustered_corrs)

	{
		std::cout<<"===COMPUTING POSE==="<<std::endl;
		for (size_t i = 0; i < rototranslations.size (); ++i)
 		{
    		std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    		//std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

    		Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    		Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

    		printf ("\n");
    		printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    		printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    		printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    		printf ("\n");
    		printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  		}

		std::cout<<"===================="<<std::endl;
	}


	template<typename T> static void go()
	{
		std::cout<<"We are here to make some noise!"<<std::endl;
	}

	static void show(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
		std::cout<<cloud->points.size()<<std::endl;
	};


};

#endif