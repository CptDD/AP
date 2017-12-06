#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/shot_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include <CloudFilter.hpp>
#include <Segmentor.hpp>

using namespace std;



int main(int argc,char**argv)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>);
	/*pcl::PointCloud<pcl::Normal>::Ptr model_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors(new pcl::PointCloud<pcl::SHOT352>);
	//pcl::PointCloud<pcl::VFHSignature308>::Ptr model_descriptors(new pcl::PointCloud<pcl::VFHSignature308>);*/

	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_model(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCDReader reader;
	pcl::PCDWriter writer;

	if(reader.read(argv[1],*model)==-1)
	{
		cout<<"An error has occured while reading the model!"<<endl;
		return -1;
	}

	CloudFilter::cleanCloud(model);

	//Segmentor::pass_filter_cup(model,filtered_model);
	Segmentor::downsample_model_big(model,filtered_model);
	//Segmentor::pass_filter_scaled_fourth(filtered_model,filtered_model);
	//Segmentor::pass_filter_scaled_fourth_side(filtered_model,filtered_model);
	//Segmentor::pass_filter_side(filtered_model,filtered_model);

	
	std::cout<<"Filtered cloud points :"<<filtered_model->width*filtered_model->height<<std::endl;
	Segmentor::pass_filter_model_big(filtered_model,filtered_model);
	std::cout<<"Filtered cloud points :"<<filtered_model->width*filtered_model->height<<std::endl;
	//Segmentor::pass_filter_fourth_big_side(filtered_model,filtered_model);
	std::cout<<"Filtered cloud points :"<<filtered_model->width*filtered_model->height<<std::endl;

	pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_handler (model, 255, 255, 255);
	viewer.addPointCloud(model,model_handler,"model");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> filtered_model_handler (filtered_model, 255,0,0);
    viewer.addPointCloud (filtered_model, filtered_model_handler, "filtered_model");

    writer.write("big_models/mushroom_down.pcd",*filtered_model);
    //writer.write("scenes_filtered/first_down_1.pcd",*filtered_model);

  	while (!viewer.wasStopped ())
  	{
    	viewer.spinOnce ();
    	sleep(0.1);
  	}



	/*pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr scene_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors(new pcl::PointCloud<pcl::SHOT352>);
	//pcl::PointCloud<pcl::VFHSignature308>::Ptr scene_descriptors(new pcl::PointCloud<pcl::VFHSignature308>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scene(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences);
	//pcl::CorrespondencesPtr model_scene_vfhs  (new pcl::Correspondences);

	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;

	pcl::PCDReader reader;
	pcl::PCDWriter writer;


	if(reader.read(argv[1],*model)==-1)
	{
		cout<<"An error has occured while reading the model!"<<endl;
		return -1;
	}

	if(reader.read(argv[2],*scene)==-1)
	{
		cout<<"An error has occured while reading the scene!"<<endl;
		return -1;
	}

	writer.write("/model/model.pcd",*model);

	CloudFilter::cleanCloud(model);
	CloudFilter::cleanCloud(scene);


	Segmentor::pass_filter_model(model,filtered_model);
	Segmentor::pass_filter_scene(scene,filtered_scene);

	CloudFilter::computeNormals(filtered_model,model_normals);
	CloudFilter::computeNormals(filtered_scene,scene_normals);

	CloudFilter::cloudDownsampling(filtered_model,model_keypoints);
	CloudFilter::cloudDownsampling(filtered_scene,scene_keypoints);

	/*CloudFilter::compute_vfh_features(model,model_normals,model_descriptors);
	CloudFilter::compute_vfh_features(scene,scene_normals,scene_descriptors);

	CloudFilter::compute_vfh_correspondences(model_descriptors,scene_descriptors,model_scene_vfhs);*/


	/*CloudFilter::compute_shot_features(filtered_model,model_normals,model_keypoints,model_descriptors);
	CloudFilter::compute_shot_features(filtered_scene,scene_normals,scene_keypoints,scene_descriptors);

	CloudFilter::compute_correspondences(model_descriptors,scene_descriptors,model_scene_corrs);

	CloudFilter::hough_cluster(filtered_model,model_normals,model_keypoints,filtered_scene,scene_normals,
		scene_keypoints,model_scene_corrs,rototranslations,clustered_corrs);


	CloudFilter::compute_pose(rototranslations);


	pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");

	viewer.addPointCloud (scene, "scene_cloud");
	//pcl::PointCloud<pcl::PointXYZ>::Ptr off_scene_model (new pcl::PointCloud<pcl::PointXYZ>);
  	//pcl::PointCloud<pcl::PointXYZ>::Ptr off_scene_model_keypoints (new pcl::PointCloud<pcl::PointXYZ>);*/

  	/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_handler (model, 255, 255, 255);
	viewer.addPointCloud(model,model_handler,"model");


	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> filtered_model_handler (filtered_model, 255, 255, 128);
    viewer.addPointCloud (filtered_model, filtered_model_handler, "filtered_model");


    for (size_t i = 0; i < rototranslations.size (); ++i)
  	{
    	pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_model (new pcl::PointCloud<pcl::PointXYZ> ());
    	pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

    	std::stringstream ss_cloud;
    	ss_cloud << "instance" << i;

    	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rotated_model_color_handler (rotated_model, 255, 0, 0);
    	viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());
    }
  	

  	while (!viewer.wasStopped ())
  	{
    	viewer.spinOnce ();
    	sleep(0.2);
  	}*/





	return 0;
}