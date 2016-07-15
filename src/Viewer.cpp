#include "Viewer.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <ctime>


boost::shared_ptr<pcl::visualization::PCLVisualizer> CloudNodeViewer::visualisation_handler(CloudNode node)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("PCL Visualiser"));

  viewer->setBackgroundColor(255,255,255);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> first_color(node.get_cloud(), 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (node.get_cloud(), first_color,"original cloud");  
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original cloud");
  return viewer;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> CloudNodeViewer::visualisation_handler(std::vector<CloudNode> nodes)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("PCL Visualiser"));

  viewer->setBackgroundColor(255,255,255);

  for(int i=0;i<nodes.size();i++)
  {		
  		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> first_color(nodes[i].get_cloud(), 255, 0, 0);
  		std::stringstream ss;
  		ss<<"Cloud "<<i;
  		viewer->addPointCloud<pcl::PointXYZ> (nodes[i].get_cloud(), first_color,ss.str());
  }

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud 0");
 
  return viewer;
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> CloudNodeViewer::visualisation_handler(CloudNode node,CloudNode node2)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("PCL Visualiser"));

  viewer->setBackgroundColor(255,255,255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> first_color(node.get_cloud(), 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (node.get_cloud(), first_color,"First cloud");
 
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> second_color(node2.get_cloud(), 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (node2.get_cloud(), second_color,"Second cloud");

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "First cloud");
 
  return viewer;
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> CloudNodeViewer::graph_vis_handler_test(std::vector<CloudNode> graph)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("PCLVisualizer"));

  viewer->setBackgroundColor(255,255,255);

  std::string id;
  for(int i=0;i<graph.size();i++)
  {
      std::stringstream ss,text_ss;
      ss<<"CloudNode_"<<i;
      text_ss<<i+1;

      id=ss.str();
      Eigen::Vector4f c1;
      CloudNode current_element=graph[i];
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> first_color(current_element.get_cloud(),0,255,0);
      viewer->addPointCloud(current_element.get_cloud(),first_color,ss.str());

      std::vector<int> indices=current_element.get_connected_indices();
      std::cout<<"Indices :"<<indices.size()<<std::endl;
      
      for(int j=0;j<indices.size();j++)
      {
        if(indices[j]!=i)
        {
          std::stringstream sl;
          sl<<"Line_"<<i<<"_"<<indices[j];
      
         c1=current_element.get_cloud_center();
          Eigen::Vector4f c2=graph[indices[j]].get_cloud_center();

          pcl::PointXYZ center1;
          center1.x=c1.x();
          center1.y=c1.y();
          center1.z=c1.z();
      
          pcl::PointXYZ center2;
          center2.x=c2.x();
          center2.y=c2.y();
          center2.z=c2.z();

          std::cout<<sl.str()<<" "<<center1.x<<" "<<center2.x<<" "<<std::endl;
          viewer->addLine(center1,center2,255,0,0,sl.str());
        }
      }
      pcl::PointXYZ text_pos;
      text_pos.x=c1.x();
      text_pos.y=c1.y();
      text_pos.z=c1.z();

      viewer->addText3D(text_ss.str(),text_pos,0.02,0,0,0,id);
  }

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "CloudNode_0");

  return viewer;

}



boost::shared_ptr<pcl::visualization::PCLVisualizer> CloudNodeViewer::graph_vis_handler(std::vector<CloudNode> graph)
{
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("PCL Visualiser"));
   viewer->setBackgroundColor(255,255,255);
   CloudNode first_element=graph[0];
   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> first_color(graph[0].get_cloud(),0,255,0);
   viewer->addPointCloud(first_element.get_cloud(),first_color,"first_element"); 

   for(int i=0;i<graph.size();i++)
   {
   		std::vector<int> indices=graph[i].get_connected_indices();

      std::string id="1";
   		for(int j=0;j<indices.size();j++)
   		{
     		 CloudNode temp=graph[indices[j]];

     		 std::stringstream ss,st;
      		 ss<<"Connected_i_j"<<i<<" "<<j;
      		 viewer->addPointCloud(temp.get_cloud(),first_color,ss.str());
     		 ss<<"Liner";
         st<<i+1;

     		Eigen::Vector4f c1=first_element.get_cloud_center();
     		Eigen::Vector4f c2=graph[indices[j]].get_cloud_center();

     		pcl::PointXYZ center1;
     		center1.x=c1.x();
     		center1.y=c1.y();
     		center1.z=c1.z();
      
     		pcl::PointXYZ center2;
     		center2.x=c2.x();
     		center2.y=c2.y();
     		center2.z=c2.z();

        pcl::PointXYZ pos;
        pos.x=center1.x;
        pos.y=center1.y;
        pos.z=center1.z;

     		 viewer->addLine(center1,center2,255,0,0,ss.str());
         viewer->addText3D(st.str(),center2,0.05,0,0,0,id);
         id+="1";
     	}
   }

   return viewer;
}



boost::shared_ptr<pcl::visualization::PCLVisualizer> CloudNodeViewer::visualisation_handler(std::vector<CloudNode> target,
			std::vector<CloudNode> query,std::vector<std::pair<int,int> >similars)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("PCL Visualiser"));

	 viewer->setBackgroundColor(255,255,255);
	
  int r=0,g=0,b=0;
  int n_r,n_g,n_b;

  int old_s=r+g+b;

  std::cout<<"Visualizing !"<<std::endl;

  for(int i=0;i<similars.size();i++)
  {
  	 do
  	 {
  	 	r=rand()%255;
  	 	g=rand()%255;
  	 	b=rand()%255;

  	 }while((r+g+b)==old_s && (r+g+b)<=0);

  	 old_s=r+g+b;
  	 cout<<r<<" "<<g<<" "<<b<<endl;
  	 std::stringstream st,sq,ls;

  	 st<<"Target_S_"<<i;
  	 sq<<"Query_S_"<<i;
     ls<<"Line_T_Q_"<<i;

     Eigen::Vector4f c1=target[similars[i].first].get_cloud_center();
     Eigen::Vector4f c2=query[similars[i].second].get_cloud_center();

     pcl::PointXYZ center1;
     center1.x=c1.x();
     center1.y=c1.y();
     center1.z=c1.z();

     pcl::PointXYZ center2;
     center2.x=c2.x();
     center2.y=c2.y();
     center2.z=c2.z();


  	 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> first_color(target[similars[i].first].get_cloud(), r,g,b);

  	 viewer->addPointCloud<pcl::PointXYZ> (target[similars[i].first].get_cloud(), first_color,st.str());
  	 viewer->addPointCloud<pcl::PointXYZ> (query[similars[i].second].get_cloud(), first_color,sq.str());

     viewer->addLine(center1,center2,0,0,0,ls.str());
  }
  
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Target_S_0");
 
  return viewer;

}


boost::shared_ptr<pcl::visualization::PCLVisualizer> CloudNodeViewer::visualisation_handler(CloudNode node,
		CloudNode node2,Eigen::Vector4f center)
{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("PCL Visualiser"));

		double scale=0.5;

  		viewer->setBackgroundColor(255,255,255);

  		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> first_color(node.get_cloud(), 0, 255, 0);
  		viewer->addPointCloud<pcl::PointXYZ> (node.get_cloud(), first_color,"original cloud");

  		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> second_color(node.get_cloud(), 255, 0, 0);
  		viewer->addPointCloud<pcl::PointXYZ> (node2.get_cloud(), second_color,"segmented cloud");

  		viewer->addCoordinateSystem(scale,center.x(),center.y(),center.z());

  		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original cloud");
		return viewer;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> CloudNodeViewer::visualisation_handler(CloudNode node,pcl::PointXYZ min,
  pcl::PointXYZ max)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("PCL Visualiser"));

    viewer->setBackgroundColor(255,255,255);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(node.get_cloud(),0,255,0);
    viewer->addPointCloud<pcl::PointXYZ>(node.get_cloud(),color_handler,"cloud");


    viewer->addArrow(min,max,0,0,0,"liner");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

    return viewer;

}




void CloudNodeViewer::view(std::vector<CloudNode> target,std::vector<CloudNode> query, std::vector<std::pair<int,int> >similars,
  bool screenshot)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer=visualisation_handler(target,query,similars);
	while(!viewer->wasStopped())
	{
	   viewer->spinOnce (100);
     boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
  if(screenshot)
  {
   time_t t = time(0);
    struct tm * now = localtime( & t );
    std::stringstream ss;
    ss<<"Similarities_"<<asctime(now);
    viewer->saveScreenshot(ss.str());
  }
}




void CloudNodeViewer::view(CloudNode node,bool screenshot)
{
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
   viewer=visualisation_handler(node);
   while(!viewer->wasStopped())
   {
       viewer->spinOnce (100);
       boost::this_thread::sleep (boost::posix_time::microseconds (100000));
   }
  if(screenshot)
  {
    time_t t = time(0);
    struct tm * now = localtime( & t );
    std::stringstream ss;
    ss<<"Single_"<<asctime(now);
    viewer->saveScreenshot(ss.str());
  }
}

void CloudNodeViewer::view(std::vector<CloudNode> nodes,bool screenshot)
{
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
   viewer=visualisation_handler(nodes);
   while(!viewer->wasStopped())
   {
       viewer->spinOnce (100);
       boost::this_thread::sleep (boost::posix_time::microseconds (100000));
   }
   if(screenshot)
   {
    time_t t = time(0);
    struct tm * now = localtime( & t );
    std::stringstream ss;
    ss<<"Object_"<<asctime(now);
    viewer->saveScreenshot(ss.str());
   }
}

void CloudNodeViewer::view(CloudNode node,CloudNode node2,bool screenshot)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer=visualisation_handler(node,node2);
	while(!viewer->wasStopped())
	{
		viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
  if(screenshot)
   {
    time_t t = time(0);
    struct tm * now = localtime( & t );
    std::stringstream ss;
    ss<<"Two_clouds_"<<asctime(now);
    viewer->saveScreenshot(ss.str());
   }
}



void CloudNodeViewer::view_graph(std::vector<CloudNode> graph,bool screenshot)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer=graph_vis_handler(graph);

	while(!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds (100000));
	}
  if(screenshot)
   {
    time_t t = time(0);
    struct tm * now = localtime( & t );
    std::stringstream ss;
    ss<<"Graph_"<<asctime(now);
    viewer->saveScreenshot(ss.str());
   }
}


void CloudNodeViewer::view_graph_test(std::vector<CloudNode> graph,bool screenshot)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer=graph_vis_handler_test(graph);

  while(!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds (100000));
  }
  if(screenshot)
   {
    time_t t = time(0);
    struct tm * now = localtime( & t );
    std::stringstream ss;
    ss<<"Graph_"<<asctime(now);
    viewer->saveScreenshot(ss.str());
   }
}

void CloudNodeViewer::view(CloudNode node,CloudNode node2,Eigen::Vector4f center,bool screenshot)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer=visualisation_handler(node,node2,center);

	while(!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds (100000));
	}
  if(screenshot)
  {
    time_t t = time(0);
    struct tm * now = localtime( & t );
    std::stringstream ss;
    ss<<"Center_"<<asctime(now);
    viewer->saveScreenshot(ss.str());
  }
}


void CloudNodeViewer::view_line_test(CloudNode node,pcl::PointXYZ min,pcl::PointXYZ max,bool screenshot)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

  viewer=visualisation_handler(node,min,max);

  while(!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds (100000));
  }

  if(screenshot)
  {
    time_t t=time(0);
    struct tm *now=localtime(& t);
    std::stringstream ss;
    ss<<"Length_estimation"<<asctime(now);
    viewer->saveScreenshot(ss.str());
  }
}