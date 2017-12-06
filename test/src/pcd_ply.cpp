#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>


using namespace std;



int main(int argc,char**argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PLYReader reader;
	pcl::PCDWriter writer;

	if(reader.read(argv[1],*cloud)==-1)
	{
		cout<<"Could not read cloud!"<<endl;
		return -1;
	}

	cout<<cloud->points.size()<<endl;

	writer.write("ply_cloud.pcd",*cloud);
	return 0;
}