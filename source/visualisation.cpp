#include<iostream>
#include<vector>
#include <random>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include<X11/Xatom.h>



using namespace std;

// --------------------------------------------------------------------------

void visualisation(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
  	pcl::visualization::PCLVisualizer viewer ("Half sphere Virtual point cloud Raduis = 4.56");
  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud, 0, 0,255);
  	viewer.addPointCloud<pcl::PointXYZ> (cloud, cloud_color_handler, "sample cloud");
  	viewer.setBackgroundColor(0,0,0);
  	viewer.addCoordinateSystem (1.0);
  	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  	// viewer.registerPointPickingCallback(evenement_click, (void*)&viewer);
  	viewer.addCoordinateSystem (1.0);
        viewer.addCoordinateSystem (1.0, "global");
        viewer.setShowFPS(true);
  	while (!viewer.wasStopped ()){
  		viewer.spinOnce ();
  	}
}

// --------------------------------------------------------------------------

pcl::PointXYZ addNormalNoise(const pcl::PointXYZ& point, double stddev) {
    static std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, stddev);
    pcl::PointXYZ noisy_point;
    noisy_point.x = point.x + distribution(generator);
    noisy_point.y = point.y + distribution(generator);
    noisy_point.z = point.z + distribution(generator);
    return noisy_point;
}

// --------------------------------------------------------------------------

