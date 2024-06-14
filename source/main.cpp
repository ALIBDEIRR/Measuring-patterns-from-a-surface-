#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "visualisation.hpp"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/centroid.h>



using namespace std;
//----------------------------------------------------------------function to calculate the root mean square RMS -------------------------------------------------------------------------
double root_mean_square( vector<double> all_distances){			
						double sum_of_distances_squared = 0.0;
						for (double d: all_distances){
											sum_of_distances_squared += d * d;}
						double mean_of_distances =  sum_of_distances_squared / all_distances.size();
						double rms = sqrt(mean_of_distances);					
						return rms;
}
//----------------------------------------------------------Define the paths for our files ------------------------------------------------------------------------------------------------
string path = "/home/ali.bdeir/Desktop/half_sphere_dimensions/pcd files/";
string pcd_name = "half_sphere_with_plane_res(80).pcd"; // half sphere pcd in global ROI
//string pcd_name = "half_sphere_50.pcd";	// half sphere pcd in local ROI
//string pcd_name = "Brassica_pollen_grain.pcd"; // point cloud for the pollen sample in pollen 3D tutorial
//string pcd_name = "Dimple_1.pcd"; // point cloud for the first dimple , small depth 
//string pcd_name = "fullSphere_4.56.pcd";
//string pcd_name = "fullSphere_5.63.pcd";


//-----------------------------------------------------------------Define variables---------------------------------------------------------------------------------------------------------
#define threshold 0.01     // minimum distace for a point in the dataset to be considered as an inliers.

int t{0};
char confirmation;
float noiseamplitude;

vector<double> distances;
vector<double> dist;
//------------------------------------------------------------------Main function -------ransac.setDistanceThreshold(threshold)--------------------------------------------------------------
int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read(path + pcd_name, *cloud);
    cout << "The Initial Point Cloud has: " << cloud->size() << " data points." << endl;
    cout << "-----------------------------------------------------------------------------"<< endl;
    cout << "Do you want to add noise to the cloud ? (Y/N)  ----->  " ;
    cin >> confirmation;
    if (confirmation == 'y' || confirmation == 'Y') {
        cout << "Enter the value of noise between 0 --> 0.1:  " << endl;
        cin >> noiseamplitude;
        for (size_t i = 0; i < cloud->size(); ++i) {
            (*cloud)[i] = addNormalNoise((*cloud)[i], noiseamplitude);
        }
    }	
    visualisation(cloud);	
    
//------------------------------------------------------------- computeModelCoefficients()   -   PCL SAC MODEL SPHERE  -------------------------------------------------------------------------
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr sac_model(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(sac_model);
    ransac.setDistanceThreshold(threshold); 
    ransac.computeModel();
    std::vector<int> inliers;
    ransac.getInliers(inliers);
    cout << "--------------------------------------------------------------------------"<<endl;
    cout << "Number of points took as inliers : "<< inliers.size() << " data points. " <<endl;
    cout << "--------------------------------------------------------------------------"<<endl;
    Eigen::VectorXf model_coefficients;
    
    ransac.getModelCoefficients(model_coefficients);
     
    pcl::PointXYZ non_optimized_center;
    non_optimized_center.x = model_coefficients[0];
    non_optimized_center.y = model_coefficients[1];
    non_optimized_center.z = model_coefficients[2];
    float non_optimized_radius = model_coefficients[3];
    cout << "Radius of the fitted sphere before opt= " << non_optimized_radius << endl;
    cout << "-------------------------------------------------------------"<<endl;
    
// ------------Here, the model coefficients we obtained form the ransac.computemodel() are used as an input to the optimization fucntion-----------------------------------------------------------------  
    Eigen::VectorXf optimized_coefficients;
    sac_model->optimizeModelCoefficients(inliers, model_coefficients, optimized_coefficients);
    pcl::PointXYZ center;
    center.x = optimized_coefficients[0];
    center.y = optimized_coefficients[1];
    center.z = optimized_coefficients[2];
    float radius = optimized_coefficients[3];
    cout << "Radius of the fitted sphere = " << radius << endl;
    cout << "----------------------------------------------------------------------------"<<endl;
  
    float error_radius;
    error_radius = non_optimized_radius - radius ; 
    cout <<  "Difference between non optimized and optimized radius :" << error_radius << endl; 
// ------------------------------------------------------------Identify intersection points-----------------------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr intersection_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cloud->size(); ++i) {
        pcl::PointXYZ point = cloud->points[i];
        float distance = sqrt(pow(point.x - center.x, 2) + pow(point.y - center.y, 2) + pow(point.z - center.z, 2));
        
        if (abs(distance - radius) < threshold) {
            intersection_cloud->points.push_back(point);
        }
    }    
    cout << "-----------------------------------------------------------------------------"<< endl;
    cout << "Threshold value of filtering: "<< threshold << endl; 
    cout << "-----------------------------------------------------------------------------"<< endl;
    cout << "The number of points fitted by the sphere is : " << intersection_cloud->size() << " data points. " << endl;    
    std::ofstream csvfile;
    csvfile.open("/home/ali.bdeir/Desktop/half_sphere_dimensions/csv files/distances.csv");
    csvfile << "PointIndex,Distance\n"; 
    
    for (size_t i = 0; i < intersection_cloud->size(); ++i) {
        pcl::PointXYZ point = intersection_cloud->points[i];
        float distance = sqrt(pow(point.x - center.x, 2) + pow(point.y - center.y, 2) + pow(point.z - center.z, 2));
        distances.push_back(distance);
        int j = i +1;
        csvfile << j << "," << distance << "\n";
    }    
    csvfile.close();
    cout << "---------------------------------------------------------------------------- "<<endl;
    cout << "The root mean square (RMS) is : " << root_mean_square(distances) << endl;
    cout << "---------------------------------------------------------------------------- "<<endl;
    cout << "Fitting accuaracy : "<<  abs(root_mean_square(distances) - radius) << endl;
    cout << "-----------------------------------------------------------------------------"<<endl;
    
    
    csvfile.open("/home/ali.bdeir/Desktop/half_sphere_dimensions/csv files/distances_t.csv");
    csvfile << "PointIndex,t\n"; 
    for (size_t i = 0; i < intersection_cloud->size(); ++i) {
        pcl::PointXYZ point = intersection_cloud->points[i];
        float distance1= sqrt(pow(point.x - center.x, 2) + pow(point.y - center.y, 2) + pow(point.z - center.z, 2));
        float t = distance1-radius;
        int j = i +1;
        csvfile << j << "," << t << "\n";
    }    
    csvfile.close();
// ------------------------------------------Visualize the point cloud with different colors------------------------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Sphere fitting"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, 255, 255, 255); 
    viewer->addPointCloud<pcl::PointXYZ>(cloud, color_handler, "cloud");
    viewer->addSphere(center, radius, 255, 0, 0, "Sphere fitted to the point cloud");
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setCameraPosition(0.0, 0.0, 0.0,0.0, 0.0, 1.0,0.0, -1.0, 0.0);
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    } 
// - ----------------------------------Visualize only the point that are fitted by the sphere (Intersection cloud)---------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("Sphere fitting"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler1(cloud, 255, 255, 255); 
    viewer1->addPointCloud<pcl::PointXYZ>(intersection_cloud, color_handler1, "cloud");
    viewer1->addSphere(center, radius, 255, 0, 0, "Points that are fitted by the sphere ");
    viewer1->setBackgroundColor(0, 0, 0);
    viewer1->setCameraPosition(0.0, 0.0, 0.0,0.0, 0.0, 1.0,0.0, -1.0, 0.0);
    while (!viewer1->wasStopped()) {
        viewer->spinOnce();
    }
//--------------------------------------------------------- 
    return 0;
}
