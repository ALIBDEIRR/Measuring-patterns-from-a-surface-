#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

pcl::PointXYZ addNormalNoise(const pcl::PointXYZ& point, double stddev);
void visualisation(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

