#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Define the callback function for the combined point cloud subscriber
void combinedCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& base03_cloud,
                            const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& base05_cloud,
                            const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& base06_cloud,
                            pcl::visualization::PCLVisualizer& viewer) {

  // Combine the three point clouds into a single point cloud
  PointCloudT::Ptr combined_cloud(new PointCloudT);
  *combined_cloud += *base03_cloud;
  *combined_cloud += *base05_cloud;
  *combined_cloud += *base06_cloud;

  // Set the point cloud color
  pcl::visualization::PointCloudColorHandlerCustom<PointT> color(combined_cloud, 255, 0, 0);

  // Add the point cloud to the viewer
  if (!viewer.updatePointCloud(combined_cloud, color, "cloud")) {
    viewer.addPointCloud(combined_cloud, color, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
  }

  // Set the viewer camera position
  viewer.setCameraPosition(0, 0, -2, 0, -1, 0, 0);

  // Spin the viewer
  viewer.spinOnce();
}

int main(int argc, char **argv) {
  // Initialize ROS node
  ros::init(argc, argv, "combined_cloud_viewer");
  ros::NodeHandle nh;

  // Create a PCL Visualizer object
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");

  // Subscribe to the three point cloud topics
  ros::Subscriber sub_base03 = nh.subscribe<PointCloudT>("/base03/ouster/points", 1, boost::bind(combinedCloudCallback, _1, _2, _3, boost::ref(viewer)));
  ros::Subscriber sub_base05 = nh.subscribe<PointCloudT>("/base05/oustser/points", 1, boost::bind(combinedCloudCallback, _1, _2, _3, boost::ref(viewer)));
  ros::Subscriber sub_base06 = nh.subscribe<PointCloudT>("/base06/ouster/points", 1, boost::bind(combinedCloudCallback, _1, _2, _3, boost::ref(viewer)));

  // Set the viewer background color
  viewer.setBackgroundColor(0, 0, 0);

  // Start the viewer
  while (!viewer.wasStopped()) {
    ros::spinOnce();
    viewer.spinOnce();
    boost::this_thread::sleep(boost::posix_time::microseconds(100));
  }

  return 0;
}

