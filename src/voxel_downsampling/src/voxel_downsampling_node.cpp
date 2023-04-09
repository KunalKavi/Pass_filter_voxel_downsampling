#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

class VoxelGrid
{
  public:
    VoxelGrid()
    {
      pub_ = nh_.advertise<sensor_msgs::PointCloud2>("downsampled_points", 1);
      sub_ = nh_.subscribe("/base03/ouster/points", 1, &VoxelGrid::cloudCallback, this);
      sub2_ = nh_.subscribe("/base05/ouster/points", 1, &VoxelGrid::cloudCallback, this);
      sub3_ = nh_.subscribe("/base06/ouster/points", 1, &VoxelGrid::cloudCallback, this);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        // Convert ROS message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // Perform pass-through filter on point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 3.0);
        pass.filter(*cloud_filtered);

        // Perform voxel downsampling on filtered point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(cloud_filtered);
        sor.setLeafSize(1.0f, 1.0f, 1.0f);
        sor.filter(*cloud_downsampled);

        // Convert voxel-downsampled point cloud back to ROS message and publish
        sensor_msgs::PointCloud2 downsampled_cloud_msg;
        pcl::toROSMsg(*cloud_downsampled, downsampled_cloud_msg);
        downsampled_cloud_msg.header = cloud_msg->header;
        pub_.publish(downsampled_cloud_msg);

        // Visualize voxel-downsampled point cloud in real time using pcl_viewer
        pcl::visualization::PCLVisualizer viewer("Voxel-downsampled Point Cloud Viewer");
        viewer.addPointCloud<pcl::PointXYZ>(cloud_downsampled, "downsampled_cloud");
        viewer.spinOnce(); // Update viewer
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Subscriber sub2_;
    ros::Subscriber sub3_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "voxel_downsampling");
  VoxelGrid vg;
  ros::spin();
  return 0;
}

