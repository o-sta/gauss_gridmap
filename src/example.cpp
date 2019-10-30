#include <ros/ros.h>
#include <ros/param.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <geometry_msgs/Point32.h>

ros::Publisher pub;
geometry_msgs::Point32 gLeafSize;

void cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud)
{
    pcl::PCLPointCloud2 cloud_filtered;
    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(gLeafSize.x, gLeafSize.y, gLeafSize.z);
    sor.filter(cloud_filtered);
    // Publish the data
    pub.publish (cloud_filtered);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh("~");
    gLeafSize.x = 0.1;
    gLeafSize.y = 0.1;
    gLeafSize.z = 0.1;
    //ros::param::param("exdata/leaf/x", gLeafSize.x, gLeafSize.x);
    // ros::param::param("exdata/leaf/y", gLeafSize.y, gLeafSize.y);
    // ros::param::param("exdata/leaf/z", gLeafSize.z, gLeafSize.z);
    nh.param("exdata/leaf/x", gLeafSize.x, gLeafSize.x);
    nh.param("exdata/leaf/y", gLeafSize.y, gLeafSize.y);
    nh.param("exdata/leaf/z", gLeafSize.z, gLeafSize.z);
    ros::param::set("exdata/leaf/rp", "ros::param::set function");
    nh.setParam("exdata/leaf/np", "nh.setParam function");
    ROS_INFO_STREAM("Import" << "LeafSize = " << gLeafSize.x << " " << gLeafSize.y << " " << gLeafSize.z << " from exdata/leaf");
    std::vector<int> v;
    nh.getParam("exdata/colordata", v);
    ROS_INFO_STREAM(v.size());
    ROS_INFO_STREAM(v[5]);
    for(auto v_i : v){
        ROS_INFO_STREAM(v_i);
    }
    // ROS_INFO_STREAM("array data size :" << v.size() << " " << v[0] <<  " " << v[0]);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("zed/point_cloud/cloud_registered", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<pcl::PCLPointCloud2> ("output", 1);

    // Spin
    ros::spin ();
}