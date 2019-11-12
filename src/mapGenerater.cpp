#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <dynamic_reconfigure/server.h>
// #include <exdata/mapGeneraterConfig.h>
#include <exdata/mapGeneraterConfig.h>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#define PI 3.14159265358979323846

double gauss_fcn(const Eigen::VectorXd& x, const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance);
void callback(exdata::mapGeneraterConfig &config, uint32_t level);
int serectColor(float value, float minValue, float maxValue, int palletSize);

Eigen::VectorXd gx = Eigen::VectorXd::Zero(2);
Eigen::VectorXd gmean = Eigen::VectorXd::Ones(2);
Eigen::MatrixXd gcovariance = Eigen::MatrixXd::Ones(2,2);
nav_msgs::OccupancyGrid og;
std::vector<float> colorMap;
pcl::PointCloud<pcl::PointXYZRGB> cloud;
sensor_msgs::PointCloud2 cloud_msg;

int main (int argc, char** argv)
{
    ros::init (argc, argv, "mapGenerater");
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("output", 1);
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("output_pcl", 1);
    og.header.stamp = ros::Time::now();
    og.header.frame_id = "/map";
    og.info.map_load_time = ros::Time::now();
    og.info.resolution = 0.05;
    og.info.width = 160;
    og.info.height = 160;
    og.info.origin.orientation.w = 0.0;
    og.info.origin.orientation.x = 1.0;
    og.info.origin.orientation.y = 1.0;
    og.info.origin.orientation.z = 1.0;
    og.info.origin.position.x = 0;
    og.info.origin.position.y = 0;
    og.info.origin.position.z = 0;
    og.data.resize(og.info.width * og.info.height);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "/map";
    cloud.points.resize(og.info.width * og.info.height);
    cloud.width=cloud.points.size();
    cloud.height=1;
    nh.param("colorMap/data", colorMap, colorMap);
    if(!(colorMap.size() > 0)) {
        ROS_ERROR_STREAM("Cannot load rosparam colorMap/data");
        return -1;
    }
    colorMap.resize(colorMap.size() - (colorMap.size() % 3)); //要素数が3の倍数(RGB)になるようにリサイズ
    // ROS_INFO_STREAM("matrix " << gx(0) << " " << gx(1) << " " << gmean(0) << " " << gmean(1) << " " << gcovariance(0,0) << " " << gcovariance(0,1) << " " << gcovariance(1,0) << " " << gcovariance(1,1) );
    dynamic_reconfigure::Server<exdata::mapGeneraterConfig> server;
    dynamic_reconfigure::Server<exdata::mapGeneraterConfig>::CallbackType fc;
    fc = boost::bind(&callback, _1, _2);
	server.setCallback(fc);
    
    for(int row = 0; row < og.info.height; ++row){
        for(int col = 0; col < og.info.width; ++col){
            gx(0) = (double)row*og.info.resolution + og.info.resolution/2;
            gx(1) = (double)col*og.info.resolution + og.info.resolution/2;
            // ROS_INFO_STREAM(gauss_fcn(gx, gmean, gcovariance));
            og.data[og.info.width*row+col] = (int)(gauss_fcn(gx, gmean, gcovariance)*100.0);
        }
    }

    //pointcloudでやる場合
    // int colorIndex;
    // int ptIndex;
    // int value;
    // double gauss;
    // int max_value;
    // int min_value = 0;
    // gx(0) = 0;
    // gx(1) = 0;
    // max_value = (int)(gauss_fcn(gx, gmean, gcovariance)*100.0);
    // for(int row = 0; row < og.info.height; ++row){
    //     for(int col = 0; col < og.info.width; ++col){
    //         gx(0) = (double)(row-(int)(og.info.height/2))*og.info.resolution + og.info.resolution/2;
    //         gx(1) = (double)(col-(int)(og.info.width/2))*og.info.resolution + og.info.resolution/2;
    //         ptIndex = row*og.info.height + col;
    //         cloud.points[ptIndex].x = gx(0);
    //         cloud.points[ptIndex].y = gx(1);
    //         cloud.points[ptIndex].z = 0.0;
    //         value = (int)(gauss_fcn(gx, gmean, gcovariance)*100.0);
    //         if(max_value < value){
    //             max_value = value;
    //         }
    //         colorIndex = serectColor(value, 0, max_value, colorMap.size()/3)*3;
    //         cloud.points[ptIndex].r = (uint8_t)(colorMap[colorIndex] * 255);
    //         cloud.points[ptIndex].g = (uint8_t)(colorMap[colorIndex+1] * 255);
    //         cloud.points[ptIndex].b = (uint8_t)(colorMap[colorIndex+2] * 255);
    //     }
    // }
    
    // pcl::toROSMsg(cloud, cloud_msg);
    // cloud_msg.header.stamp = ros::Time::now();
    // cloud_msg.header.frame_id = "/map";
    
    ros::Rate loop_rate(20);
    while(ros::ok()){
        pub.publish(og);
        pcl_pub.publish(cloud_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
}

double gauss_fcn(const Eigen::VectorXd& x, const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance){
    // ROS_INFO_STREAM("gauss_fcn " << x(0) << " " << x(1) << " " << mean(0) << " " << mean(1) << " " << covariance(0,0) << " " << covariance(0,1) << " " << covariance(1,0) << " " << covariance(1,1) );
    double result, index, scalar, matrix_scalar, matrix_double;
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(1,1);
    index = (double)(x.rows());
    scalar = 1.0 / (std::pow(std::sqrt(2.0*PI), index) * std::sqrt(covariance.determinant()));
    matrix = (x-mean).transpose()*covariance.inverse()*(x-mean);
    matrix_scalar = matrix(0,0);
    matrix_double = std::exp( -1.0 / 2.0*matrix_scalar);
    result = scalar*matrix_double;
    return result;
}

void callback(exdata::mapGeneraterConfig &config, uint32_t level){
    // ROS_INFO_STREAM("config_callback");
    gmean(0) = config.mean_x;
    gmean(1) = config.mean_y;
    // ROS_INFO_STREAM("config_callback2");
    gcovariance(0,0) = config.covariance_x1y1;
    gcovariance(0,1) = config.covariance_x1y2;
    gcovariance(1,0) = config.covariance_x2y1;
    gcovariance(1,1) = config.covariance_x2y2;
    og.info.origin.orientation.w = config.qw;
    og.info.origin.orientation.x = config.qx;
    og.info.origin.orientation.y = config.qy;
    og.info.origin.orientation.z = config.qz;
    og.info.origin.position.x = config.ox;
    og.info.origin.position.y = config.oy;
    og.info.origin.position.z = config.oz;

    //コストマップでやる場合 不具合がある
    for(int row = 0; row < og.info.height; ++row){
        for(int col = 0; col < og.info.width; ++col){
            gx(0) = (double)row*og.info.resolution + og.info.resolution/2;
            gx(1) = (double)col*og.info.resolution + og.info.resolution/2;
            og.data[og.info.width*row+col] = config.value;//(int8_t)(gauss_fcn(gx, gmean, gcovariance)*100.0);
            // if(og.data[og.info.width*row+col] > 100){
            //     ROS_INFO_STREAM("MAX VALUE " << (int)og.data[og.info.width*row+col]);
            //     og.data[og.info.width*row+col] = 100;
            // }    
        }
    }

    //pointcloudでやる場合
    int colorIndex;
    int ptIndex;
    int value;
    double gauss;
    int max_value;
    int min_value = 0;
    gx(0) = config.mean_x;
    gx(1) = config.mean_y;
    max_value = (int)(gauss_fcn(gx, gmean, gcovariance)*100.0);
    for(int row = 0; row < og.info.height; ++row){
        for(int col = 0; col < og.info.width; ++col){
            gx(0) = (double)(row-(int)(og.info.height/2))*og.info.resolution + og.info.resolution/2;
            gx(1) = (double)(col-(int)(og.info.width/2))*og.info.resolution + og.info.resolution/2;
            ptIndex = row*og.info.width + col;
            gauss = gauss_fcn(gx, gmean, gcovariance);
            value = (int)(gauss*100.0);
            if(max_value < value){
                max_value = value;
            }
            cloud.points[ptIndex].x = gx(0);
            cloud.points[ptIndex].y = gx(1);
            cloud.points[ptIndex].z = gauss;
            colorIndex = serectColor(value, 0, max_value, colorMap.size()/3)*3;
            cloud.points[ptIndex].r = (uint8_t)(colorMap[colorIndex] * 255);
            cloud.points[ptIndex].g = (uint8_t)(colorMap[colorIndex+1] * 255);
            cloud.points[ptIndex].b = (uint8_t)(colorMap[colorIndex+2] * 255);
        }
    }

    
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "/map";
}



int serectColor(float value, float minValue, float maxValue, int palletSize){
    if(value <= minValue){
        return 0;
    }else if(maxValue <= value){
        return palletSize-1;
    }
    float range = (maxValue - minValue)/palletSize;
    return (int)((value - minValue)/range);
}