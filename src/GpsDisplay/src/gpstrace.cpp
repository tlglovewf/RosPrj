#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/NavSatFix.h>
#include "Functions.h"
#include <iostream>
#include <string>
   // 建立 pcl 点云
pcl::PointCloud<pcl::PointXYZRGB> cloud;

void gpsdatacallback(const sensor_msgs::NavSatFix& data)
{
   
    static bool first = true;
    pcl::PointXYZRGB point;

    static int ori_x = 0;
    static int ori_y = 0;

    if(first)
    {
        ori_x = Functions::mector_lon(data.longitude);
        ori_y = Functions::mector_lat(data.latitude) ;
        first = false;
    }

    std::string info =  std::to_string(Functions::mector_lon(data.longitude)) + " >< " + std::to_string(Functions::mector_lat(data.latitude)) + "!" ;
    ROS_INFO(info.c_str());

    info =  std::to_string(data.longitude) + " = " + std::to_string(data.latitude) + "!" ;
    ROS_INFO(info.c_str());

    point.x = Functions::mector_lon(data.longitude) - ori_x;
    point.y = Functions::mector_lat(data.latitude)  - ori_y;
    point.z = 0.0;
    point.r = 0.0;
    point.g = 255.0;
    point.b = 0.0;

    cloud.points.push_back(point);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "GpsDataHandle");  

    ros::NodeHandle handle("~");
    
    ROS_INFO("begin to get gps data");
    
    ros::Publisher  trace = handle.advertise<sensor_msgs::PointCloud2>("/gpscloudtrace",100);

    ros::Subscriber sub = handle.subscribe("/gps/fix",100,gpsdatacallback);

    ros::Rate rate(100);

    while(ros::ok())
    {
        ros::spinOnce();

         // 建立ros点云
        sensor_msgs::PointCloud2 output_msg;
        output_msg.header.stamp = ros::Time::now();
        // 将pcl点云转化为ros消息发布
        pcl::toROSMsg(cloud, output_msg);
         // 发布的点云坐标系 rviz中需要对应着修改
        output_msg.header.frame_id = "map";
        trace.publish(output_msg);
        rate.sleep();
    }

    
    return 0;
}