#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <turtlesim/Pose.h>

#if 0
int main(int argc, char **argv)
{
    ros::init(argc,argv,"point_cloud_node");
    
    ros::NodeHandle nh("~");

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_publisher_topic",1000);

    ros::Rate rate(10000);

    uint8_t num_points = 1000;

    // 建立 pcl 点云
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    // 点云初始化
    cloud.points.resize(num_points);
    // 建立ros点云
    sensor_msgs::PointCloud2 output_msg;
    // ros::ok() 检查上面创建的节点是否还在
    while (ros::ok()){
        // 调用ros获取时间的接口，获取系统时间
        output_msg.header.stamp = ros::Time::now();
        // 创建三个绿色的点
        for (int i = 0; i < num_points; i++)
        {
            cloud.points[i].x = i;
            cloud.points[i].y = i;
            cloud.points[i].z = i;
            cloud.points[i].r = 0; 
            cloud.points[i].g = 255;
            cloud.points[i].b = 0;
        }
        // 将pcl点云转化为ros消息发布
        pcl::toROSMsg(cloud, output_msg);
        // 发布的点云坐标系 rviz中需要对应着修改
        output_msg.header.frame_id = "map";
        // 发布
        pub.publish(output_msg);
        rate.sleep();
    }
    ros::spin();

    return 0;
}
#else

void posecallback(const turtlesim::PoseConstPtr& msg)
{
    static tf::TransformBroadcaster br;

    tf::Transform trans;

    trans.setOrigin(tf::Vector3(msg->x,msg->y,0.0));

    tf::Quaternion qua;

    qua.setRPY(0,0,msg->theta);

    trans.setRotation(qua);

    br.sendTransform(tf::StampedTransform(trans,ros::Time::now(),"world","turtle1"));
}
#pragma region  //tf test
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tfexample");
    ros::NodeHandle node;

    ros::Rate rate(10);
    //ros::WallRate rate(100);
    tf::TransformBroadcaster br;
    tf::Transform trans;
    trans.setOrigin(tf::Vector3(20,10,0));

    tf::Quaternion qua;

    qua.setRPY(0,0,0);

    trans.setRotation(qua);

    while(node.ok())
    {
        #if 0
        ros::WallTime wt = ros::WallTime::now();
        br.sendTransform(tf::StampedTransform(trans,ros::Time(wt.sec, wt.nsec),"map","velodyne"));
        #else
        br.sendTransform(tf::StampedTransform(trans,ros::Time::now(),"map","velodyne"));
        #endif
        rate.sleep();
    }
    return 0;
}
#endif

#pragma endregion   