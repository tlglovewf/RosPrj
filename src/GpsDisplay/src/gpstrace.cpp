#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "Functions.h"
#include "tf/transform_broadcaster.h"
#include "nmea_msgs/Gprmc.h"
#include "nmea_msgs/Sentence.h"
#include <iostream>
#include <string>
#include <vector>
#include <regex>
#include <sstream>
   // 建立 pcl 点云
pcl::PointCloud<pcl::PointXYZRGB> cloud;


static void pushgpspt(double lon , double lat, bool& isfirst)
{
    pcl::PointXYZRGB point;

    static int ori_x = 0;
    static int ori_y = 0;

    if(isfirst)
    {
        ori_x = Functions::mector_lon(lon);
        ori_y = Functions::mector_lat(lat);
        isfirst = false;
    } 
    point.x = Functions::mector_lon(lon) - ori_x;
    point.y = Functions::mector_lat(lat) - ori_y;
    point.z = 0.0;
    point.r = 0.0;
    point.g = 255.0;
    point.b = 0.0;
    
    cloud.points.push_back(point);
}

void gpsdatacallback(const sensor_msgs::NavSatFix& data)
{
    static bool isfirst = true;
    pushgpspt(data.longitude,data.latitude,isfirst);
}


std::vector<std::string> splitSV(std::string strv, std::string delims = " ")
{
	std::vector<std::string> output;
	size_t first = 0;

	while (first < strv.size())
	{
		const auto second = strv.find_first_of(delims, first);

		if (first != second)
			output.emplace_back(strv.substr(first, second - first));

		if (second == std::string::npos)
			break;

		first = second + 1;
	}

	return output;
}

void gprmccallback(const nmea_msgs::Sentence& data)
{
    static bool isfirst = true;
    //ROS_INFO(data.sentence.c_str());//info.c_str());
    static const std::regex rgx("^\\$GPRMC,[\\d\\.]*,[A|V],(-?[0-9]*\\.?[0-9]+),([NS]*),(-?[0-9]*\\.?[0-9]+),([EW]*),.*");

    if(std::regex_match(data.sentence,rgx))
    {
       
       auto values = splitSV(data.sentence,",");
       
       std::string str = values[3] + "--" + values[5];
       double lon = atof(values[5].c_str()) * 0.01;
       double lat = atof(values[3].c_str()) * 0.01;
       if(values[4] == "S")
        {
            lat *= -1;
        }
        if(values[6] == "W")
        {
            lon *= -1;
        }
       lon = 5 / 3.0f * lon - (int)lon / 3.0;
       lat = 5 / 3.0f * lat - (int)lat / 3.0;

       static double prelon = 0;
       static double prelat = 0;
       if(fabs(prelon-lon) < 1e-6 && fabs(prelat-lat) < 1e-6)
        return;
       ROS_INFO((std::to_string(lon) + " " + std::to_string(lat)).c_str());
       pushgpspt(lon,lat,isfirst);
       prelon = lon;
       prelat = lat;
    }
} 

void imucallback(const sensor_msgs::Imu& data)
{
    std::stringstream ss;
    //ss << data.linear_acceleration.x << " " << data.linear_acceleration.y << " " << data.linear_acceleration.z;
    ss << data.orientation.x << " " << data.orientation.y << " " << data.orientation.z << " " << data.orientation.w;
    ROS_INFO(ss.str().c_str());

    //do some point insert
    

}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "GpsDataHandle");  

    ros::NodeHandle handle("~");
    
    ROS_INFO("begin to get /gps/fix data");
    
    ros::Publisher  trace = handle.advertise<sensor_msgs::PointCloud2>("/gpscloudtrace",100);

    ros::Subscriber sub = handle.subscribe("/gps/fix",10,gpsdatacallback);

    ROS_INFO("begin to get nmea data");

    ros::Subscriber gpssub = handle.subscribe("/nmea_sentence",10,gprmccallback);

    ros::Subscriber imusub = handle.subscribe("/imu_raw",10,imucallback);

    ros::Rate rate(1);
    //ros::WallRate rate(100);
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