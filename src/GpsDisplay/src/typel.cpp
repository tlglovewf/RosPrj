#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <fstream>
#include <sstream>

#include "Functions.h"

class ImuHandler final
{
public:
    ImuHandler()
    {
        #if 0
        //for test pose
        _pub = _hdl.advertise<visualization_msgs::Marker>("imumaker",10);
        _sub = _hdl.subscribe("/imu_raw",50,&ImuHandler::_handleimudata,this);
        #else
        _pub = _hdl.advertise<sensor_msgs::PointCloud2>("imutrace",10);
        _sub = _hdl.subscribe("/imu/data",1,&ImuHandler::_calimupose,this);
        #endif 
    }
            
    //初始速度.姿态.零偏 影响都比较大
    struct ImuStatus
    {
       Eigen::Vector3d      gyro_bias   = Eigen::Vector3d(0,0,0);
       Eigen::Vector3d      acc_bias    = Eigen::Vector3d(0,0,0);

       Eigen::Vector3d      pos         = Eigen::Vector3d(0,0,0);

       Eigen::Vector3d      grav        = Eigen::Vector3d(0,0,9.81);

       Eigen::Vector3d      speed       = Eigen::Vector3d(0,0,0);
       Eigen::Quaterniond   qua         = Eigen::Quaterniond(1, 0, 0, 0);

       Eigen::Quaterniond   pose        = Eigen::Quaterniond(1,0,0,0);
       //add cov 
    };

    struct ImuData
    {
        double          time;
        Eigen::Vector3d rpy;
        
        Eigen::Vector3d gyro;
        Eigen::Vector3d acc;

        Eigen::Vector3d pos;
    };

public:


    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        return dq;
    }

    void _predictimu(const  ImuData& predata, const ImuData& curdata)
    {
        double dt = curdata.time - predata.time;
        //中指积分
        Eigen::Vector3d half_gyro = (predata.gyro + curdata.gyro) * 0.5 - imustatus.gyro_bias;  
        
        Eigen::Vector3d un_acc_0 = imustatus.qua * (predata.acc - imustatus.acc_bias) ;//- imustatus.grav ;

        imustatus.qua = imustatus.qua * deltaQ(half_gyro * dt);
        imustatus.qua.normalize();

        Eigen::Vector3d un_acc_1 = imustatus.qua  * (curdata.acc - imustatus.acc_bias) ;// - imustatus.grav;

        Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

        std::stringstream ss;

        //长距离 imu pos积分误差大  
        imustatus.pos = imustatus.pos + imustatus.speed * dt + 0.5 * dt * dt * un_acc;
        imustatus.speed = imustatus.speed + un_acc * dt;
 
        ss << dt << " " << imustatus.pos.x() << " " << imustatus.pos.y() << " " << imustatus.pos.z() << std::endl;
        _saveimupose(imustatus.pos);
        // ROS_INFO(ss.str().c_str());
    }


    void _calimupose(const sensor_msgs::Imu& data)
    {
        static int index = 0;
        static ImuData lastdata;
        static ImuData curdata;
        imustatus.pose = Eigen::Quaterniond(data.orientation.w,data.orientation.x,data.orientation.y,data.orientation.z);
        if(index++ < 1)
        {
            lastdata.acc  =  Eigen::Vector3d(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z);
            lastdata.gyro =  Eigen::Vector3d(data.angular_velocity.x,data.angular_velocity.y,data.angular_velocity.z);
            lastdata.time =  data.header.stamp.toSec();
            imustatus.qua = imustatus.pose;
            imustatus.speed = Eigen::Vector3d( -0.0234549716,-0.000400833029,0.00864264462);
            curdata = lastdata;
            return;
        }
        else
        {
            curdata.acc  = Eigen::Vector3d(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z);
            curdata.gyro = Eigen::Vector3d(data.angular_velocity.x,data.angular_velocity.y,data.angular_velocity.z);
            curdata.time = data.header.stamp.toSec();
        }

        _predictimu(lastdata,curdata);
        //用于可视化
        pcl::PointXYZRGB point;
        point.x = imustatus.pos.x();
        point.y = imustatus.pos.y();
        point.z = imustatus.pos.z();
        point.r = 255.0;
        point.g = 100.0;
        point.b = 0.0;
        
        cloud.points.push_back(point);
        // 建立ros点云
        sensor_msgs::PointCloud2 output_msg;
        output_msg.header.stamp = ros::Time::now();
        // 将pcl点云转化为ros消息发布
        pcl::toROSMsg(cloud, output_msg);
        // 发布的点云坐标系 rviz中需要对应着修改
        output_msg.header.frame_id = "map";
        _pub.publish(output_msg);
        lastdata  = curdata;
    }
    
    //display cur imu status
    void _handleimudata(const sensor_msgs::Imu& data)
    {
        static tf::TransformBroadcaster br;
        visualization_msgs::Marker maker;
        maker.header.frame_id = "map";
        maker.header.stamp = ros::Time::now();
        maker.ns = "basic_shape";
        maker.id = 0;

        maker.type = visualization_msgs::Marker::ARROW;
        maker.action = visualization_msgs::Marker::ADD;
        
        maker.pose.position.x = 0;
        maker.pose.position.y = 0;
        maker.pose.position.z = 0;
        maker.pose.orientation.x = data.orientation.x;
        maker.pose.orientation.y = data.orientation.y;
        maker.pose.orientation.z = data.orientation.z;
        maker.pose.orientation.w = data.orientation.w;

        maker.scale.x = 1.0;
        maker.scale.y = 0.1;
        maker.scale.z = 0.1;

        maker.color.r = 0;
        maker.color.g = 1;
        maker.color.b = 0;
        maker.color.a = 1;

        maker.lifetime = ros::Duration();
        ROS_INFO("push one maker info");
        _pub.publish(maker);

        tf::Transform trans;
        trans.setOrigin(tf::Vector3(0.0,0.0,0.0));
        tf::Quaternion qua(data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w);
        trans.setRotation(qua);
        br.sendTransform(tf::StampedTransform(trans,ros::Time::now(),"map","base_link"));
    }

    void _saveimupose(const Eigen::Vector3d& pos)
    {
        static std::ofstream ofile("/home/tlg/Document/Sources/rosprj/imupose.txt");
        ofile << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
    }

public:
 ImuStatus                         imustatus;
protected:
    ros::Publisher  _pub;
    ros::Subscriber _sub;
    ros::NodeHandle _hdl;

    // 建立 pcl 点云
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

   
};
#define D2R 0.0174532925388889
#define R2D 57.29577945088757

// euler2Rotation:   body frame to interitail frame
Eigen::Quaterniond euler2Rotation( Eigen::Vector3d  eulerAngles)
{
    double roll = D2R * eulerAngles(0);
    double pitch = D2R * eulerAngles(1);
    double yaw = D2R * eulerAngles(2);
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Eigen::Quaterniond q;
    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;

    return q;
}

Eigen::Vector3d ToEulerAngles(const Eigen::Quaterniond& q) {
    Eigen::Vector3d angles;    //roll pitch yaw
    const auto x = q.x();
    const auto y = q.y();
    const auto z = q.z();
    const auto w = q.w();

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles[2] = std::atan2(siny_cosp, cosy_cosp);


    return angles;
}

using ImuSimVector = std::vector<ImuHandler::ImuData>;
void LoadLvxData(const std::string& str, ImuSimVector& imudatas)
{
    std::ifstream file(str);
    while(!file.eof())
    {
        std::string line;
        std::getline(file,line);
        double time;
        double quaw,quax,quay,quaz;
        double roll,pitch,yaw;
        double posx,posy,posz;
        double gyrx,gyry,gyrz;
        double accx,accy,accz;
        double lon,lat,alt;

        sscanf(line.c_str(),"%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf",&time, 
                                                          &roll, &pitch, &yaw,
                                                          &accx,&accy,&accz,
                                                          &gyrx,&gyry,&gyrz,
                                                          &lon,&lat,&alt);

        ImuHandler::ImuData dataitem;
        dataitem.time = time;
        
        dataitem.pos  = Eigen::Vector3d(Functions::mector_lon(lon), Functions::mector_lon(lat), alt);
        dataitem.acc  = Eigen::Vector3d(accx, accy, accz);
        dataitem.gyro = Eigen::Vector3d(gyrx * D2R, gyry * D2R, gyrz * D2R);// 度->弧度<四元数对应的旋转轴的弧度>
        dataitem.rpy  = Eigen::Vector3d(roll,pitch,yaw);
        
        imudatas.push_back(dataitem);
    }
}

visualization_msgs::Marker RvizDrawMaker(const Eigen::Quaterniond& qua,const Eigen::Vector3d& pos)
{
    static tf::TransformBroadcaster br;
    visualization_msgs::Marker maker;
    maker.header.frame_id = "map";
    maker.header.stamp = ros::Time::now();
    maker.ns = "basic_shape";
    maker.id = 0;

    maker.type = visualization_msgs::Marker::ARROW;
    maker.action = visualization_msgs::Marker::ADD;
    
    maker.pose.position.x = pos.x();
    maker.pose.position.y = pos.y();
    maker.pose.position.z = pos.z();
    maker.pose.orientation.x = qua.x();
    maker.pose.orientation.y = qua.y();
    maker.pose.orientation.z = qua.z();
    maker.pose.orientation.w = qua.w();

    maker.scale.x = 2.0;
    maker.scale.y = 0.1;
    maker.scale.z = 0.1;

    maker.color.r = 0;
    maker.color.g = 1;
    maker.color.b = 0;
    maker.color.a = 1;

    maker.lifetime = ros::Duration();


    tf::Transform trans;
    trans.setOrigin(tf::Vector3(pos.x(),pos.y(),pos.z()));
    tf::Quaternion qa(qua.x(),qua.y(),qua.z(),qua.w());
    trans.setRotation(qa);
    br.sendTransform(tf::StampedTransform(trans,ros::Time::now(),"map","base_link"));
    return std::move(maker);
}

void drawPointAndAxis(const Eigen::Quaterniond& qua, double x, double y, double z, ros::Publisher &pub)
{
    static pcl::PointCloud<pcl::PointXYZRGB> tempcloud;
    static int index = 0;
    static pcl::PointXYZRGB originpt;
    if(index++ == 0)
    {
        originpt.x = x;
        originpt.y = y;
        originpt.z = z;
        originpt.r = 255.0;
        originpt.g = 0.0;
        originpt.b = 0.0;
    }
    pcl::PointXYZRGB point;
    point.x = x - originpt.x;
    point.y = y - originpt.y;
    point.z = z - originpt.z;

    point.r = 0.0;
    point.g = 255.0;
    point.b = 0.0;
    tempcloud.points.push_back(point);

    sensor_msgs::PointCloud2 output_msg;
    output_msg.header.stamp = ros::Time::now();
     // 将pcl点云转化为ros消息发布
    pcl::toROSMsg(tempcloud, output_msg);
      // 发布的点云坐标系 rviz中需要对应着修改
    output_msg.header.frame_id = "map";
    pub.publish(output_msg);

    RvizDrawMaker(qua,Eigen::Vector3d(point.x,point.y,point.z));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_handle_node");
    ros::NodeHandle node;
    
    ImuHandler handler;
    
    int irate = 20;
    if(argc > 1) irate = atoi(argv[1]);
    ROS_INFO(("Rate : " + std::to_string(irate)).c_str());
    ros::Rate rate(irate);
    
    ImuSimVector  imudatas;

    //加载imu模拟数据
    LoadLvxData("/home/tlg/Document/Datas/L0999.txt",imudatas);
    
    auto _gps = node.advertise<sensor_msgs::PointCloud2>("gpstrace",10);

    int index = 0;

    while(node.ok())
    {
       if(index < imudatas.size())
       {
           sensor_msgs::Imu tempitem;
           tempitem.header.stamp =  ros::Time(imudatas[index].time);
           tempitem.linear_acceleration.x = imudatas[index].acc.x();
           tempitem.linear_acceleration.y = imudatas[index].acc.y();
           tempitem.linear_acceleration.z = imudatas[index].acc.z();
           
           tempitem.angular_velocity.x    = imudatas[index].gyro.x();
           tempitem.angular_velocity.y    = imudatas[index].gyro.y();
           tempitem.angular_velocity.z    = imudatas[index].gyro.z();
          
           auto temp =  /*Eigen::Quaterniond*/(euler2Rotation(imudatas[index].rpy));
           tempitem.orientation.w = temp.w();
           tempitem.orientation.x = temp.x();
           tempitem.orientation.y = temp.y();
           tempitem.orientation.z = temp.z();
           handler._calimupose(tempitem);

           drawPointAndAxis(temp, imudatas[index].pos.y(), imudatas[index].pos.x(), imudatas[index].pos.z(), _gps);
            ++index;
       }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}