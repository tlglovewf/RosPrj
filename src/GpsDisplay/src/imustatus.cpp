#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sstream>

class ImuHandler final
{
public:
    ImuHandler()
    {
        #if 0
        //for test pose
        _pub = _hdl.advertise<visualization_msgs::Marker>("imumaker",10);
        _sub = _hdl.subscribe("/imu_raw",10,&ImuHandler::_handleimudata,this);
        #else
        _pub = _hdl.advertise<sensor_msgs::PointCloud2>("imutrace",10);
        _sub = _hdl.subscribe("/imu_raw",10,&ImuHandler::_calimupose,this);
        #endif 
    }

protected:

    struct ImuStatus
    {
       Eigen::Vector3d      gyro_bias   = Eigen::Vector3d(0,0,0);
       Eigen::Vector3d      acc_bias    = Eigen::Vector3d(0,0,0);

       Eigen::Vector3d      pos         = Eigen::Vector3d(0,0,0);
       Eigen::Vector3d      speed       = Eigen::Vector3d(0,0,0);
       Eigen::Vector3d      grav        = Eigen::Vector3d(0,0,-9.81);
       Eigen::Quaterniond   qua         = Eigen::Quaterniond(1,0,0,0);

       Eigen::Quaterniond   pose        = Eigen::Quaterniond(1,0,0,0);
       //add cov 
    };

    struct ImuData
    {
        double          time;
        Eigen::Vector3d gyro;
        Eigen::Vector3d acc;
        
    };

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
        
        Eigen::Vector3d half_gyro = (predata.gyro + curdata.gyro) * 0.5 - imustatus.gyro_bias;  
        
        

        Eigen::Vector3d un_acc_0 = imustatus.pose * (predata.acc - imustatus.acc_bias) - imustatus.grav ;

        imustatus.qua = imustatus.qua * deltaQ(half_gyro * dt);

        Eigen::Vector3d un_acc_1 = imustatus.pose  * (curdata.acc - imustatus.acc_bias) - imustatus.grav;

        Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

        std::stringstream ss;
        ss << ">>>>>> " <<  un_acc.x() << " " << un_acc.y() << " " << un_acc.z();
        // ROS_INFO(ss.str().c_str());
       
        imustatus.pos = imustatus.pos + imustatus.speed * dt + 0.5 * dt * dt * un_acc;
        imustatus.speed = imustatus.speed + un_acc * dt;
        //imustatus.pos.z() = 0.0;
        // ss.clear();
        ss << "++++++ " << imustatus.pos.x() << " " << imustatus.pos.y() << " " << imustatus.pos.z();
        ROS_INFO(ss.str().c_str());
       
    }
    void _calimupose(const sensor_msgs::Imu& data)
    {
        static int index = 0;
        static ImuData lastdata;
        static ImuData curdata;
        imustatus.pose =  Eigen::Quaterniond(data.orientation.w,data.orientation.x,data.orientation.y,data.orientation.z);
        if(index++ < 1)
        {
            lastdata.acc  =  Eigen::Vector3d(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z);
            lastdata.gyro =  Eigen::Vector3d(data.angular_velocity.x,data.angular_velocity.y,data.angular_velocity.z);
            lastdata.time =  data.header.stamp.toSec();
            
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
protected:
    ros::Publisher  _pub;
    ros::Subscriber _sub;
    ros::NodeHandle _hdl;

    // 建立 pcl 点云
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    ImuStatus                         imustatus;
};


class PathHandler final
{
public:
    PathHandler(ros::NodeHandle& node, std::string frame_id = "map")
    {
        //initlization
        _pathpub = node.advertise<nav_msgs::Path>("pathline",1,true);
        _dirpub  = node.advertise<visualization_msgs::Marker>("pathlinearrow",1);
        _path.header.stamp = ros::Time::now();
        
        _path.header.frame_id = frame_id;

    }

    void pushpoint(const geometry_msgs::Quaternion &qua,double x, double y, double z = 0)
    {
        geometry_msgs::PoseStamped this_pose;
        this_pose.pose.position.x = x;
        this_pose.pose.position.y = y;
        this_pose.pose.position.z = z;
        this_pose.pose.orientation = qua;
        this_pose.header.stamp = ros::Time::now();
        this_pose.header.frame_id = _path.header.frame_id;

        _path.poses.push_back(this_pose);

        _pathpub.publish(_path);

        insertarrow(this_pose);
    }

protected:

    void insertarrow(const geometry_msgs::PoseStamped& pose)
    {
           visualization_msgs::Marker maker;
        maker.header.frame_id = "map";
        maker.header.stamp = ros::Time::now();
        maker.ns = "basic_shape";
        maker.id = 0;

        maker.type = visualization_msgs::Marker::ARROW;
        maker.action = visualization_msgs::Marker::ADD;
        
        maker.pose.position = pose.pose.position;
  
        maker.pose.orientation = pose.pose.orientation;

        maker.scale.x = 1.0;
        maker.scale.y = 0.1;
        maker.scale.z = 0.1;

        maker.color.r = 0;
        maker.color.g = 1;
        maker.color.b = 0;
        maker.color.a = 1;

        maker.lifetime = ros::Duration();
        ROS_INFO("push one maker info");
        _dirpub.publish(maker);
    }

protected:
    ros::Publisher _pathpub;
    ros::Publisher _dirpub;
    nav_msgs::Path _path;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_handle_node");
    ros::NodeHandle node;
    
    ImuHandler handler;

    ros::Rate rate(10);

    PathHandler path(node);

     while(node.ok())
     {
 
         //geometry_msgs::Quaternion qua = tf::createQuaternionMsgFromYaw(th);
 
         //path.pushpoint(qua,x,y);

         ros::spinOnce();
         rate.sleep();
     }

    return 0;
}