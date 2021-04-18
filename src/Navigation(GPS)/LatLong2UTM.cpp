#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>

#include <GPS_Out.h>
#include <UTM.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#define THRESHOLD 4

using namespace std;
using namespace ros;
using namespace GeographicLib;

int zone;
double lat_ =  37.58;
double long_ = 126.89;

bool m_isInNorthernHemisphere = true;

class GPS2UTM{
    public:
        GPS2UTM(ros::NodeHandle& n);
        ~GPS2UTM();

        void IMUCallback (const sensor_msgs::Imu& msg);
        void GPS_OutCallback(const hellocm_msgs::GPS_Out& msg);

        ros::NodeHandle nh;

        ros::Subscriber GPS_Out_sub;
        ros::Subscriber IMU_sub;
        ros::Publisher Visual_pub;
        ros::Publisher UTM_raw_pub;

        geometry_msgs::Pose2D utm_origin;
        geometry_msgs::Pose2D utm_global;
        geometry_msgs::Pose2D utm_local;

        geometry_msgs::Quaternion imu_quat;
        
        nav_msgs::Odometry utm_local_raw_viz;
};

GPS2UTM::GPS2UTM(ros::NodeHandle& n){
    nh = n;
    ROS_INFO("GPS2UTM is created.");

    IMU_sub = nh.subscribe("/carmaker/imu/data", 10, &GPS2UTM::IMUCallback, this);
    GPS_Out_sub = nh.subscribe("/gps_out", 10, &GPS2UTM::GPS_OutCallback, this);

    UTM_raw_pub = nh.advertise<geometry_msgs::Pose2D>("/UTM_raw", 1000);
    Visual_pub = nh.advertise<nav_msgs::Odometry> ("/UTM_raw_viz", 1000);

    //Get UTM info of origin point (for converting to local UTM)
    GeographicLib::UTMUPS::Forward(lat_, long_, zone, m_isInNorthernHemisphere, utm_origin.x, utm_origin.y);
}

GPS2UTM::~GPS2UTM(){
    ROS_INFO("GPS2UTM destructor.");
}

void GPS2UTM::IMUCallback(const sensor_msgs::Imu& msg){
    imu_quat = msg.orientation;
}

void GPS2UTM::GPS_OutCallback(const hellocm_msgs::GPS_Out& msg){

    GeographicLib::UTMUPS::Forward(msg.latitude, msg.longitude, zone, m_isInNorthernHemisphere, utm_global.x, utm_global.y);

    tf::Quaternion q(imu_quat.x, imu_quat.y, imu_quat.z, imu_quat.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    utm_local.x = utm_global.x - utm_origin.x;
    utm_local.y = utm_global.y - utm_origin.y;
    utm_local.theta = yaw;

    UTM_raw_pub.publish(utm_local);

    //visualization
    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // transform.setOrigin( tf::Vector3(utm_local.x, utm_local.y, 0) );
    // transform.setRotation(tf::Quaternion(imu_quat.x, imu_quat.y, imu_quat.z, imu_quat.w));
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

    utm_local_raw_viz.header.stamp = ros::Time::now();
    utm_local_raw_viz.header.frame_id = "odom";
    utm_local_raw_viz.child_frame_id = "base_link";
    utm_local_raw_viz.pose.pose.position.x = utm_local.x;
    utm_local_raw_viz.pose.pose.position.y = utm_local.y;

    utm_local_raw_viz.pose.pose.orientation = imu_quat; //*이게 맞나?
    Visual_pub.publish(utm_local_raw_viz);
}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "GPS2UTM");
    ros::NodeHandle _nh;
    
    printf("Initiate: GPS2UTM\n");

    GPS2UTM gps2utm(_nh);
    ros::Rate loop_rate(10);

    while (ros::ok()){
        loop_rate.sleep();
        spinOnce();
    }

    printf("Terminate: UTM_writer\n");

    return 0;
}