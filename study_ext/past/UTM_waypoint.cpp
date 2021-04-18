#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>

#include <GPS_Out.h>
#include <UTM.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
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

class GPS_INFO_TO_UTM_FILE{
    public:
        GPS_INFO_TO_UTM_FILE(ros::NodeHandle& n);
        ~GPS_INFO_TO_UTM_FILE();

        void IMUCallback (const sensor_msgs::Imu& msg);
        void GPS_OutCallback(const hellocm_msgs::GPS_Out& msg);

        ros::NodeHandle nh;

        ros::Subscriber GPS_Out_sub;
        ros::Subscriber IMU_sub;
        ros::Publisher Visual_pub;

        bool start = false;
        double move;

        geometry_msgs::Pose2D utm_origin;
        geometry_msgs::Pose2D utm_global;
        geometry_msgs::Pose2D utm_local;
        geometry_msgs::Pose2D utm_prev;

        geometry_msgs::Quaternion imu_quat;
        nav_msgs::Odometry utm_local_raw_viz;
        double yaw;
};

GPS_INFO_TO_UTM_FILE::GPS_INFO_TO_UTM_FILE(ros::NodeHandle& n){
    nh = n;
    ROS_INFO("GPS_INFO_TO_UTM_FILE is created.");

    IMU_sub = nh.subscribe("/carmaker/imu/data", 10, &GPS_INFO_TO_UTM_FILE::IMUCallback, this);
    GPS_Out_sub = nh.subscribe("/gps_out", 10, &GPS_INFO_TO_UTM_FILE::GPS_OutCallback, this);
    Visual_pub = nh.advertise<nav_msgs::Odometry> ("/UTM_raw_viz", 1000);

    GeographicLib::UTMUPS::Forward(lat_, long_, zone, m_isInNorthernHemisphere, utm_origin.x, utm_origin.y);
}

GPS_INFO_TO_UTM_FILE::~GPS_INFO_TO_UTM_FILE(){
    ROS_INFO("GPS_INFO_TO_UTM_FILE destructor.");
}

void GPS_INFO_TO_UTM_FILE::IMUCallback(const sensor_msgs::Imu& msg){
    imu_quat = msg.orientation;
}

void GPS_INFO_TO_UTM_FILE::GPS_OutCallback(const hellocm_msgs::GPS_Out& msg){

    GeographicLib::UTMUPS::Forward(msg.latitude, msg.longitude, zone, m_isInNorthernHemisphere, utm_global.x, utm_global.y);

    utm_local.x = utm_global.x - utm_origin.x;
    utm_local.y = utm_global.y - utm_origin.y;

    move = sqrt(pow((utm_local.x - utm_prev.x), 2) + pow((utm_local.y - utm_prev.y), 2)); //이동량 계산

    if (start != true){
        start = true;
    }

    if (move > THRESHOLD){
        yaw = atan2(utm_local.x - utm_prev.x, utm_local.y - utm_prev.y);
    }

    //visualization
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(utm_local.x, utm_local.y, 0) );
    transform.setRotation(tf::Quaternion(imu_quat.x, imu_quat.y, imu_quat.z, imu_quat.w));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

    utm_local_raw_viz.header.stamp = ros::Time::now();
    utm_local_raw_viz.header.frame_id = "odom";
    utm_local_raw_viz.child_frame_id = "base_link";
    utm_local_raw_viz.pose.pose.position.x = utm_local.x;
    utm_local_raw_viz.pose.pose.position.y = utm_local.y;
    
    utm_local_raw_viz.pose.pose.orientation.z = yaw;
    Visual_pub.publish(utm_local_raw_viz);
    
}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "UTM_writer");
    ros::NodeHandle _nh;
    
    printf("Initiate: UTM_writer\n");

    GPS_INFO_TO_UTM_FILE gps_info_to_utm_file(_nh);
    ros::Rate loop_rate(10);
    

    std::ofstream UTMfile ("/home/usrg/catkin_ws/src/hyundai_2021/STUDY1/UTM.txt");

    while (ros::ok()){
        if (gps_info_to_utm_file.move > THRESHOLD && gps_info_to_utm_file.start == true){
            UTMfile << gps_info_to_utm_file.utm_local.x << "," << gps_info_to_utm_file.utm_local.y << "," <<gps_info_to_utm_file.move <<std::endl;
            gps_info_to_utm_file.utm_prev.x = gps_info_to_utm_file.utm_local.x;
            gps_info_to_utm_file.utm_prev.y = gps_info_to_utm_file.utm_local.y;
        }
        loop_rate.sleep();
        spinOnce();
    }

    UTMfile.close(); 
    printf("Terminate: UTM_writer\n");

    return 0;
}