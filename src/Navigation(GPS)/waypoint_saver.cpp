#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>

#include <UTM.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#define THRESHOLD 2

using namespace std;
using namespace ros;
using namespace GeographicLib;

std::string waypoint_file_path = "/home/guri/Study/catkin_ws_st/src/hyundai_2021/study/UTM_2.csv";

class UTM_SAVER{
    public:
        UTM_SAVER(ros::NodeHandle& n);
        ~UTM_SAVER();

        void IMUCallback (const sensor_msgs::Imu& msg);
        void UTMEFKCallback (const nav_msgs::Odometry& msg);

        ros::NodeHandle nh;

        ros::Subscriber imu_sub;
        ros::Subscriber utm_ekf_sub;
        ros::Publisher visual_pub;

        bool first;
        double move;

        nav_msgs::Odometry utm_local;
        nav_msgs::Odometry utm_prev;

        double utm_local_x, utm_local_y, utm_prev_x, utm_prev_y;
        double roll, pitch, utm_local_yaw;

        geometry_msgs::Quaternion imu_quat;
        nav_msgs::Odometry utm_EKF_local_viz;
};

UTM_SAVER::UTM_SAVER(ros::NodeHandle& n){
    nh = n;
    ROS_INFO("UTM_SAVER is created.");

    first = true;

    imu_sub = nh.subscribe("/carmaker/imu/data", 10, &UTM_SAVER::IMUCallback, this);
    utm_ekf_sub = nh.subscribe("/EKF/utm_local", 10, &UTM_SAVER::UTMEFKCallback, this);
    visual_pub = nh.advertise<nav_msgs::Odometry> ("/UTM_EKF_viz", 1000);
}

UTM_SAVER::~UTM_SAVER(){
    ROS_INFO("UTM_SAVER destructor.");
}

void UTM_SAVER::IMUCallback(const sensor_msgs::Imu& msg){
    imu_quat = msg.orientation;
}

void UTM_SAVER::UTMEFKCallback(const nav_msgs::Odometry& msg){
    utm_local = msg;

    if (first == true){
        utm_prev = utm_local;
        first = false;
        return;
    }
    
    // tf::Quaternion q(utm_local.pose.pose.orientation.x, utm_local.pose.pose.orientation.y, utm_local.pose.pose.orientation.z, utm_local.pose.pose.orientation.w);
    // tf::Matrix3x3 m(q);
    // m.getRPY(roll, pitch, utm_local_yaw);

    utm_local_x = utm_local.pose.pose.position.x;
    utm_local_y = utm_local.pose.pose.position.y;
    utm_prev_x = utm_prev.pose.pose.position.x;
    utm_prev_y = utm_prev.pose.pose.position.y;

    move = sqrt(pow((utm_local_x - utm_prev_x), 2) + pow((utm_local_y - utm_prev_y), 2)); //이동량 계산

    if (move > THRESHOLD){
        utm_local_yaw = atan2(utm_local_x - utm_prev_x, utm_local_y - utm_prev_y);
    }

    //visualization TF ("odom" --> "base_link")
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(utm_local_x, utm_local_y, 0));
    geometry_msgs::Quaternion utm_local_yaw_qaut = tf::createQuaternionMsgFromRollPitchYaw(0,0,utm_local_yaw);
    // transform.setRotation(tf::Quaternion(utm_local_yaw_qaut.x, utm_local_yaw_qaut.y, utm_local_yaw_qaut.z, utm_local_yaw_qaut.w));
    transform.setRotation(tf::Quaternion(imu_quat.x, imu_quat.y, imu_quat.z, imu_quat.w));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

    //visualization marker
    utm_EKF_local_viz.header.stamp = ros::Time::now();
    utm_EKF_local_viz.header.frame_id = "odom";
    utm_EKF_local_viz.child_frame_id = "base_link";
    utm_EKF_local_viz.pose.pose.position.x = utm_local_x;
    utm_EKF_local_viz.pose.pose.position.y = utm_local_y;
    // utm_EKF_local_viz.pose.pose.orientation.z = utm_local_yaw;
    utm_EKF_local_viz.pose.pose.orientation = utm_local_yaw_qaut;
    // utm_EKF_local_viz.pose.pose.position.x = utm_local_x;
    // utm_EKF_local_viz.pose.pose.position.y = utm_local_y;
    // // utm_EKF_local_viz.pose.pose.orientation.z = utm_local_yaw;
    // utm_EKF_local_viz.pose.pose.orientation = utm_local_yaw_qaut;
    visual_pub.publish(utm_EKF_local_viz);   
}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "Waypoint_saver");
    ros::NodeHandle _nh;
    
    printf("Initiate: Waypoint_saver\n");

    UTM_SAVER utm_saver(_nh);
    ros::Rate loop_rate(10);
    

    std::ofstream UTMfile (waypoint_file_path);

    while (ros::ok()){
        if (utm_saver.move > THRESHOLD && utm_saver.first == false){
            UTMfile << utm_saver.utm_local_x << "," << utm_saver.utm_local_y << "," << utm_saver.utm_local_yaw <<std::endl;
            utm_saver.utm_prev = utm_saver.utm_local;
            std::cout << "move" << utm_saver.move << std::endl;
        }
        loop_rate.sleep();
        spinOnce();
    }

    UTMfile.close(); 
    printf("Terminate: Waypoint_saver\n");

    return 0;
}