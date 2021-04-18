#include <ros/ros.h>

#include <autoware_msgs/LaneArray.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace GeographicLib;

int LOOK_AHEAD = 2;
int LANE_NUM = 0;

//Subscribe GPS data and find the nearest waypoint for navigation.
class Waypoint_Naigator{
    public:
    Waypoint_Naigator(ros::NodeHandle n);
    ~Waypoint_Naigator();
    ros::NodeHandle nh;

    void WaypointCallback(const autoware_msgs::LaneArray& msg);
    void GPS_OutCallback(const hellocm_msgs::GPS_Out& msg);
    void IMUCallback(const sensor_msgs::Imu& msg);
    int Min_Point_Calculator(double current_x, double current_y);
    void Visualization();

    ros::Subscriber GPS_Out_sub;
    ros::Subscriber IMU_sub;
    ros::Subscriber Waypoint_LaneArray_sub;
    ros::Publisher current_point_Visualization_pub;
    ros::Publisher dest_point_Visualization_pub;

    autoware_msgs::LaneArray waypoint_lane;

    geometry_msgs::Pose2D utm_origin;
    geometry_msgs::Pose2D utm_global;
    geometry_msgs::Pose2D utm_local;

    nav_msgs::Path body_diff_from_waypoint;

    bool is_waypoint_loaded = false;
    bool is_body_diff_from_waypoint_first = true;

    geometry_msgs::Quaternion imu_quat;
    int current_point_on_lane, dest_point;
    double x_diff_global, y_diff_global;
    nav_msgs::Odometry dest_point_coordinate;
    nav_msgs::Odometry current_point;
    // double yaw;
};

Waypoint_Naigator::Waypoint_Naigator(ros::NodeHandle n){
    n = nh;
    ROS_INFO("Waypoint_Naigator is created");

    Waypoint_LaneArray_sub = nh.subscribe("/waypoint_lane", 10, &Waypoint_Naigator::WaypointCallback, this);
    GPS_Out_sub = nh.subscribe("/EKF/utm_local", 10, &Waypoint_Naigator::GPS_OutCallback, this);
    IMU_sub = nh.subscribe("/carmaker/imu/data", 10, &Waypoint_Naigator::IMUCallback, this);
    current_point_Visualization_pub = nh.advertise<nav_msgs::Odometry>("/current_point", 1000);
    dest_point_Visualization_pub = nh.advertise<nav_msgs::Odometry>("/destination_point", 1000);

    GeographicLib::UTMUPS::Forward(lat_, long_, zone, m_isInNorthernHemisphere, utm_origin.x, utm_origin.y);
}

Waypoint_Naigator::~Waypoint_Naigator(){
    ROS_INFO("Waypoint_Naigator destructed");
}

void Waypoint_Naigator::WaypointCallback (const autoware_msgs::LaneArray& msg){
    if (is_waypoint_loaded == false){
        waypoint_lane = msg;
        is_waypoint_loaded = true;
    }
}

void Waypoint_Naigator::GPS_OutCallback(const nav_msgs::Odometry& msg){

    geometry_msgs::PoseStamped PoseStamped_buf;
    
    if (is_waypoint_loaded == true){
        GeographicLib::UTMUPS::Forward(msg.latitude, msg.longitude, zone, m_isInNorthernHemisphere, utm_global.x, utm_global.y);
        utm_local.x = utm_global.x - utm_origin.x;
        utm_local.y = utm_global.y - utm_origin.y;
        // utm_local.theta = //?????????????????????;
        current_point_on_lane = Min_Point_Calculator(utm_local.x, utm_local.y);     //get id of closest point on waypoint
        dest_point = current_point_on_lane + LOOK_AHEAD;
        //difference from current position to destination in global fram
        x_diff_global = waypoint_lane.lanes[LANE_NUM].waypoints[dest_point].pose.pose.position.x - utm_local.x;
        y_diff_global = waypoint_lane.lanes[LANE_NUM].waypoints[dest_point].pose.pose.position.y - utm_local.y;
        //difference from current position to destination in body frame.
        PoseStamped_buf[0].pose.position.x = cos(utm_local.theta)*x_diff_global + sin(utm_local.theta)*y_diff_global;
        PoseStamped_buf[0].pose.position.y = -1*sin(utm_local.theta)*x_diff_global + cos(utm_local.theta)*y_diff_global;

        if (is_body_diff_from_waypoint_first != true){
            body_diff_from_waypoint.poses.pop_back();
        }else{
            is_body_diff_from_waypoint_first = false;
        }
        body_diff_from_waypoint.poses.push_back(PoseStamped_buf[0]);
        Destination_body_pub.publish(body_diff_from_waypoint);

        std::cout<<"body destination x: " << body_diff_from_waypoint.poses[0].pose.position.x << std::endl;
        std::cout<<"body destination y: " << body_diff_from_waypoint.poses[0].pose.position.y << std::endl;

        Visualization();
    }
}

void Waypoint_Naigator::IMUCallback(const sensor_msgs::Imu& msg){
    imu_quat = msg.orientation;
}

//Function that finds the closest point in waypoint lane from current position.
int Waypoint_Naigator::Min_Point_Calculator(double current_x, double current_y){

    int lane_size = sizeof(waypoint_lane.lanes[LANE_NUM]);

    double lane_x;
    double lane_y;
    double current_distance;
    double min_distance;
    int min_distance_point;

    for (int waypoint_num = 0; waypoint_num < lane_size; ++waypoint_num){
        lane_x = waypoint_lane.lanes[LANE_NUM].waypoints[waypoint_num].pose.pose.position.x;
        lane_y = waypoint_lane.lanes[LANE_NUM].waypoints[waypoint_num].pose.pose.position.y;
        current_distance = sqrt(pow((current_x - lane_x) ,2) + pow((current_y - lane_y),2));   //calculate distance from current point to a point in waypoint array.
        if (waypoint_num == 0 || current_distance < min_distance){                              //compare distance and update the minimum distance waypoint
            min_distance = current_distance;
            min_distance_point = waypoint_num;  //save the minimum distance point from current state.
        }
    }

    return min_distance_point;
}

void Waypoint_Naigator::Visualization(){

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(utm_local.x, utm_local.y, 0) );
    transform.setRotation(tf::Quaternion(imu_quat.x, imu_quat.y, imu_quat.z, imu_quat.w));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

    current_point.header.stamp = ros::Time::now();
    current_point.header.frame_id = "odom";
    current_point.child_frame_id = "base_link";
    current_point.pose.pose.position.x = utm_local.x;
    current_point.pose.pose.position.y = utm_local.y;
    // current_point.pose.pose.orientation.z = yaw;
    current_point_Visualization_pub.publish(current_point);

    dest_point_coordinate.header.stamp = ros::Time::now();
    dest_point_coordinate.header.frame_id = "odom";
    dest_point_coordinate.child_frame_id = "base_link";
    dest_point_coordinate.pose.pose.position.x = waypoint_lane.lanes[LANE_NUM].waypoints[dest_point].pose.pose.position.x;
    dest_point_coordinate.pose.pose.position.y = waypoint_lane.lanes[LANE_NUM].waypoints[dest_point].pose.pose.position.y;
    // dest_point_coordinate.pose.pose.orientation.z = yaw;            // 목적지 방향성 넣어줘야됨
    dest_point_Visualization_pub.publish(dest_point_coordinate);
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "Waypoint_Naigator");
    ros::NodeHandle _nh;
    ros::Rate loop_rate(10);
    
    printf("Initiate: WAY_POINT_LOADER\n");

    Waypoint_Naigator waypoint_navigator(_nh);
    
    while (ros::ok()){
        loop_rate.sleep();
        ros::spinOnce();
    }

    printf("Terminate: WAY_POINT_LOADER\n");

    return 0;
}
