#include <ros/ros.h>

#include <autoware_msgs/LaneArray.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;
using namespace ros;

int LANE_NUM = 0;
int PointAhead = 50;

class WaypointTracker{
    public:
    WaypointTracker(ros::NodeHandle& n);
    ~WaypointTracker();

    ros::NodeHandle nh;

    void WaypointCallback(const autoware_msgs::LaneArray& msg);
    void IMUCallback(const sensor_msgs::Imu& msg);
    void CurrentPointCallback(const nav_msgs::Odometry& msg);

    int MinPointCalculator(double current_x, double current_y);
    void Waypoint2Bodyframe(double current_x_, double current_y_, double current_yaw_, int goal_idx);

    ros::Subscriber waypoint_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber current_point_sub;
    ros::Publisher waypoint_body_pub;

    autoware_msgs::LaneArray waypoint_array_global;
    nav_msgs::Path waypoint_array_body;

    double current_x, current_y, current_yaw;
    geometry_msgs::Quaternion imu_quat;
    int current_point_idx, farest_point_idx;

    bool is_waypoint_subscribed;
};

WaypointTracker::WaypointTracker(ros::NodeHandle& n){
    nh = n;
    ROS_INFO("WaypointTracker is created.");

    is_waypoint_subscribed = false;

    waypoint_sub = nh.subscribe("/waypoint_lane", 10, &WaypointTracker::WaypointCallback, this);
    imu_sub = nh.subscribe("/carmaker/imu/data", 10, &WaypointTracker::IMUCallback, this);
    current_point_sub = nh.subscribe("/EKF/utm_local", 10, &WaypointTracker::CurrentPointCallback, this);

    waypoint_body_pub = nh.advertise<nav_msgs::Path>("/body_frame/waypoints", 1000);
}
WaypointTracker::~WaypointTracker(){
    ROS_INFO("WaypointTracker is destructed.");
}

void WaypointTracker::WaypointCallback(const autoware_msgs::LaneArray& msg){
    if (is_waypoint_subscribed == false){
        waypoint_array_global = msg;
        is_waypoint_subscribed = true;
    }
}

void WaypointTracker::IMUCallback(const sensor_msgs::Imu& msg){
    imu_quat = msg.orientation;
}

void WaypointTracker::CurrentPointCallback(const nav_msgs::Odometry& msg){
    if (is_waypoint_subscribed == false){
        return;
    }
    current_x = msg.pose.pose.position.x;
    current_y = msg.pose.pose.position.y;
    tf::Quaternion qaut_current_orientation(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(qaut_current_orientation);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw);

    current_point_idx = MinPointCalculator(current_x, current_y);
    farest_point_idx = current_point_idx + PointAhead;

    for (int point_idx = current_point_idx +1; point_idx <= farest_point_idx; ++point_idx){
        Waypoint2Bodyframe(current_x, current_y, current_yaw, point_idx);
    }

    waypoint_body_pub.publish(waypoint_array_body);
    
}

int WaypointTracker::MinPointCalculator(double current_x_, double current_y_){

    int lane_size = sizeof(waypoint_array_global.lanes[LANE_NUM]);

    double lane_x;
    double lane_y;
    double current_distance;
    double min_distance;
    int min_distance_point;

    for (int waypoint_num = 0; waypoint_num < lane_size; ++waypoint_num){
        lane_x = waypoint_array_global.lanes[LANE_NUM].waypoints[waypoint_num].pose.pose.position.x;
        lane_y = waypoint_array_global.lanes[LANE_NUM].waypoints[waypoint_num].pose.pose.position.y;
        current_distance = sqrt(pow((current_x_ - lane_x) ,2) + pow((current_y_ - lane_y),2));   //calculate distance from current point to a point in waypoint array.
        if (waypoint_num == 0 || current_distance < min_distance){                              //compare distance and update the minimum distance waypoint
            min_distance = current_distance;
            min_distance_point = waypoint_num;  //save the minimum distance point from current state.
        }
    }

    return min_distance_point;
}

void WaypointTracker::Waypoint2Bodyframe(double current_x_, double current_y_, double current_yaw_, int goal_idx){
    double x_diff_global, y_diff_global;
    geometry_msgs::PoseStamped PoseStamped_buf;

    x_diff_global = waypoint_array_global.lanes[LANE_NUM].waypoints[goal_idx].pose.pose.position.x - current_x_;
    y_diff_global = waypoint_array_global.lanes[LANE_NUM].waypoints[goal_idx].pose.pose.position.y - current_y_;
    //difference from current position to destination in body frame.
    PoseStamped_buf.pose.position.x = cos(current_yaw_)*x_diff_global + sin(current_yaw_)*y_diff_global;
    PoseStamped_buf.pose.position.y = -1*sin(current_yaw_)*x_diff_global + cos(current_yaw_)*y_diff_global;
    
    waypoint_array_body.poses.push_back(PoseStamped_buf);
}

// void WaypointTracker::Visualization(){

//     static tf::TransformBroadcaster br;
//     tf::Transform transform;
//     transform.setOrigin( tf::Vector3(current_x, current_y, 0) );
//     transform.setRotation(tf::Quaternion(imu_quat.x, imu_quat.y, imu_quat.z, imu_quat.w));
//     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

//     current_point.header.stamp = ros::Time::now();
//     current_point.header.frame_id = "odom";
//     current_point.child_frame_id = "base_link";
//     current_point.pose.pose.position.x = utm_local.x;
//     current_point.pose.pose.position.y = utm_local.y;
//     // current_point.pose.pose.orientation.z = yaw;
//     current_point_Visualization_pub.publish(current_point);

//     dest_point_coordinate.header.stamp = ros::Time::now();
//     dest_point_coordinate.header.frame_id = "odom";
//     dest_point_coordinate.child_frame_id = "base_link";
//     dest_point_coordinate.pose.pose.position.x = waypoint_lane.lanes[LANE_NUM].waypoints[dest_point].pose.pose.position.x;
//     dest_point_coordinate.pose.pose.position.y = waypoint_lane.lanes[LANE_NUM].waypoints[dest_point].pose.pose.position.y;
//     // dest_point_coordinate.pose.pose.orientation.z = yaw;            // 목적지 방향성 넣어줘야됨
//     dest_point_Visualization_pub.publish(dest_point_coordinate);
// }

int main (int argc, char** argv)
{
    ros::init (argc, argv, "WaypointTracker");
    ros::NodeHandle _nh;
    ros::Rate loop_rate(10);
    
    printf("Initiate: WaypointTracker\n");

    WaypointTracker waypoint_tracker(_nh);

    while (ros::ok()){
        loop_rate.sleep();
        ros::spinOnce();
    }

    printf("Terminate: WaypointTracker\n");

    return 0;
}