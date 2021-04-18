#include <ros/ros.h>
#include <math.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>

using namespace std;
using namespace ros;

double range = 1.5; //[m]

class SetRoute {
    public:
    SetRoute(ros::NodeHandle& n);
    ~SetRoute();

    ros::NodeHandle nh;

    void PathPointCallback(const geometry_msgs::PoseStamped& msg);
    void CurrentModeCallback(const std_msgs::Int32& msg);
    void CurrentPoseCallback(const nav_msgs::Odometry& msg);
    void StopFlagCallback(const std_msgs::Bool& msg);           //Stop and initialize everything

    void RoutePublisher(int current_route_idx);
    void Initializer();

    ros::Subscriber path_point_sub;
    ros::Subscriber current_mode_sub;
    ros::Subscriber current_pose_sub;
    // ros::Subscriber stop_flag_sub;                              //Stop and initialize everything

    ros::Publisher route_initialpose_pub;
    ros::Publisher route_goal_pub;

    nav_msgs::Odometry current_pose;
    geometry_msgs::PoseWithCovarianceStamped route_initialpose_msg;         //initial point of each splited route
    geometry_msgs::PoseStamped route_goal_msg;                              //goal point of each splited route
    geometry_msgs::PoseArray route_array;                                //goal point of each splited route
    
    double dist_from_goal = 9999;

    int total_route_number = 0;     //Same as the number of routes (goals)
    int current_route_index = 0;    //the index of current route

    bool auto_move = false;
    bool start_from_current_pose = false;
};

SetRoute::SetRoute(ros::NodeHandle& n){
    nh = n;
    path_point_sub = nh.subscribe("/path", 10, &SetRoute::PathPointCallback, this);//루트 들어오는 메세지 받기
    current_mode_sub = nh.subscribe("/Int/OperationMode", 10, &SetRoute::CurrentModeCallback, this);
    current_pose_sub = nh.subscribe("/lego_odom_ndt",10,&SetRoute::CurrentPoseCallback, this);
    // stop_flag_sub = nh.subscribe("", 10, &SetRoute::StopFlagCallback, this); //중도에 루트 다 취소하고 다시 다 설정하는 경우 메세지 하나 파기

    route_initialpose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/initialpose", 1000);
    route_goal_pub = nh.advertise<geometry_msgs::PoseStamped> ("/move_base_simple/goal", 1000);

    ROS_INFO("SetRoute is created");
}
SetRoute::~SetRoute(){
    ROS_INFO("SetRoute is destruced");
}

//subscribe goals
void SetRoute::PathPointCallback(const geometry_msgs::PoseStamped& msg){
    route_array.push_back(msg.pose);
    ++total_route_number;   //한번 목적지가 들어올 때 마다 루트 숫자 더해주기 
}

void SetRoute::CurrentModeCallback(const std_msgs::Int32& msg){
    if (msg == 2){      // Visual Servoing mode (initial pose of the next path is current position)
        start_from_current_pose = true;
    }else if (msg == 3){    //Auto mode
        auto_move = true;
    }
}

void SetRoute::CurrentPoseCallback(const nav_msgs::Odometry& msg){
    current_pose = msg;
    dist_from_goal = sqrt(pow((msg.pose.pose.position.x - route_goal_msg.pose.position.x),2)+pow((msg.pose.pose.position.y - route_goal_msg.pose.position.y),2));
    if (dist_from_goal < range){
        std::cout << "Reached Destination" << current_route_index << std::endl;
        ++current_route_index;
        if (current_route_index == total_route_number){
            std::cout << "TOTAL PATH END" << std::endl;
            Initializer();
        }else{
            RoutePublisher(current_route_index);
            start_from_current_pose = false;        //초기화
        }
    }
}
//Stop and initialize routes for resetting
// void SetRoute::StopFlagCallback(){
//     Initializer();
//     ROS_INFO("Initialized Routes")
// }

//Publish new route by given route number
void SetRoute::RoutePublisher(int current_route_idx){
    if (current_route_idx == 1 || start_from_current_pose == true){
        route_initialpose_msg.pose = current_pose.pose.pose;
    }else{
        route_initialpose_msg.pose = route_array.pose[current_route_idx - 1];    //previous goal is new start point
    }
    route_goal_msg.pose = route_array.pose[current_route_idx];                  //set new goal point
    route_initialpose_pub.publish(route_initialpose_msg);
    route_goal_pub.publish(route_goal_msg);
}
//Initialize variables
void SetRoute::Initializer(){
    route_array.clear();
    dist_from_goal = 9999;
    total_route_number = 0;
    current_route_index = 0;
    auto_move = false;
    start_from_current_pose = false;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "SetRoute");
    ros::NodeHandle _nh;

    printf("Initiate: SetRoute\n");

    ros::Rate loop_rate(25);
    SetRoute setRoute(_nh);

    while (ros::ok()){
        loop_rate.sleep();
        ros::spinOnce();
    }
    printf("Terminate: SetRoute\n");

    return 0;
}