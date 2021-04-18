#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <autoware_msgs/LaneArray.h>

std::string way_point_file_path = "/home/usrg/catkin_ws/src/hyundai_2021/STUDY1/UTM.csv";

int LANE_NUM = 0;
bool read_realtime_waypoint = false;
int LOOK_AHEAD = 2;

using namespace std;
using namespace ros;

class WayPoint_Loader{
    public:
    WayPoint_Loader(ros::NodeHandle& n);
    ~WayPoint_Loader();
    ros::NodeHandle nh;

    void WaypointFileReader();
    void WayPointPublisher();

    ros::Publisher waypoint_lane_pub;

    autoware_msgs::LaneArray waypoint_lane;
    std::ifstream waypointFile;
    
    bool is_realtime = read_realtime_waypoint;
};

WayPoint_Loader::WayPoint_Loader(ros::NodeHandle& n){
    nh = n;
    ROS_INFO("WayPoint_Loader is created.");
    waypointFile.open(way_point_file_path); // select file to open

    waypoint_lane_pub = nh.advertise<autoware_msgs::LaneArray> ("/waypoint_lane", 1000);

    if (is_realtime == false){
        WayPoint_Loader::WaypointFileReader(); //read waypoint file. (if not want to load waypoint in realtime)
    }
}

WayPoint_Loader::~WayPoint_Loader(){
    ROS_INFO("WayPoint_Loader destructed.");
}

//Function to copy recored waypoints (in csv file) to Autoware_msgs::LaneArray type variable
void WayPoint_Loader::WaypointFileReader(){
    int i = 0;
    std::string line;

    autoware_msgs::Lane Lane_buf[LANE_NUM + 1];
    autoware_msgs::Waypoint Waypoint_buf;
    geometry_msgs::PoseStamped PoseStamped_buf;

    while (getline(waypointFile, line)){

        std::stringstream linestream(line);
        string waypoint_x;
        string waypoint_y;

        std::getline(linestream, waypoint_x, ',');
        std::getline(linestream, waypoint_y, '\n');

        // Waypoint_buf.pose.header.stamp = ros::Time::now();
        // Waypoint_buf.pose.header.frame_id = "odom";
        Waypoint_buf.pose.pose.position.x = std::stod(waypoint_x);
        Waypoint_buf.pose.pose.position.y = std::stod(waypoint_y);
        Waypoint_buf.pose.pose.orientation.w = 1;
        Lane_buf[LANE_NUM].waypoints.push_back(Waypoint_buf);

        ++i;
    }

    waypoint_lane.lanes.push_back(Lane_buf[LANE_NUM]);

    // for (int j = 0; j < i; ++j){
    //     std::cout << "pose.x" << j << ":" <<waypoint_lane.lanes[LANE_NUM].waypoints[j].pose.pose.position.x <<endl;
    //     std::cout << "pose.y" << j << ":" <<waypoint_lane.lanes[LANE_NUM].waypoints[j].pose.pose.position.y <<endl;
    // }

    waypoint_lane.lanes[LANE_NUM].waypoints;

}

//Function to publish waypoint messeges.
void WayPoint_Loader::WayPointPublisher(){
    waypoint_lane_pub.publish(waypoint_lane);
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "WAYPOINT_LOADER");
    ros::NodeHandle _nh;
    ros::Rate loop_rate(10);
    
    printf("Initiate: WAYPOINT_LOADER\n");

    // std::ofstream waypointFile ("/home/usrg/catkin_ws/src/STUDY1/UTM.csv");

    WayPoint_Loader waypoint_loader(_nh);

    while (ros::ok()){
        //if want to laod waypoint in realtime
        if (waypoint_loader.is_realtime == true){
            waypoint_loader.WaypointFileReader();
        }

        waypoint_loader.WayPointPublisher();

        loop_rate.sleep();
        ros::spinOnce();
    }

    waypoint_loader.waypointFile.close(); 
    printf("Terminate: WAY_POINT_LOADER\n");

    return 0;
}
