#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <autoware_msgs/LaneArray.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

std::string way_point_file_path = "/home/guri/Study/catkin_ws_st/src/hyundai_2021/study/UTM_2.csv";

int LANE_NUM = 0;

using namespace std;
using namespace ros;

class WaypointLoader{
    public:
    WaypointLoader(ros::NodeHandle& n);
    ~WaypointLoader();
    ros::NodeHandle nh;

    void WaypointFileReader();
    void WaypointPublisher();

    ros::Publisher waypoint_lane_pub;

    std::ifstream waypointFile;

    autoware_msgs::LaneArray waypoint_lane;

    bool finished_reading_waypoints;
};

WaypointLoader::WaypointLoader(ros::NodeHandle& n){
    nh = n;
    ROS_INFO("WaypointLoader is created.");
    waypointFile.open(way_point_file_path); // select file to open

    bool finished_reading_waypoints = false;

    waypoint_lane_pub = nh.advertise<autoware_msgs::LaneArray> ("/waypoint_lane", 1000);

    WaypointLoader::WaypointFileReader();
}

WaypointLoader::~WaypointLoader(){
    ROS_INFO("WaypointLoader destructed.");
}

//Function to copy recored waypoints (in csv file) to Autoware_msgs::LaneArray type variable
void WaypointLoader::WaypointFileReader(){
    int i = 0;
    std::string line;

    autoware_msgs::Lane lane_buf[LANE_NUM + 1]; //[LANE_NUM + 1]는 크기 정해준거
    autoware_msgs::Waypoint waypoint_buf;
    geometry_msgs::PoseStamped PoseStamped_buf;
    
    while (getline(waypointFile, line)){

        std::stringstream linestream(line);
        string waypoint_x;
        string waypoint_y;
        string waypoint_yaw;

        std::getline(linestream, waypoint_x, ',');  //get x
        std::getline(linestream, waypoint_y, ',');  //get x
        std::getline(linestream, waypoint_yaw, '\n'); //get yaw
        //convert yaw (Euler ---> quaternion)
        tf2::Quaternion waypoint_orientation_qt;
        waypoint_orientation_qt.setRPY(0, 0, std::stod(waypoint_yaw));
        waypoint_orientation_qt = waypoint_orientation_qt.normalize();

        waypoint_buf.pose.pose.position.x = std::stod(waypoint_x);
        waypoint_buf.pose.pose.position.y = std::stod(waypoint_y);
        waypoint_buf.pose.pose.orientation = tf2::toMsg(waypoint_orientation_qt);

        lane_buf[LANE_NUM].waypoints.push_back(waypoint_buf);
    }
    waypoint_lane.lanes.push_back(lane_buf[LANE_NUM]);

    finished_reading_waypoints = true;
    std::cout << "finished_reading_waypoints: " << finished_reading_waypoints <<std::endl;

}

//Function to publish waypoint messeges.
void WaypointLoader::WaypointPublisher(){
    if (finished_reading_waypoints == true){
        waypoint_lane_pub.publish(waypoint_lane);
        std::cout << "Published" <<std::endl;
    }
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "WaypointLoader");
    ros::NodeHandle _nh;
    ros::Rate loop_rate(10);
    
    printf("Initiate: WaypointLoader\n");

    WaypointLoader waypoint_loader(_nh);

    while (ros::ok()){
        waypoint_loader.WaypointPublisher();

        loop_rate.sleep();
        ros::spinOnce();
    }

    waypoint_loader.waypointFile.close();
    printf("Terminate: WaypointLoader\n");

    return 0;
}
