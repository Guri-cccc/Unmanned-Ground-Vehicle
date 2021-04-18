#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>

bool read_realtime_waypoint = false;
int LOOK_AHEAD = 50;

class WayPoint_Loader{
    public:
    WayPoint_Loader(ros::NodeHandle& n, std::ofstream file);
    ~WayPoint_Loader();
    ros::NodeHandle nh;

    void WaypointFileReader(std::ofstream file);
    void WayPointPublisher();

    ros::Publisher  waypoint_lane_pub;

    Autoware_msgs::LaneArray waypoint_lane;
    
    bool is_realtime = read_realtime_waypoint;
};

WayPoint_Loader::WayPoint_Loader(ros::NodeHandle& n, std::ofstream file){
    nh = n;
    ROS_INFO("WayPoint_Loader is created.");

    waypoint_lane_pub = nh.advertise<Autoware_msgs::LaneArray> ("/waypoint_lane", 1000);

    if (is_realtime == false){
        void WayPoint_Loader::WaypointFileReader(); //read waypoint file. (if not want to load waypoint in realtime)
    }
}

WayPoint_Loader::~WayPoint_Loader(file){
    ROS_INFO("WayPoint_Loader destructed.");
}

//Function to copy recored waypoints (in csv file) to Autoware_msgs::LaneArray type variable
void WayPoint_Loader::WaypointFileReader(std::ofstream file){
    int i = 0;

    while (getline(file, line);){
        std::stringstream linestream(line);

        string waypoint_x;
        string waypoint_y;

        std::getline(linestream, waypoint_x, ',');
        waypoint_lane[LANE_NUM].Waypoint[i].pose.pose.position.x = std::stod(waypoint_x);
        std::getline(linestream, waypoint_y, ',');
        waypoint_lane[LANE_NUM].Waypoint[i].pose.pose.position.y = std::stod(waypoint_y);

        ++i;
    }
}

//Function to publish waypoint messeges.
void WayPointPublisher(){
    waypoint_lane_pub.publish(waypoint_lane);
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "WAYPOINT_LOADER");
    ros::NodeHandle _nh;
    ros::Rate loop_rate(10);
    
    printf("Initiate: WAYPOINT_LOADER\n");

    std::ofstream waypointFile ("/home/usrg/catkin_ws/src/STUDY1/UTM.csv");

    WayPoint_Loader waypoint_loader(_nh, waypointFile);

    while (ros::ok()){
        //if want to laod waypoint in realtime
        if (waypoint_loader.is_realtime == true){
            waypoint_loader.WaypointFileReader(waypointFile);
        }

        waypoint_loader.WayPointPublisher();

        loop_rate.sleep();
        spinOnce();
    }

    waypointFile.close(); 
    printf("Terminate: WAY_POINT_LOADER\n");

    return 0;
}