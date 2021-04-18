#include <ros/ros.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

using namespace std;

float steer_max = 90;   //[degrees]
float acc_max = 20;     //[m/s^2]

class JOY_CONTROL{
    public:
    JOY_CONTROL(ros::NodeHandle& n);
    ~JOY_CONTROL();

    void JoyCallback(const sensor_msgs::Joy& msg);

    ros::NodeHandle nh;
    ros::Subscriber Joy_sub;
    ros::Publisher Joy_command_pub;
    ros::Publisher Joy_reverse_command_pub;

    ackermann_msgs::AckermannDriveStamped joy_controller;
    std_msgs::Bool Reverse;

    bool manual = false;
    bool move = true;
};

JOY_CONTROL::JOY_CONTROL(ros::NodeHandle& n){
    nh = n;
    Joy_sub = nh.subscribe("/joy", 10, &JOY_CONTROL::JoyCallback, this);
    Joy_command_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped> ("/Ackermann/command/joy", 1000);
    Joy_reverse_command_pub = nh.advertise<std_msgs::Bool> ("/Joy/command/reverse", 1000);

    ROS_INFO("JOY_CONTROL is created");
}

JOY_CONTROL::~JOY_CONTROL(){
    ROS_INFO("JOY_CONTROL destruced.");
}

void JOY_CONTROL::JoyCallback(const sensor_msgs::Joy& msg){

    if (msg.buttons[0] ==1){
        move = false;
    }

    if (msg.buttons[1] == 1){
        manual = true;
        move = true;
    }

    if (msg.buttons[5] == 1){
        Reverse.data = true;
    }else{
        Reverse.data = false;
    }

    if (manual == true){
        joy_controller.drive.steering_angle = msg.axes[3] * steer_max;
        joy_controller.drive.acceleration = msg.axes[4] * acc_max;
            if (move == true){
            Joy_command_pub.publish(joy_controller);
            Joy_reverse_command_pub.publish(Reverse);
            }
    }


}
int main(int argc, char** argv){
    ros::init(argc, argv, "Joy_controller");
    ros::NodeHandle _nh;

    printf("Initiate: Joy_controller\n");

    ros::Rate loop_rate(25);
    JOY_CONTROL joy_control(_nh);

    while (ros::ok()){
        loop_rate.sleep();
        ros::spinOnce();
    }
    printf("Terminate: Joy_controller\n");

    return 0;
}