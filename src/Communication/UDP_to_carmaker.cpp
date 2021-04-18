#define CARMAKER_ROS
// essential header for ROS-OpenCV operation
#include <ros/ros.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <arpa/inet.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>

#if defined(WIN32) && !defined(INTIME)
#  include <windows.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <math.h>

using namespace ros;
using namespace std;

#define DF_UDP_PORTNUM      60001
#define DF_UDP_SERVER_ADDR  "127.0.0.1" //"192.168.0.130" //"127.0.0.1" : LOCAL

#define PI 3.14159265359

#pragma pack (push, 1)
typedef struct tUDP_Input {
	struct {
		unsigned char MsgID; 
	} Header;
	
    struct {
		double SteeringWheel;   // rad
		double Ax; 				// [m/s^2]
		int GearNo; 			// 1: Driving, 0: Stop, -1: Back, -9: Parking
        char Light;             //0: Off, 1: Left indicator, 2: Right indicator, 3: Hazard
    } DriveCont;
} tUDP_Input;
#pragma pack(pop)

class SERVER_TO_SEND
{
    public:
        SERVER_TO_SEND(ros::NodeHandle& n);
        ~SERVER_TO_SEND();
        void JoyCommandCallback(const ackermann_msgs::AckermannDriveStamped& msg);
        void JoyReverseCommandCallback(const std_msgs::Bool& msg);
        
        ros::NodeHandle nh;
        ros::Subscriber JoyCommand_sub;
        ros::Subscriber JoyReverseCommand_sub;

        double steeringwheel;
        double ax;
        int reverse;


};

SERVER_TO_SEND::SERVER_TO_SEND(ros::NodeHandle& n)
{
    nh = n;
    JoyCommand_sub = nh.subscribe("/Ackermann/command/joy",10,&SERVER_TO_SEND::JoyCommandCallback, this);
    JoyReverseCommand_sub = nh.subscribe("/Joy/command/reverse",10,&SERVER_TO_SEND::JoyReverseCommandCallback, this);
    ROS_INFO("SERVER_TO_SEND is created");
}

SERVER_TO_SEND::~SERVER_TO_SEND()
{
    ROS_INFO("SERVER_TO_SEND destructor.");
} //여기는 왜 ; 없지? //**ROS_DEBUG와 ROS_INFO의 차이

void SERVER_TO_SEND::JoyCommandCallback(const ackermann_msgs::AckermannDriveStamped& msg)
{
    steeringwheel = msg.drive.steering_angle * (PI/180);
    ax = msg.drive.acceleration;
}

void SERVER_TO_SEND::JoyReverseCommandCallback(const std_msgs::Bool& msg)
{
    if (msg.data == true){
        reverse = -1;
    }else{
        reverse = 1;
    }
}

int main(int argc, char** argv)
{
    init (argc, argv, "Carmaker_UPD_TX");
    ros::NodeHandle _nh;

    printf("Initiate: Server_TX\n");
    ros::Rate loop_rate(25);

    int Socket;
    struct sockaddr_in ServerAddr;
    struct tUDP_Input TX_buff;      //여기에 보낼 데이터 저장

    Socket = socket(PF_INET, SOCK_DGRAM, 0);
    int enable = 1;
    setsockopt(Socket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)); 

    if(Socket == -1){
        printf("[ERROR] 'socket()'\n");
        return -1;
    }
    else{
        printf("[DONE] UDP socket is created\n");
    }

    memset(&ServerAddr, 0, sizeof(ServerAddr));
    ServerAddr.sin_family = PF_INET;
    ServerAddr.sin_port = htons(DF_UDP_PORTNUM);;
    ServerAddr.sin_addr.s_addr = inet_addr(DF_UDP_SERVER_ADDR);

    SERVER_TO_SEND _server_to_send(_nh);

    while(ros::ok()){
        TX_buff.Header.MsgID = 0xFF;                //**??
        TX_buff.DriveCont.SteeringWheel = _server_to_send.steeringwheel;
        TX_buff.DriveCont.Ax = _server_to_send.ax;
        TX_buff.DriveCont.GearNo = _server_to_send.reverse;
        TX_buff.DriveCont.Light = 0;

        sendto(Socket, (char*)&TX_buff, sizeof(TX_buff), 0, (struct sockaddr *)(&ServerAddr), sizeof(ServerAddr));

        loop_rate.sleep();

        spinOnce();
    }

    printf("Terminate: Server_TX\n");

    close(Socket);

    return 0;
}