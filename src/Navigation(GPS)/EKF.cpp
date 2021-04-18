#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <iostream>
#include <iterator>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

// using namespace std;
using namespace ros;
using namespace Eigen;

typedef Matrix<double, 5, 5> Matrix5d;
typedef Matrix<double, 5, 1> Vector5d;
typedef Matrix<double, 3, 5> Matrix35d;
typedef Matrix<double, 5, 3> Matrix53d;

class EKF {
    public:
    EKF(ros::NodeHandle& n);
    ~EKF();

    ros::NodeHandle nh;

    void RawUTMCallback(const geometry_msgs::Pose2D& msg);
    void VelocityCallback(const nav_msgs::Odometry& msg);
    void IMUCallback(const sensor_msgs::Imu& msg);
    void Estimation();
    void KalmanGain();
    void Calculation();
    void Covariance();
    void PipLine();
    void Visualization();

    ros::Subscriber raw_UTM_sub;
    ros::Subscriber velocity_sub;
    ros::Subscriber imu_sub;

    ros::Publisher EKF_UTM_local_pub;
    ros::Publisher visual_pub;

    double current_time;
    double prev_time;
    double dt;

    double theta;

    bool is_utm_raw_updated = false;
    bool is_first_UTM = true;
    bool is_first_Vel = true;
    
    Vector3d X_measured_k;
    Vector5d X_estimated_k;
    Vector5d X_calculated_k;
    Vector5d U_measured_k = Vector5d::Zero();

    Matrix5d A_dot = Matrix5d::Zero();
    Matrix5d A = Matrix5d::Zero();
    Matrix53d K = Matrix53d::Zero();
    Matrix5d P = Matrix5d::Zero();
    Matrix5d Q = Matrix5d::Zero();

    Matrix35d H = Matrix35d::Zero(); // system error
    Matrix3d R = Matrix3d::Zero(); //system error
    Matrix5d I5 = Matrix5d::Identity();

    nav_msgs::Odometry EKF_UTM_local;

    geometry_msgs::Quaternion imu_quat;
    nav_msgs::Odometry utm_EKF_local_viz;

};



EKF::EKF(ros::NodeHandle& n): prev_time(0.0)
{ 
    nh = n;
    ROS_INFO("EKF is created.");
    raw_UTM_sub = nh.subscribe("/UTM_raw", 10, &EKF::RawUTMCallback, this);
    velocity_sub = nh.subscribe("/Odometry/system", 10 , &EKF::VelocityCallback, this);
    imu_sub = nh.subscribe("/carmaker/imu/data", 10, &EKF::IMUCallback, this);

    EKF_UTM_local_pub = nh.advertise<nav_msgs::Odometry>("/EKF/utm_local", 1000);
    visual_pub = nh.advertise<nav_msgs::Odometry> ("/UTM_EKF_viz", 1000);


    H(0,0) = 1.0;
    H(1,1) = 1.0;
    H(2,2) = 1.0;

    Q(0, 0) = 0.01;
    Q(1, 1) = 0.01;
    Q(2, 2) = 0.0341;

    R(0, 0) = 5.0;
    R(1, 1) = 5.0;
    R(2, 2) = 0.1;

}

EKF::~EKF(){
    ROS_INFO("EKF destructed.");
}

void EKF::IMUCallback(const sensor_msgs::Imu& msg){
    imu_quat = msg.orientation;
}
//X_measured_k
void EKF::RawUTMCallback(const geometry_msgs::Pose2D& msg){
    double received_theta = msg.theta;
    if (std::isnan(received_theta)){
        received_theta = 0;
    }
    if (is_first_UTM == true){
        X_calculated_k[0] = msg.x;
        X_calculated_k[1] = msg.y;
        X_calculated_k[2] = received_theta;
        X_estimated_k[0] = msg.x;
        X_estimated_k[1] = msg.y;
        X_estimated_k[2] = received_theta;

        is_first_UTM = false;
        // std::cout << "is_first_UTM" <<std::endl;

        return;
    }
    // std::cout << "!!utm_raw_updated!!" <<std::endl;
    //Update X_measured_k
    X_measured_k[0] = msg.x;
    X_measured_k[1] = msg.y;
    X_measured_k[2] = received_theta;

    is_utm_raw_updated = true;
}
//X_estimated_k
void EKF::VelocityCallback(const nav_msgs::Odometry& msg){
    if (is_first_Vel == true){
        X_calculated_k[3] = msg.twist.twist.linear.x;
        X_calculated_k[4] = msg.twist.twist.angular.z;

        is_first_Vel = false;
        // std::cout << "is_first_Vel" <<std::endl;
    }
    //Define dt using prev. time and current time    
    if(prev_time == 0.0){
        prev_time = msg.header.stamp.toSec();
        // std::cout << "prev_time = 0.0" <<std::endl;
        return;
    }
    // std::cout << "!!vel_updated!!" <<std::endl;
    current_time = msg.header.stamp.toSec();
    dt = current_time - prev_time;
    
    std::cout << "dt" << dt <<std::endl;

    //Update U_measured_k 
    U_measured_k[3] = msg.twist.twist.linear.x;
    U_measured_k[4] = msg.twist.twist.angular.z;

    //Update states
    PipLine();

    //Save time
    prev_time = current_time;
}

//X_estimated_k
void EKF::Estimation(){

    theta = X_estimated_k[2];

    A_dot(0,3) = cos(theta);
    A_dot(1,3) = sin(theta);
    A_dot(2,4) = 1;
    I5(3,3) =0;
    I5(4,4) =0;
    A = A_dot*dt + I5;

    //Estimate X_estimated_k
    X_estimated_k = A * X_calculated_k + U_measured_k;

    //Estimate P
    P = A * P * A.transpose() + Q;
    std::cout << "***********" <<std::endl;
    std::cout << "Control Input" <<std::endl;
    std::cout << U_measured_k << std::endl;
    std::cout << "Estimation" <<std::endl;
    std::cout << X_estimated_k << std::endl;
}
//K
void EKF::KalmanGain(){
    //Calculate K
    if ((H * P * H.transpose() + R).determinant() == 0){
        // std::cout << "WARN: o determinant" <<std::endl;
    }else {
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        // std::cout << "KalmanGain" <<std::endl;
    }

}
//X_calculated_k
void EKF::Calculation(){
    //Calculate X_calculated_k (State)
    X_calculated_k = X_estimated_k + K * (X_measured_k - H * X_estimated_k);
    // std::cout << "Calculation" <<std::endl;

}
//P
void EKF::Covariance(){
    P = P - K * H * P;
    // std::cout << "Covariance" <<std::endl;
    // std::cout << "***********" <<std::endl;
    // std::cout << std::endl;
    
}

void EKF::PipLine(){
    // Update K, X_calculated_k only when raw UTM is updated
    if (is_utm_raw_updated == false){
        Estimation();

        tf2::Quaternion Orientation_qaut_e;
        Orientation_qaut_e.setRPY(0, 0, X_estimated_k[2]);
        Orientation_qaut_e = Orientation_qaut_e.normalize();
        EKF_UTM_local.pose.pose.orientation = tf2::toMsg(Orientation_qaut_e);

        EKF_UTM_local.pose.pose.position.x = X_estimated_k[0];
        EKF_UTM_local.pose.pose.position.y = X_estimated_k[1];
        EKF_UTM_local.twist.twist.linear.x = X_estimated_k[3];
        EKF_UTM_local.twist.twist.angular.z = X_estimated_k[4];

        EKF_UTM_local_pub.publish(EKF_UTM_local);
        Visualization();

        // std::cout << "#***********#" <<std::endl;
        // std::cout << std::endl;
        return;
    }
    // std::cout << "K1" << K << std::endl << std::endl;
    // std::cout << "P1" << P << std::endl << std::endl;
    // std::cout << "H1" << H << std::endl << std::endl;
    // std::cout << "A1" << A << std::endl << std::endl;
    Estimation();
    // std::cout << "K2" << K << std::endl << std::endl;
    // std::cout << "P2" << P << std::endl << std::endl;
    // std::cout << "H2" << H << std::endl << std::endl;
    // std::cout << "A2" << A << std::endl << std::endl;
    KalmanGain();
    // std::cout << "K3" << K << std::endl << std::endl;
    // std::cout << "P3" << P << std::endl << std::endl;
    // std::cout << "H3" << H << std::endl << std::endl;
    // std::cout << "A3" << A << std::endl << std::endl;
    Calculation();
    // std::cout << "K4" << K << std::endl << std::endl;
    // std::cout << "P4" << P << std::endl << std::endl;
    // std::cout << "H4" << H << std::endl << std::endl;
    // std::cout << "A4" << A << std::endl << std::endl;
    Covariance();

    //Convert orientation to quaternion form
    tf2::Quaternion Orientation_qaut;
    Orientation_qaut.setRPY(0, 0, X_calculated_k[2]);
    Orientation_qaut = Orientation_qaut.normalize();
    EKF_UTM_local.pose.pose.orientation = tf2::toMsg(Orientation_qaut);

    //Publish EKF_UTM
    EKF_UTM_local.pose.pose.position.x = X_calculated_k[0];
    EKF_UTM_local.pose.pose.position.y = X_calculated_k[1];
    EKF_UTM_local.twist.twist.linear.x = X_calculated_k[3];
    EKF_UTM_local.twist.twist.angular.z = X_calculated_k[4];
    EKF_UTM_local_pub.publish(EKF_UTM_local);
    Visualization();

    is_utm_raw_updated = false;
}

void EKF::Visualization(){
    //visualization TF ("odom" --> "base_link")
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(EKF_UTM_local.pose.pose.position.x, EKF_UTM_local.pose.pose.position.y, 0));
    transform.setRotation(tf::Quaternion(imu_quat.x, imu_quat.y, imu_quat.z, imu_quat.w));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

    //visualization marker
    utm_EKF_local_viz.header.stamp = ros::Time::now();
    utm_EKF_local_viz.header.frame_id = "odom";
    utm_EKF_local_viz.child_frame_id = "base_link";
    utm_EKF_local_viz.pose.pose.position.x = EKF_UTM_local.pose.pose.position.x;
    utm_EKF_local_viz.pose.pose.position.y = EKF_UTM_local.pose.pose.position.y;
    utm_EKF_local_viz.pose.pose.orientation = EKF_UTM_local.pose.pose.orientation;
    visual_pub.publish(utm_EKF_local_viz);
}
int main (int argc, char** argv){
    ros::init (argc, argv, "EKF");
    ros::NodeHandle _nh;
    
    printf("Initiate: EKF\n");

    EKF ekf(_nh);

    spin(); //loop rate 가 중요한 경우
 
    printf("Terminate: EKF\n");

    return 0;
}