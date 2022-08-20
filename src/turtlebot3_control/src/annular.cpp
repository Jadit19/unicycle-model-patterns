#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

//! -------- CHANGEABLE PARAMETERS --------
const double V = 0.1;     // Velocity of the bot
const double R_MIN = 2; // Minimum radius of the annular region
const double R_MAX = 4; // Maximum radius of the annular region
const int RATE = 30;    // Rate at which the state of the bot will be updated

//! -------- DERIVED PARAMETERS --------
const double M = (R_MAX+R_MIN)*V/(R_MAX-R_MIN);     // Slope of the linear generating function
const double C = -2*V*(R_MAX*R_MIN)/(R_MAX-R_MIN);  // y-intercept of the linear generatung function

//! -------- FINITE STATE MACHINE --------
enum FSM {              // Finite state machine declaration as enumeration
    ALIGN,              // Responsible for the bot's initial alignment angle
    MOVE                // Responsible for moving the bot according to the control law
};

//! -------- GLOBAL PARAMETERS --------
geometry_msgs::Twist cmd_vel = geometry_msgs::Twist();  // Command velocity that is published to the TurtleBot
tf::Quaternion q;           // Quaternion responsible for converting orientation of the bot into RPY
double roll;                // Roll angle of the bot
double pitch;               // Pitch angle of the bot
double yaw;                 // Yaw angle of the bot
double angleDiff = 0;       // Angular difference for initial alignment
FSM state = FSM::ALIGN;     // State the bot currently is in

// Linear generating function, g(r) = mr + c
// => Control function f(r) = m/r
double f(double r){
    return (M / r);
}

// Stop the movement of the bot
void stopMoving(){
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    return;
}

// Calculating the roll, pitch and yaw of the bot
void updateRPY(const geometry_msgs::Quaternion& orientation){
    q.setW(orientation.w);
    q.setX(orientation.x);
    q.setY(orientation.y);
    q.setZ(orientation.z);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    return;
}

// Calculating u(t) = f(r)
double calcAlphaDot(const geometry_msgs::Point& position){
    double r = sqrt(position.x*position.x + position.y*position.y);
    r = std::min(4.0, std::max(2.0, r));
    return f(r);
}

// Odometry call-back function
void odometryCallback(const nav_msgs::OdometryConstPtr& msg){
    stopMoving();

    if (state == FSM::ALIGN){
        updateRPY(msg->pose.pose.orientation);
        angleDiff = abs(- M_PI_2 - yaw);
        if (angleDiff > 0.003){
            cmd_vel.angular.z = -0.1;
        } else {
            state = FSM::MOVE;
            std::cout << "Alignment Done!" << std::endl;
            std::cout << "Moving now.." << std::endl;
        }
    } else if (state == FSM::MOVE){
        cmd_vel.linear.x = V;
        cmd_vel.angular.z = calcAlphaDot(msg->pose.pose.position);
    }

    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "annular");
    ros::NodeHandle nh;

    ros::Publisher pub_cmd_vel_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", RATE);
    ros::Subscriber sub_odom_ = nh.subscribe<nav_msgs::Odometry>("odom", RATE, odometryCallback);
    ros::Rate loopRate(RATE);

    while (ros::ok()){
        ros::spinOnce();
        pub_cmd_vel_.publish(cmd_vel);
        loopRate.sleep();
    }

    return EXIT_SUCCESS;
}