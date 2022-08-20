#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

//! -------- CHANGEABLE PARAMETERS --------
double V;               // Velocity of the bot
double R_MIN;           // Minimum radius of the annular region
double R_MAX;           // Maximum radius of the annular region
double INITIAL_ANGLE;   // Initial angle of the bot w.r.t. X-Axis
int RATE;               // Rate at which the state of the bot will be updated

//! -------- DERIVED PARAMETERS --------
double M;               // Slope of the linear generating function
double C;               // y-intercept of the linear generatung function

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

// Setting the derived parameters
void setDerivedParameters(){
    M = (R_MAX+R_MIN)*V/(R_MAX-R_MIN);
    C = -2*V*(R_MAX*R_MIN)/(R_MAX-R_MIN);
    INITIAL_ANGLE *= M_PI / 180;
}

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
        angleDiff = abs(INITIAL_ANGLE - yaw);
        if (angleDiff > 0.003){
            if (INITIAL_ANGLE > yaw)
                cmd_vel.angular.z = 0.1 + 0.04*angleDiff;
            else
                cmd_vel.angular.z = -0.1 - 0.04*angleDiff;
        } else {
            state = FSM::MOVE;
            std::cout << std::endl << "Alignment Done!" << std::endl;
            std::cout << "Moving now.." << std::endl << std::endl;
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

    if (!nh.getParam("velocity", V) || !nh.getParam("minimum_radius", R_MIN) || !nh.getParam("maximum_radius", R_MAX) || !nh.getParam("refresh_rate", RATE) || !nh.getParam("initial_angle", INITIAL_ANGLE)){
        ROS_ERROR("\nCan't load params..");
        ROS_ERROR("Exiting now\n");
        return EXIT_FAILURE;
    }
    setDerivedParameters();

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