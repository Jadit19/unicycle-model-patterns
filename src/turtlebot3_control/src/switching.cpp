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
    TRAJECTORY_1,       // Move according to trajectory 1
    TRAJECTORY_2        // Move according to trajectory 2
};

//! -------- GLOBAL PARAMETERS --------
geometry_msgs::Twist cmd_vel = geometry_msgs::Twist();  // Command velocity that is published to the TurtleBot
tf::Quaternion q;           // Quaternion responsible for converting orientation of the bot into RPY
double roll;                // Roll angle of the bot
double pitch;               // Pitch angle of the bot
double yaw;                 // Yaw angle of the bot
double angleDiff = 0;       // Angular difference for initial alignment
double rPrev;               // Previous value of r
double rSwitching;          // Switching radius
FSM state = FSM::ALIGN;     // State the bot currently is in

// Setting the derived parameters
void setDerivedParameters(){
    M = (R_MAX+R_MIN)*V/(R_MAX-R_MIN);
    C = -2*V*(R_MAX*R_MIN)/(R_MAX-R_MIN);
    INITIAL_ANGLE *= M_PI / 180;
    rPrev = R_MIN;
    rSwitching = (-C) / M;
}

double f1(double r){
    return (M / r);
}

double f2(double r){
    return (-M / r);
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
            state = FSM::TRAJECTORY_1;
            std::cout << std::endl << "Alignment Done!" << std::endl;
            std::cout << "Moving now.." << std::endl << std::endl;
        }
    } else {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double r = sqrt(x*x + y*y);
        cmd_vel.linear.x = V;
        if ((r-rSwitching)*(rPrev-rSwitching) <= 0){
            std::cout << "Switching now" << std::endl;
            if (state == FSM::TRAJECTORY_1)
                state = FSM::TRAJECTORY_2;
            else if (state == FSM::TRAJECTORY_2)
                state = FSM::TRAJECTORY_1;
        }
        if (state == FSM::TRAJECTORY_1)
            cmd_vel.angular.z = f1(r);
        else if (state == FSM::TRAJECTORY_2)
            cmd_vel.angular.z = f2(r);
        rPrev = r;
    }

    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "switching");
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