#include <iostream>
#include <utility>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

enum FSM {
    MOVE,
    ALIGN,
    TRAJECTORY
};

const int RATE = 30;
const double V = 0.2;
const double R_MIN = 2.0;
const double R_MAX = 2.5;
const double TOLERANCE = 0.001;
const double INITIAL_ANGLE = -M_PI_2;
std::vector<geometry_msgs::Point> targets;

geometry_msgs::Point pt1, pt2, pt3, pt4;

double M = (R_MAX+R_MIN)*V/(R_MAX-R_MIN);
double C = (V-M)*R_MAX;
double r;
double x, y;
double roll, pitch, yaw;

tf::Quaternion q;
nav_msgs::Path path = nav_msgs::Path();
geometry_msgs::Twist cmd_vel = geometry_msgs::Twist();
geometry_msgs::PoseStamped pose = geometry_msgs::PoseStamped();

FSM state = FSM::MOVE;

void stopMoving(){
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    return;
}

void addToPath(const nav_msgs::OdometryConstPtr& msg){
    path.header = msg->header;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    path.poses.push_back(pose);
    return;
}

void updateRPY(const geometry_msgs::Quaternion& orientation){
    q.setW(orientation.w);
    q.setX(orientation.x);
    q.setY(orientation.y);
    q.setZ(orientation.z);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    return;
}

double f(double r){
    return (-M/r);
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg){
    stopMoving();
    addToPath(msg);
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    if (state == FSM::MOVE){
        cmd_vel.linear.x = 0.05;
        if (abs(pt1.x+R_MAX-x) < TOLERANCE){
            state = FSM::ALIGN;
            ROS_WARN("\tMovement done. Aligning now");
        }
        return;
    }
    
    if (state == FSM::ALIGN){
        updateRPY(msg->pose.pose.orientation);
        double angleDiff = INITIAL_ANGLE - yaw;
        if (abs(angleDiff) > 0.003){
            if (angleDiff > 0)
                cmd_vel.angular.z = 0.1 + 0.04*angleDiff;
            else
                cmd_vel.angular.z = -0.1 + 0.04*angleDiff;
        } else {
            state = FSM::TRAJECTORY;
            ROS_WARN("\tAlignment done. Generating trajectory now..");
        }
        return;
    }
    
    cmd_vel.linear.x = V;
    r = 1e8;
    for (geometry_msgs::Point pt: targets){
        r = std::min(r, sqrt(pow(pt.x-x,2) + pow(pt.y-y,2)));
    }
    cmd_vel.angular.z = f(r);
    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "multi");
    ros::NodeHandle nh;

    ros::Publisher pub_cmd_vel_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", RATE);
    ros::Publisher pub_path_ = nh.advertise<nav_msgs::Path>("path", RATE);
    ros::Subscriber sub_odom_ = nh.subscribe<nav_msgs::Odometry>("odom", RATE, odomCallback);
    ros::Rate loopRate(RATE);
 
    
    pt1.x = 1; pt1.y = 0;
    pt2.x = 0; pt2.y = -1;
    pt3.x = -1; pt3.y = 0;
    pt4.x = 0; pt4.y = 1;

    targets.push_back(pt1);
    targets.push_back(pt2);
    targets.push_back(pt3);
    targets.push_back(pt4);

    while (ros::ok()){
        ros::spinOnce();
        pub_cmd_vel_.publish(cmd_vel);
        pub_path_.publish(path);
        loopRate.sleep();
    }

    return EXIT_SUCCESS;
}