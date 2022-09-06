#include <iostream>
#include <utility>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

//! -------- FINITE STATE MACHINE --------
enum FSM {                                  // Finite state machine declaration as enumeration
    MOVE,                                   // Move the bot to the required location
    ALIGN,                                  // Responsible for the bot's initial alignment angle
    TRAJECTORY                              // Generating trajectory based on the control law
}; 

//! -------- CHANGEABLE PARAMETERS --------
const int RATE = 30;                        // Rate at which the state of the bot will be updated
const double V = 0.1;                       // Velocity of the bot
const double R_MIN = 1.0;                   // Minimum radius of the annular region
const double R_MAX = 1.5;                   // Maximum radius of the annular region
const double TOLERANCE = 0.001;             // Tolerance limit for multiple purpose
const double INITIAL_ANGLE = -M_PI_2;       // Initial angle of the bot w.r.t. X-Axis
std::vector<geometry_msgs::Point> targets;  // Target points about which the bot has to switch

//! -------- DERIVED PARAMETERS --------
double M = (R_MAX+R_MIN)*V/(R_MAX-R_MIN);   // y = M*x + C
double C = (V-M)*R_MAX;                     // y = M*x + C

//! -------- GLOBAL PARAMETERS --------
double r;                                                       // distance of the bot from the closest target
double x, y;                                                    // Extracting the (x, y) coordinate from the odometry
double roll, pitch, yaw;                                        // Roll, Pitch and Yaw angle of the bot
tf::Quaternion q;                                               // Quaternion responsible for converting orientation of the bot to RPY
nav_msgs::Path path = nav_msgs::Path();                         // Path that is to be published to RVIZ
geometry_msgs::Twist cmd_vel = geometry_msgs::Twist();          // Command velocity published to the TurtleBot
geometry_msgs::PoseStamped pose = geometry_msgs::PoseStamped(); // Pose of the bot, used in RVIZ
FSM state = FSM::MOVE;                                          // State the bot currently is in

// Stop the movement of the bot
void stopMoving(){
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    return;
}

// adding new poses to the path of the bot
void addToPath(const nav_msgs::OdometryConstPtr& msg){
    path.header = msg->header;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    path.poses.push_back(pose);
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

// f(r) = (1/r) * (d/dr)g(r)
double f(double r){
    return (-M/r);
}

// Odometry call back function to subscriber
void odomCallback(const nav_msgs::OdometryConstPtr& msg){
    stopMoving();
    addToPath(msg);
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    if (state == FSM::MOVE){
        cmd_vel.linear.x = 0.05;
        if (abs(targets[0].x+R_MAX-x) < TOLERANCE){
            state = FSM::ALIGN;
            ROS_WARN("\tMovement done. Aligning now..");
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
    for (geometry_msgs::Point target: targets){
        r = std::min(r, sqrt(pow(target.x-x,2) + pow(target.y-y,2)));
    }
    cmd_vel.angular.z = f(r);
    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "target");
    ros::NodeHandle nh;

    ros::Publisher pub_cmd_vel_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", RATE);
    ros::Publisher pub_path_ = nh.advertise<nav_msgs::Path>("path", RATE);
    ros::Subscriber sub_odom_ = nh.subscribe<nav_msgs::Odometry>("odom", RATE, odomCallback);
    ros::Rate loopRate(RATE);

    int noOfTargets, x, y;
    geometry_msgs::Point p;
    p.z = 0;

    ROS_WARN("\tEnter the number of targets:");
    std::cin >> noOfTargets;
    if (noOfTargets < 2){
        ROS_ERROR("\tEnter at least 2 targets to switch between.\n");
        return EXIT_FAILURE;
    }

    for (int i=1; i<=noOfTargets; i++){
        std::cout << std::endl;
        ROS_WARN("\tPosition (x, y) of target #%d:", i);
        std::cout << "\tEnter x coordinate: ";
        std::cin >> p.x;
        std::cout << "\tEnter y coordinate: ";
        std::cin >> p.y;
        targets.push_back(p);
    }
    std::cout << std::endl;
    for (int i=0; i<=noOfTargets+1; i++){
        geometry_msgs::Point c1 = targets[(i)%noOfTargets];
        geometry_msgs::Point c2 = targets[(i+1)%noOfTargets];
        double dist = sqrt(pow(c1.x-c2.x,2) + pow(c1.y-c2.y,2));
        if (dist > (R_MAX+R_MIN)){
            ROS_ERROR("\tDistance between point %d and %d is %lf > %lf\n", (i%noOfTargets), ((i+1)%noOfTargets), dist, (R_MAX+R_MIN));
            return EXIT_FAILURE;
        }
    }
    ROS_WARN("\tTargets initialized successfully. Moving now..");

    while (ros::ok()){
        ros::spinOnce();
        pub_cmd_vel_.publish(cmd_vel);
        pub_path_.publish(path);
        loopRate.sleep();
    }

    return EXIT_SUCCESS;
}