#include <iostream>
#include <fstream>
#include <tf/tf.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

enum FSM {
    MOVE,
    ALIGN,
    TRAJECTORY
};

int RATE;
bool TAKE_OUTPUT;
double TOLERANCE;
double LINEAR_VEL;
double ANGULAR_VEL;
double INITIAL_ANGLE;

int current = 1;
double prevDist = 1e4;
double roll, pitch, yaw;
std::ofstream outputFile;

tf::Quaternion q;
nav_msgs::Path path = nav_msgs::Path();
std::vector<geometry_msgs::Point> waypoints;
geometry_msgs::Twist cmd_vel = geometry_msgs::Twist();
geometry_msgs::PoseStamped pose = geometry_msgs::PoseStamped();
geometry_msgs::Point fileWaypoint = geometry_msgs::Point();

FSM state = FSM::MOVE;

void addToPath(const nav_msgs::OdometryConstPtr& msg){
    path.header = msg->header;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    path.poses.push_back(pose);
    return;
}

void stopMoving(){
    cmd_vel.linear.x = 0;
    cmd_vel.angular.y = 0;
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

int getDirection(double theta){
    if (abs(theta-yaw)<0.1 || abs(M_PI*2+theta-yaw)<0.1 || abs(M_PI*2-theta+yaw)<0.1)
        return 0;
    
    if (theta > yaw){
        if (theta>M_PI_2 && yaw<-M_PI_2)
            return -1;
        else
            return 1;
    } else {
        if (theta<-M_PI_2 && yaw>M_PI_2)
            return 1;
        else
            return -1;
    }
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg){
    stopMoving();
    addToPath(msg);
    updateRPY(msg->pose.pose.orientation);
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    if (state == FSM::MOVE){
        cmd_vel.linear.x = 0.05;
        if (abs(waypoints[0].x-x) < TOLERANCE){
            state = FSM::ALIGN;
            ROS_WARN("\tMovement done. Aligning now...");
        }
        return;
    }

    if (state == FSM::ALIGN){
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

    if (state == FSM::TRAJECTORY){
        if (current == waypoints.size()){
            return;
        }
        double finalX = waypoints[current].x;
        double finalY = waypoints[current].y;
        double dist = sqrt(pow(finalX-x,2) + pow(finalY-y,2));
        if (dist >= prevDist){
            ROS_INFO("\tReached waypoint #%d with a distance of %f", current, prevDist);
            if (TAKE_OUTPUT)
                outputFile << finalX << "," << finalY << "," << x << "," << y << "\n";
            current++;
            prevDist = 1e4;
            return;
        }
        double theta = atan2(finalY-y, finalX-x);
        cmd_vel.angular.z = getDirection(theta) * ANGULAR_VEL;
        cmd_vel.linear.x = LINEAR_VEL;
        prevDist = dist;
        return;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "follower");
    ros::NodeHandle nh;

    std::string inputFileName;
    std::string outputFileName;

    if (!nh.getParam("linear_vel", LINEAR_VEL) || !nh.getParam("angular_vel", ANGULAR_VEL) || !nh.getParam("rate", RATE) || !nh.getParam("tolerance", TOLERANCE) || !nh.getParam("take_output", TAKE_OUTPUT) || !nh.getParam("initial_angle", INITIAL_ANGLE) || !nh.getParam("input_file", inputFileName)){
        ROS_ERROR("Can't load params..");
        ROS_ERROR("Exiting now\n");
        return EXIT_FAILURE;
    }
    if (TAKE_OUTPUT && !nh.getParam("output_file", outputFileName)){
        ROS_ERROR("Please specify an output file..");
        ROS_ERROR("Exiting now");
        return EXIT_FAILURE;
    }
    INITIAL_ANGLE *= M_PI / 180;

    fileWaypoint.z = 0;
    std::fstream inputFile;
    std::string line;
    inputFile.open(inputFileName, std::ios::in);
    outputFile.open(outputFileName);

    if (inputFile.is_open()){
        std::string x, y;
        while (getline(inputFile, line)){
            std::stringstream ss(line);
            ss >> x >> y;
            fileWaypoint.x = std::stod(x);
            fileWaypoint.y = std::stod(y);
            waypoints.push_back(fileWaypoint);
        }
        ROS_WARN("No. of waypoints read = %lu", waypoints.size());
        inputFile.close();
    } else {
        ROS_ERROR("Couldn't open input file");
        return EXIT_FAILURE;
    }

    if (TAKE_OUTPUT){
        if (outputFile.is_open()){
            outputFile << "x_matlab,y_matlab,x_gazebo,y_gazebo\n";
        } else {
            ROS_ERROR("Couldn't open output file");
            return EXIT_FAILURE;
        }
    }

    ros::Publisher pub_cmd_vel_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", RATE);
    ros::Publisher pub_path_ = nh.advertise<nav_msgs::Path>("path", RATE);
    ros::Subscriber sub_odom_ = nh.subscribe<nav_msgs::Odometry>("odom", RATE, odomCallback);
    ros::Rate loopRate(RATE);

    while (ros::ok()){
        ros::spinOnce();
        pub_cmd_vel_.publish(cmd_vel);
        pub_path_.publish(path);
        loopRate.sleep();
    }

    return EXIT_SUCCESS;
}