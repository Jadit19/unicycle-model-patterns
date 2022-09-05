#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

const int RATE = 30;

geometry_msgs::Point fileWaypoint = geometry_msgs::Point();
std::vector<geometry_msgs::Point> waypoints;

int main(int argc, char** argv){
    ros::init(argc, argv, "trial");
    ros::NodeHandle nh;

    fileWaypoint.z = 0;
    std::fstream file;
    std::string line;
    file.open("/home/adit/unicycle-model-patterns/src/turtlebot3_follower/path/waypoints.txt", std::ios::in);
    if (file.is_open()){
        std::string x, y;
        while (getline(file, line)){
            std::stringstream ss(line);
            ss >> x >> y;
            fileWaypoint.x = std::stoi(x);
            fileWaypoint.y = std::stoi(y);
            waypoints.push_back(fileWaypoint);
        }
        ROS_WARN("No. of waypoints read = %lu", waypoints.size());
        file.close();
    } else {
        ROS_ERROR("\tCouldn't open file");
        return EXIT_FAILURE;
    }

    ros::Rate loopRate(RATE);

    while (ros::ok()){
        ros::spinOnce();

        loopRate.sleep();
    }

    return EXIT_SUCCESS;
}