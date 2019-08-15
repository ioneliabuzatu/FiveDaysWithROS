#include <ros/ros.h>
#include<std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist where_to_move; // global object for publisher moving object TODO: make a class here with global and private

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    int length = 720; // len of the vector ranges left, front, right seens of the robot
    ROS_INFO("%f", msg->ranges[length/2]);
    if (msg->ranges[length/2]>1){ // far from the wall
        where_to_move.linear.x=0.1;
        where_to_move.angular.z=0.0;
        if(msg->ranges[length - length/4] < 0.4){
            where_to_move.linear.x=-0.1;
            where_to_move.angular.z=-0.2;
        }
        else if(msg->ranges[length/4] < 0.4){
            where_to_move.linear.x=-0.1;
            where_to_move.angular.z=0.2;
        }
    }
    else if (msg->ranges[length/2]<1){
        where_to_move.linear.x=0.0;
        if (msg->ranges[length/2]<0.7){
            where_to_move.linear.x=-0.1;
        }
        else if (msg->ranges[length - length/4] < msg->ranges[length/4]) // very close to the wall
            where_to_move.angular.z=-0.3; // move right
        else
            where_to_move.angular.z=0.3; // move left
    }
}



int main(int argc, char** argv) {

    ros::init(argc, argv, "read_write_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    ros::Rate loop_rate(2);
    where_to_move.linear.x=0.1;

    ros::Subscriber sub = nh.subscribe("kobuki/laser/scan", 1000, laserCallback);

    while (ros::ok()) { // while contrl + c stops the program, move the tobot

        pub.publish(where_to_move);
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
