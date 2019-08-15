#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <service_server_pkg/MyCustomServiceMessage.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "service_move_bb8_in_square_client");
    ros::NodeHandle nh;

    // connect to the service /perform_square and add empty msg
    ros::ServiceClient perform_square_service_client = nh.serviceClient<service_server_pkg::MyCustomServiceMessage>("/perform_square");
    service_server_pkg::MyCustomServiceMessage srv;

    srv.request.radius = 3.0;
    srv.request.repetitions = 1;

    if (perform_square_service_client.call(srv))
    {
        ROS_INFO("Service successfully called. Moving BB8 in a square.");
    }
    else
    {
        ROS_ERROR("Failed to call service /perform_square");
        return 1;
    }

    return 0;
}