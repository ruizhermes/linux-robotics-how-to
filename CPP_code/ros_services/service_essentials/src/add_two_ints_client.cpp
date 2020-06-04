#include "ros/ros.h"
#include "service_essentials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv){
    
    ros::init(argc, argv, "add_two_ints_client");
    if (argc != 3)
    {
        ROS_INFO("usage: add_two_ints_client X Y");
        return 1;
    }

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<service_essentials::AddTwoInts>("add_two_ints");
    
    // Creating a service Object. This is used to access the elements of the service class
    service_essentials::AddTwoInts srv;
    
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);
    
    // Calling the server 
    if (client.call(srv))
    {
        ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }

    return 0;
}