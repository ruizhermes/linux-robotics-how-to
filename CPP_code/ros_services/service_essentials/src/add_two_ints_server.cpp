#include "ros/ros.h"
#include "service_essentials/AddTwoInts.h"

bool add(service_essentials::AddTwoInts::Request  &req,
         service_essentials::AddTwoInts::Response &resp)
{
    resp.sum = req.a + req.b;
    ROS_INFO("request: a=%ld, b=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("Sending back respoinse [%ld]", (long int)resp.sum);
    return true;
}

int main(int argc, char **argv){


    ros::init(argc, argv, "add_two_ints_server");
    ros::NodeHandle nh;

    // nh.advertiseService("name of service used by client", arguments (eg. function_call))
    ros::ServiceServer service = nh.advertiseService("add_two_ints", add);
    ROS_INFO("Ready to add two ints");
    ros::spin();

}


