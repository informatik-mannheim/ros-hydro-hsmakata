#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>


ros::Publisher joint_states_pub;
ros::Subscriber yaw_sub;
ros::Subscriber pitch_sub;

int js_size = 0;


void js_callback(const sensor_msgs::JointState ptr)
{
    std::vector<std::string>::const_iterator it;

    for(it = ptr.name.begin(); it != ptr.name.end(); ++it)
    {
        ROS_INFO("%s",(*it).c_str());
    }
}

int main(int argc, char**argv)
{
    ros::init(argc,argv,"dynamixel_commander");
    ros::NodeHandle nh;
    std::vector<std::string> joints;

    ros::param::get("katana_joints",joints);

    for(std::vector<std::string>::const_iterator it = joints.begin(); it != joints.end(); ++it)
    {
        ROS_INFO("%s", (*it).c_str());
    }

    ros::Subscriber sub = nh.subscribe("/joint_states",10,  &js_callback);
    ros::Rate(20);
    ros::spin();

}
