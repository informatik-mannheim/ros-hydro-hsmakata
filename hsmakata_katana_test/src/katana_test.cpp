#include <ros/ros.h>

#include <tf/tf.h>

#include <moveit/move_group_interface/move_group.h>

#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <shape_tools/solid_primitive_dims.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"katana_test");
    ros::NodeHandle nh;




    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    moveit_msgs::DisplayTrajectory display_trajectory;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    geometry_msgs::PoseStamped p1;
    p1.header.stamp = ros::Time::now();
    p1.header.frame_id = "base_link";

    p1.pose.orientation.w = 1.0;
    p1.pose.position.x = 0.5;
    p1.pose.position.y = 0.0;
    p1.pose.position.z = 0.8;

    geometry_msgs::PoseStamped p2;
    p2.header.stamp = ros::Time::now();
    p2.header.frame_id = "base_link";

    p2.pose.orientation.w = 1.0;
    p2.pose.position.x = 0.45;
    p2.pose.position.y = 0.2;
    p2.pose.position.z = 0.6;


    moveit::planning_interface::MoveGroup katana("manipulator");

    katana.setNamedTarget("home");
    katana.move();


    geometry_msgs::Point p;
    p.x = 0.4;
    p.y = 0.25;
    p.z = 0.85;

    /*
    katana.setPositionTarget(p.x,p.y,p.z,"katana_gripper_tool_frame");

    std::vector<double> currJoints = katana.getCurrentJointValues();
    std::vector<std::string> jointNames = katana.getActiveJoints();

    for(int i = 0; i < currJoints.size(); i++)
    {
        katana.setJointValueTarget(jointNames[i],currJoints[i]);
    }
    katana.setJointValueTarget("katana_motor5_wrist_roll_joint",1.57);
    katana.move();


    currJoints = katana.getCurrentJointValues();
    jointNames = katana.getActiveJoints();

    for(int i = 0; i < currJoints.size(); i++)
    {
        katana.setJointValueTarget(jointNames[i],currJoints[i]);
    }
    katana.setJointValueTarget("katana_motor5_wrist_roll_joint",1.57);
    katana.setJointValueTarget("katana_motor1_pan_joint",1.57);
    katana.move();

    ros::Duration(2.0).sleep();

    katana.setJointValueTarget("katana_motor1_pan_joint",1.57);
    katana.move();

    ros::Duration(2.0).sleep();

    katana.setJointValueTarget("katana_motor1_pan_joint",-1.57);
    katana.move();

    ros::Duration(2.0).sleep();

    katana.setPositionTarget(p.x,p.y,p.z,"katana_gripper_tool_frame");
    katana.move();

    ros::Duration(2.0).sleep();


    katana.setPlanningTime(45.0);
    tf::Quaternion q;
    geometry_msgs::Quaternion q2;
    geometry_msgs::Pose pose;


    q = tf::createQuaternionFromRPY(0.0,0.0,0.0);
    tf::quaternionTFToMsg(q,q2);

    pose.position = p;
    pose.orientation = q2;

    std::cout << "test";
    katana.setPoseTarget(pose,"katana_gripper_tool_frame");
    //katana.move();

    std::vector<double> rpy = katana.getCurrentRPY();

    for(int i = 0; i < rpy.size(); i++)
    {
        ROS_INFO("%f",rpy[i]);
    }

    katana.setPositionTarget(p.x,p.y,p.z,"katana_gripper_tool_frame");
    katana.move();

    rpy = katana.getCurrentRPY();

    for(int i = 0; i < rpy.size(); i++)
    {
        ROS_INFO("%f",rpy[i]);
    }
 */
    katana.allowReplanning(true);

    katana.setPositionTarget(0.3,0.4,0.9,katana.getEndEffectorLink());
    katana.move();

    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose pose = katana.getCurrentPose(katana.getEndEffectorLink()).pose;

    pose.position.x += 0.05;
    pose.position.y += 0.1;

    waypoints.push_back(pose);

    pose.position.x -= 0.1;
    pose.position.y -= 0.2;

    waypoints.push_back(pose);
    moveit_msgs::RobotTrajectory trajectory;
    double frac = katana.computeCartesianPath(waypoints, 0.01, 0.0 ,trajectory);






    ros::spin();






}
