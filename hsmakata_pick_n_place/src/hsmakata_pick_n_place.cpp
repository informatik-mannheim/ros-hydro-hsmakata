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

#include <shape_tools/solid_primitive_dims.h>

ros::Publisher pub_co;
ros::Publisher pub_aco;
ros::Publisher grasps_marker;

ros::Subscriber sub_point;

moveit_msgs::CollisionObject co;
moveit_msgs::AttachedCollisionObject aco;

void add_collision_object()
{
    ROS_INFO("Adding collision object.");
    co.operation = moveit_msgs::CollisionObject::ADD;
    pub_co.publish(co);
}
void remove_collision_object()
{
    ROS_INFO("Removing collision object.");
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co.publish(co);
}
void add_attached_collision_object()
{
    ROS_INFO("Adding attached collision object.");
    aco.object.operation = moveit_msgs::CollisionObject::ADD;
    pub_aco.publish(aco);
}
void remove_attached_collision_object()
{
    ROS_INFO("Removing attached collision object.");
    aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_aco.publish(aco);
}

void hypo(double &x, double &y, float dist)
{
    double alpha = atan2(y,x);
    double xs = cos(alpha)*dist;
    double ys = sin(alpha)*dist;

    x -= xs;
    y -= ys;
}

moveit_msgs::Grasp tf_transform_to_grasp(tf::Transform t)
{
    static int i = 0;

    moveit_msgs::Grasp grasp;
    geometry_msgs::PoseStamped pose;

    tf::Vector3& origin = t.getOrigin();
    tf::Quaternion rotation = t.getRotation();

    tf::quaternionTFToMsg(rotation, pose.pose.orientation);

    pose.header.frame_id = "base_link";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = origin.m_floats[0];
    pose.pose.position.y = origin.m_floats[1];
    pose.pose.position.z = origin.m_floats[2];

    grasp.grasp_pose = pose;
    double x = grasp.grasp_pose.pose.position.x;
    double y = grasp.grasp_pose.pose.position.y;
    //hypo(x,y,0.1);

    grasp.id = boost::lexical_cast<std::string>(i);

    grasp.pre_grasp_approach.direction.vector.x = 1.0;
   //grasp.post_grasp_retreat.direction.vector.z = -1.0;

    grasp.pre_grasp_approach.direction.header.frame_id = "katana_gripper_tool_frame";
    grasp.pre_grasp_approach.min_distance = 0.07;
    grasp.pre_grasp_approach.desired_distance = 0.15;
    grasp.pre_grasp_approach.direction.header.stamp = ros::Time::now();

    grasp.post_grasp_retreat.direction.vector.x = -1.0;
    grasp.post_grasp_retreat.direction.vector.z = 1.0;
    grasp.post_grasp_retreat.min_distance = 0.07;
    grasp.post_grasp_retreat.desired_distance = 0.15;

    grasp.post_grasp_retreat.direction.header.frame_id = "katana_gripper_tool_frame";
    grasp.post_grasp_retreat.direction.header.stamp = ros::Time::now();

    // TODO: fill in grasp.post_place_retreat (copy of post_grasp_retreat?)

    grasp.pre_grasp_posture.joint_names.push_back("katana_l_finger_joint");
    grasp.pre_grasp_posture.joint_names.push_back("katana_r_finger_joint");
    grasp.pre_grasp_posture.points.resize(1);
    grasp.pre_grasp_posture.points[0].positions.push_back(0.3);
    grasp.pre_grasp_posture.points[0].positions.push_back(0.3);

    grasp.grasp_posture.joint_names.push_back("katana_l_finger_joint");
    grasp.grasp_posture.joint_names.push_back("katana_r_finger_joint");
    grasp.grasp_posture.points.resize(1);
    grasp.grasp_posture.points[0].positions.push_back(-0.44);
    grasp.grasp_posture.points[0].positions.push_back(-0.44);

    grasp.allowed_touch_objects.resize(1);
    grasp.allowed_touch_objects[0] = "testbox";

    i++;
    return grasp;
}

moveit_msgs::PlaceLocation tf_transform_to_place(tf::Transform t)
{
    static int i = 0;

    moveit_msgs::PlaceLocation place;
    geometry_msgs::PoseStamped pose;

    tf::Vector3& origin = t.getOrigin();
    tf::Quaternion rotation = t.getRotation();

    tf::quaternionTFToMsg(rotation, pose.pose.orientation);

    pose.header.frame_id = "base_link";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = origin.m_floats[0];
    pose.pose.position.y = origin.m_floats[1];
    pose.pose.position.z = origin.m_floats[2];

    place.place_pose = pose;
    double x = place.place_pose.pose.position.x;
    double y = place.place_pose.pose.position.y;
    hypo(x,y,0.1);

    place.id = boost::lexical_cast<std::string>(i);

    //place.pre_place_approach.direction.vector.x = 1.0;
    place.pre_place_approach.direction.vector.z = -1.0;
    place.pre_place_approach.direction.header.frame_id = "katana_gripper_tool_frame";
    place.pre_place_approach.min_distance = 0.07;
    place.pre_place_approach.desired_distance = 0.15;
    place.pre_place_approach.direction.header.stamp = ros::Time::now();

    place.post_place_retreat.direction.vector.x = -1.0;
    //place.post_place_retreat.direction.vector.z = 1.0;
    place.post_place_retreat.min_distance = 0.07;
    place.post_place_retreat.desired_distance = 0.15;
    place.post_place_retreat.direction.header.frame_id = "katana_gripper_tool_frame";
    place.post_place_retreat.direction.header.stamp = ros::Time::now();

    // TODO: fill in grasp.post_place_retreat (copy of post_grasp_retreat?)

    place.post_place_posture.joint_names.push_back("katana_l_finger_joint");
    place.post_place_posture.joint_names.push_back("katana_r_finger_joint");
    place.post_place_posture.points.resize(1);
    place.post_place_posture.points[0].positions.push_back(0.3);
    place.post_place_posture.points[0].positions.push_back(0.3);

    place.allowed_touch_objects.resize(1);
    place.allowed_touch_objects[0] = "testbox";

    i++;
    return place;
}

void publish_grasps_as_markerarray(std::vector<moveit_msgs::Grasp> grasps)
{
    visualization_msgs::MarkerArray markers;
    int i = 0;


    for(std::vector<moveit_msgs::Grasp>::iterator it = grasps.begin(); it != grasps.end(); ++it)
    {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "base_link";
        marker.id = i;
        marker.type = 0;
        marker.ns = "graspmarker";
        marker.pose = it->grasp_pose.pose;
        marker.scale.x = 0.05;
        marker.scale.y = 0.005;
        marker.scale.z = 0.005;
        marker.color.b = 1;
        marker.color.a = 1;
        markers.markers.push_back(marker);
        i++;
    }

    grasps_marker.publish(markers);
}

void publish_places_as_markerarray(std::vector<moveit_msgs::PlaceLocation> places)
{
    visualization_msgs::MarkerArray markers;
    int i = 0;

    for(std::vector<moveit_msgs::PlaceLocation>::iterator it = places.begin(); it != places.end(); ++it)
    {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "base_link";
        marker.id = i;
        marker.type = 0;
        marker.ns = "graspmarker";
        marker.pose = it->place_pose.pose;
        marker.scale.x = 0.05;
        marker.scale.y = 0.005;
        marker.scale.z = 0.005;
        marker.color.b = 1;
        marker.color.a = 1;
        markers.markers.push_back(marker);
        i++;
    }

    grasps_marker.publish(markers);
}


std::vector<moveit_msgs::Grasp> generate_grasps(double x,double y,double z)
{
    static const double ANGLE_INC = M_PI / 16;
    static const double START_ANGLE = 0;//(3* M_PI)/2;
    // how far from the grasp center should the wrist be?
    static const double STANDOFF = 0.05;

    std::vector<moveit_msgs::Grasp> grasps;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, 0.0));

    tf::Transform standoff_trans;
    standoff_trans.setOrigin(tf::Vector3(STANDOFF, 0.0, 0.0));
    standoff_trans.setRotation(tf::Quaternion(0.0, 0.0, 0.0, sqrt(2)/2));


    double yaw = atan2(y, x);
    double x_u = x;
    double y_u = y;

    for(double z_u = 0; z_u <= 0.04; z_u += 0.005)
    {

        transform.setOrigin(tf::Vector3(x_u, y_u, z + z_u));
        transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, yaw));
        grasps.push_back(tf_transform_to_grasp(transform * standoff_trans));

        transform.setOrigin(tf::Vector3(x_u, y_u, z - z_u));
        transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, yaw));
        grasps.push_back(tf_transform_to_grasp(transform * standoff_trans));
    }

    double roll = 0.0;

   //for (double pitch = -ANGLE_MAX - 4*ANGLE_INC ; pitch <= STRAIGHT_ANGLE_MIN - ANGLE_MAX/2  +0.01; pitch += ANGLE_INC)
   for (int cnt = 0 ; cnt <= 3; cnt++)
   {
       transform.setOrigin(tf::Vector3(x, y, z));
       transform.setRotation(tf::createQuaternionFromRPY(roll, START_ANGLE + cnt*ANGLE_INC, yaw));
       grasps.push_back(tf_transform_to_grasp(transform * standoff_trans));

       if(cnt != 0)
       {
           transform.setOrigin(tf::Vector3(x, y, z));
           transform.setRotation(tf::createQuaternionFromRPY(roll, START_ANGLE - cnt*ANGLE_INC, yaw));
           grasps.push_back(tf_transform_to_grasp(transform * standoff_trans));
       }
   }


    publish_grasps_as_markerarray(grasps);

    return grasps;
}

std::vector<moveit_msgs::PlaceLocation> generate_places(double x,double y,double z)
{
    static const double ANGLE_INC = M_PI / 16;
    static const double START_ANGLE = 0;//(3* M_PI)/2;
    // how far from the grasp center should the wrist be?
    static const double STANDOFF = 0.005;

    std::vector<moveit_msgs::PlaceLocation> places;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, 0.0));

    tf::Transform standoff_trans;
    standoff_trans.setOrigin(tf::Vector3(0.0, 0.0, STANDOFF));
    standoff_trans.setRotation(tf::Quaternion(0.0, 0.0, 0.0, sqrt(2)/2));


    double yaw = atan2(y, x);
    double x_u = x;
    double y_u = y;

    for(double z_u = 0; z_u <= 0.04; z_u += 0.005)
    {

        transform.setOrigin(tf::Vector3(x_u, y_u, z + z_u));
        transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, yaw));
        places.push_back(tf_transform_to_place(transform * standoff_trans));

        transform.setOrigin(tf::Vector3(x_u, y_u, z - z_u));
        transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, yaw));
        places.push_back(tf_transform_to_place(transform * standoff_trans));
    }

    double roll = 0.0;

   //for (double pitch = -ANGLE_MAX - 4*ANGLE_INC ; pitch <= STRAIGHT_ANGLE_MIN - ANGLE_MAX/2  +0.01; pitch += ANGLE_INC)
   for (int cnt = 0 ; cnt <= 3; cnt++)
   {
       transform.setOrigin(tf::Vector3(x, y, z));
       transform.setRotation(tf::createQuaternionFromRPY(roll, START_ANGLE + cnt*ANGLE_INC, yaw));
       places.push_back(tf_transform_to_place(transform * standoff_trans));

       if(cnt != 0)
       {
           transform.setOrigin(tf::Vector3(x, y, z));
           transform.setRotation(tf::createQuaternionFromRPY(roll, START_ANGLE - cnt*ANGLE_INC, yaw));
           places.push_back(tf_transform_to_place(transform * standoff_trans));
       }
   }


    publish_places_as_markerarray(places);

    return places;
}


void cb_points(const visualization_msgs::MarkerConstPtr& data)
{
    bool success = false;
    double x = data->pose.position.x;
    double y = data->pose.position.y ;
    double z = data->pose.position.z;

    ROS_INFO("Points received! x: %f y: %f z: %f",x,y,z);



    moveit::planning_interface::MoveGroup group("manipulator");
    group.setPlanningTime(45.0);
    group.setNumPlanningAttempts(10);
    group.allowReplanning(true);
    group.clearPathConstraints();

    co.header.stamp = ros::Time::now();
    co.header.frame_id = "base_link";
    co.id = "testbox";
    co.primitives.resize(1);
    co.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
    co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.04;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.14;

    //co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.14;
    co.primitive_poses.resize(1);
    co.primitive_poses[0].position.x = x;
    co.primitive_poses[0].position.y = y;
    co.primitive_poses[0].position.z = z;
    co.primitive_poses[0].orientation.w = 1.0;

    aco.object = co;
    aco.link_name = "katana_gripper_tool_frame";

    add_collision_object();
    ros::WallDuration(5.0).sleep();

    ROS_INFO("Generating Grasps");
    std::vector<moveit_msgs::Grasp> grasps = generate_grasps(x, y, z);

    success = group.pick("testbox",grasps);
    if (success)
    {
        ROS_INFO("Pick was successful.");
    }
    else
    {
        ROS_FATAL("Pick failed.");
        //return EXIT_FAILURE;
    }

    ros::WallDuration(3.0).sleep();


    if(success)
    {
        std::vector<moveit_msgs::PlaceLocation> places = generate_places(x, -y, z);
        success = group.place("testbox",places);
        if (success)
        {

            ROS_INFO("Place was successful.");
            geometry_msgs::PoseStamped po = group.getCurrentPose();
            ROS_INFO("x: %f y: %f z: %f",po.pose.position.x,po.pose.position.y,po.pose.position.z);
            ROS_INFO("w: %f, x: %f y: %f z: %f",po.pose.orientation.w, po.pose.orientation.x, po.pose.orientation.y, po.pose.orientation.z);
        }
        else
        {
            ROS_FATAL("Place failed.");
            //return EXIT_FAILURE;
        }
    }
    group.setStartState(*group.getCurrentState());
    remove_collision_object();

    group.setStartState(*group.getCurrentState());
    group.setNamedTarget("home");
    group.move();
    geometry_msgs::PoseStamped po = group.getCurrentPose();
    ROS_INFO("x: %f y: %f z: %f",po.pose.position.x,po.pose.position.y,po.pose.position.z);
    ROS_INFO("w: %f, x: %f y: %f z: %f",po.pose.orientation.w, po.pose.orientation.x, po.pose.orientation.y, po.pose.orientation.z);


}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "hsmakata_pick_n_place_demo");
    ros::NodeHandle nh;

    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
    pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);
    grasps_marker = nh.advertise<visualization_msgs::MarkerArray>("grasps_marker", 10);

    sub_point = nh.subscribe<visualization_msgs::Marker>("cup_center",1,cb_points);

    moveit::planning_interface::MoveGroup gripper("gripper");
    gripper.setNamedTarget("closed");
    gripper.move();

    moveit::planning_interface::MoveGroup katana("manipulator");
    katana.setNamedTarget("home");
    katana.move();


    ros::spin();


}

