
#include <limits>
#include <atomic>
#include <string>
#include <stdexcept>
#include <exception>

#include <Eigen/Dense>

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <std_msgs/Bool.h>
#include <waypoint_manager_msgs/Waypoint.h>

namespace {
    static std::atomic_bool recived_waypoint;
    static float default_goal_radius = 1;
    static float current_goal_radius = default_goal_radius;
    static Eigen::Vector2f goal_position = Eigen::Vector2f::Zero();
}

void waypointCallback(const waypoint_manager_msgs::Waypoint::ConstPtr &msg) {
    try {
        goal_position.x() = msg->pose.position.x;
        goal_position.y() = msg->pose.position.y;

        bool found_goal_radius = false;

        for(unsigned int i = 0; i < msg->properties.size(); i ++) {
            if(msg->properties[i].name == "goal_radius") {
                current_goal_radius = std::stof(msg->properties[i].data);
                found_goal_radius = true;
            }
        }
        if(!found_goal_radius) {
            current_goal_radius = default_goal_radius;
        }
        recived_waypoint.store(true);
    }
    catch(const std::exception &) {
        recived_waypoint.store(false);
        ROS_WARN("Failed parse radius_node");
    }
}

auto main(int argc, char **argv) -> int {
    ros::init(argc, argv, "radius_node");
    ros::NodeHandle nh,
                    private_nh("~");

    std::string goal_topic,
                waypoint_topic,
                is_reached_goal_topic;

    std::string robot_base_frame,
                global_frame;

    float goal_check_frequency,
          wait_no_waypoint_time;

    recived_waypoint.store(false);

    private_nh.param(
        "goal_topic",
        goal_topic,
        std::string("move_base_simple/goal")
    );
    private_nh.param(
        "waypoint",
        waypoint_topic,
        std::string("waypoint")
    );
    private_nh.param(
        "is_reached_goal_topic",
        is_reached_goal_topic,
        std::string("waypoint/is_reached")
    );
    private_nh.param(
        "robot_base_frame",
        robot_base_frame,
        std::string("base_link")
    );
    private_nh.param(
        "global_frame",
        global_frame,
        std::string("map")
    );
    private_nh.param(
        "goal_check_frequency",
        goal_check_frequency,
        static_cast<float>(1.0)
    );
    private_nh.param(
        "wait_no_waypoint_time",
        wait_no_waypoint_time,
        static_cast<float>(5.0)
    );
    private_nh.param(
        "default_goal_radius",
        default_goal_radius,
        static_cast<float>(1.0)
    );
    current_goal_radius = default_goal_radius;

    if(robot_base_frame == global_frame) {
        throw std::runtime_error("Please set different frame name robot_base_frame or global_frame");
    }

    auto loop_rate = ros::Rate(goal_check_frequency);
    auto is_reached_goal_publisher = nh.advertise<std_msgs::Bool>(
        is_reached_goal_topic,
        1,
        true
    );
    auto waypoint_subscriber = nh.subscribe(
        waypoint_topic,
        1,
        waypointCallback
    );

    ROS_INFO("Start radius_node");
    ROS_INFO("robot_base_frame is %s", robot_base_frame.c_str());
    ROS_INFO("global_base_frame is %s", global_frame.c_str());
    ROS_INFO("default_goal_radius is %f", default_goal_radius);

    tf::TransformListener tf_listener;

    while(ros::ok()) {
        Eigen::Vector2f distance_of_goal;

        ros::spinOnce();

        if(!recived_waypoint.load()) {
            ROS_INFO("Waiting waypoint radius_node");
            ros::Duration(wait_no_waypoint_time).sleep();
            continue;
        }

        try {
            tf::StampedTransform tf_transform;
            tf_listener.lookupTransform(
                global_frame,
                robot_base_frame,
                ros::Time(0),
                tf_transform
            );

            distance_of_goal.x() = goal_position.x() - tf_transform.getOrigin().x();
            distance_of_goal.y() = goal_position.y() - tf_transform.getOrigin().y();

            std_msgs::Bool msg;

            if(distance_of_goal.lpNorm<2>() < current_goal_radius) {
                ROS_INFO("Goal reached from goal_event_handler::radius_node");
                msg.data = true;
            }
            else {
                msg.data = false;
            }

            is_reached_goal_publisher.publish(msg);
        }
        catch(const tf::TransformException &e) {
            ROS_WARN("Transform failed %s to %s", global_frame.c_str(), robot_base_frame.c_str());
            ros::Duration(1.0).sleep();
        }
        loop_rate.sleep();
    }
    ROS_INFO("Finish waypoint_server_node");

    return 0;
}

