
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <waypoint_manager_msgs/Waypoint.h>

class Node {
    public :
        Node();

        void spin();

    private :
        bool latch;

        std::string pose_topic,
                    waypoint_topic,
                    frame_id;

        ros::NodeHandle nh,
                        private_nh;

        ros::Publisher pose_publisher;
        ros::Subscriber waypoint_subscriber;

        void waypointCallback(const waypoint_manager_msgs::Waypoint::ConstPtr &);
};

Node::Node() : nh(), private_nh("~") {
    private_nh.param(
        "latch",
        latch,
        false
    );
    private_nh.param(
        "frame_id",
        frame_id,
        std::string("map")
    );
    private_nh.param(
        "pose_topic",
        pose_topic,
        std::string("move_base_simple/goal")
    );
    private_nh.param(
        "waypoint_topic",
        waypoint_topic,
        std::string("waypoint")
    );

    pose_publisher = nh.advertise<geometry_msgs::PoseStamped>(
        pose_topic,
        1,
        latch
    );
    waypoint_subscriber = nh.subscribe<waypoint_manager_msgs::Waypoint>(
        waypoint_topic,
        1,
        &Node::waypointCallback,
        this
    );
}

void Node::spin() {
    ros::spin();
}

void Node::waypointCallback(const waypoint_manager_msgs::Waypoint::ConstPtr &msg) {
    geometry_msgs::PoseStamped pose_stamped;

    pose_stamped.pose = msg->pose;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = ros::Time::now();

    pose_publisher.publish(pose_stamped);
}

auto main(int argc, char **argv) -> int {
    ros::init(argc, argv, "waypoint_to_posestamped_node");

    Node node{};
    node.spin();

    return 0;
}
