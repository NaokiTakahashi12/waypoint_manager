
#include <iostream>
#include <string>
#include <functional>
#include <thread>

#include <Eigen/Dense>

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <std_srvs/Trigger.h>

#include <waypoint_manager_msgs/Waypoint.h>
#include <waypoint_manager_msgs/WaypointStamped.h>
#include <waypoint_manager_msgs/Waypoints.h>
#include <waypoint_manager_msgs/Route.h>

#include <waypoint_server/waypoint_server.h>

namespace waypoint_server {
    struct NodeParameters {
        std::string goal_topic,
                    waypoint_topic,
                    is_reached_goal_topic,
                    regist_goal_pose_topic,
                    regist_goal_point_topic,
                    erase_goal_topic,
                    update_goal_topic,
                    route_topic,
                    append_route_topic,
                    erase_route_topic,
                    waypoints_topic;

        std::string regist_waypoint_prefix;

        std::string robot_base_frame,
                    global_frame;

        std::string waypoints_file,
                    route_file;

        bool debug,
             latch,
             enable_2d,
             enable_3d,
             enable_loop;

        int publish_queue_size,
            subscribe_queue_size;

        float wait_publish_waypoints_time;

        double goal_publish_frequency;
    };

    class Node {
        public :
            Node();

            void spin();

        private :
            ros::NodeHandle private_nh,
                            nh;

            ros::Publisher waypoint_publisher,
                           waypoints_publisher,
                           route_publisher;

            ros::Subscriber is_reached_goal_subscriber,
                            regist_goal_pose_subscriber,
                            regist_goal_point_subscriber,
                            erase_goal_subscriber,
                            update_goal_subscriber,
                            append_route_subscriber,
                            erase_route_subscriber;

            ros::ServiceServer save_service,
                               save_waypoints_service,
                               save_route_service,
                               reset_route_service,
                               switch_cancel_service,
                               next_waypoint_service;

            NodeParameters param;

            Map waypoint_map;
            Route router;

            std::atomic<unsigned int> regist_goal_id;
            std::atomic_bool is_cancel;

            // TODO Change message type
            void isReachedGoal(const std_msgs::Bool::ConstPtr &),
                 registGoalPose(const geometry_msgs::PoseStamped::ConstPtr &),
                 registGoalPoint(const geometry_msgs::PointStamped::ConstPtr &),
                 eraseGoal(const std_msgs::String::ConstPtr &),
                 updateGoalPose(const waypoint_manager_msgs::WaypointStamped::ConstPtr &),
                 appendRoute(const std_msgs::String::ConstPtr &),
                 eraseRoute(const std_msgs::String::ConstPtr &);

            bool save(
                std_srvs::TriggerRequest &request,
                std_srvs::TriggerResponse &response
            );
            bool saveWaypoints(
                std_srvs::TriggerRequest &request,
                std_srvs::TriggerResponse &response
            );
            bool saveRoute(
                std_srvs::TriggerRequest &request,
                std_srvs::TriggerResponse &response
            );
            bool resetRoute(
                std_srvs::TriggerRequest &request,
                std_srvs::TriggerResponse &response
            );
            bool switchCancel(
                std_srvs::TriggerRequest &request,
                std_srvs::TriggerResponse &response
            );
            bool nextWaypoint(
                std_srvs::TriggerRequest &request,
                std_srvs::TriggerResponse &response
            );

            void publishGoal(),
                 publishWaypoints(),
                 publishRoute();

            void publishLatchedData();

            void exchangeCancelState();
    };

    Node::Node() :
        private_nh("~"),
        nh() {
        
        private_nh.param(
            "goal_topic",
            param.goal_topic,
            std::string("move_base_simple/goal")
        );
        private_nh.param(
            "waypoint_topic",
            param.waypoint_topic,
            std::string("waypoint")
        );
        private_nh.param(
            "is_reached_goal_topic",
            param.is_reached_goal_topic,
            std::string("waypoint/is_reached")
        );
        private_nh.param(
            "regist_goal_pose_topic",
            param.regist_goal_pose_topic,
            std::string("waypoint/regist_pose")
        );
        private_nh.param(
            "regist_goal_point_topic",
            param.regist_goal_point_topic,
            std::string("waypoint/regist_point")
        );
        private_nh.param(
            "erase_goal_topic",
            param.erase_goal_topic,
            std::string("waypoint/erase")
        );
        private_nh.param(
            "update_goal_topic",
            param.update_goal_topic,
            std::string("waypoint/update")
        );
        private_nh.param(
            "route_topic",
            param.route_topic,
            std::string("route")
        );
        private_nh.param(
            "append_route_topic",
            param.append_route_topic,
            std::string("route/append")
        );
        private_nh.param(
            "erase_route_topic",
            param.erase_route_topic,
            std::string("route/erase")
        );
        private_nh.param(
            "waypoints_topic",
            param.waypoints_topic,
            std::string("waypoints")
        );
        private_nh.param(
            "regist_waypoint_prefix",
            param.regist_waypoint_prefix,
            std::string("registed_")
        );
        private_nh.param(
            "robot_base_frame",
            param.robot_base_frame,
            std::string("base_link")
        );
        private_nh.param(
            "global_frame",
            param.global_frame,
            std::string("map")
        );
        private_nh.param(
            "waypoints_file",
            param.waypoints_file,
            std::string("")
        );
        private_nh.param(
            "route_file",
            param.route_file,
            std::string("")
        );
        private_nh.param(
            "debug",
            param.debug,
            false
        );
        private_nh.param(
            "latch",
            param.latch,
            false
        );
        private_nh.param(
            "enable_loop",
            param.enable_loop,
            false
        );
        private_nh.param(
            "publish_queue_size",
            param.publish_queue_size,
            1
        );
        private_nh.param(
            "subscribe_queue_size",
            param.subscribe_queue_size,
            1
        );
        private_nh.param(
            "wait_publish_waypoints_time",
            param.wait_publish_waypoints_time,
            static_cast<float>(5e-3)
        );
        private_nh.param(
            "goal_publish_frequency",
            param.goal_publish_frequency,
            0.5
        );

        waypoint_publisher
            = nh.advertise<waypoint_manager_msgs::Waypoint>(
                param.waypoint_topic,
                param.publish_queue_size,
                param.latch
              );
        waypoints_publisher
            = nh.advertise<waypoint_manager_msgs::Waypoints>(
                param.waypoints_topic,
                param.publish_queue_size,
                true
              );
        route_publisher
            = nh.advertise<waypoint_manager_msgs::Route>(
                param.route_topic,
                param.publish_queue_size,
                true
              );

        is_reached_goal_subscriber
            = nh.subscribe<std_msgs::Bool>(
                param.is_reached_goal_topic,
                param.subscribe_queue_size,
                &Node::isReachedGoal,
                this
              );
        regist_goal_pose_subscriber
            = nh.subscribe<geometry_msgs::PoseStamped>(
                param.regist_goal_pose_topic,
                param.subscribe_queue_size,
                &Node::registGoalPose,
                this
              );
        regist_goal_point_subscriber
            = nh.subscribe<geometry_msgs::PointStamped>(
                param.regist_goal_point_topic,
                param.subscribe_queue_size,
                &Node::registGoalPoint,
                this
              );
        erase_goal_subscriber
            = nh.subscribe<std_msgs::String>(
                param.erase_goal_topic,
                param.subscribe_queue_size,
                &Node::eraseGoal,
                this
              );
        update_goal_subscriber
            = nh.subscribe<waypoint_manager_msgs::WaypointStamped>(
                param.update_goal_topic,
                param.subscribe_queue_size,
                &Node::updateGoalPose,
                this
              );
        append_route_subscriber
            = nh.subscribe<std_msgs::String>(
                param.append_route_topic,
                param.subscribe_queue_size,
                &Node::appendRoute,
                this
              );
        erase_route_subscriber
            = nh.subscribe<std_msgs::String>(
                param.erase_route_topic,
                param.subscribe_queue_size,
                &Node::eraseRoute,
                this
              );

        save_service
            = private_nh.advertiseService(
                "save",
                &Node::save,
                this
              );
        save_waypoints_service
            = private_nh.advertiseService(
                "save_waypoints",
                &Node::saveWaypoints,
                this
              );
        save_route_service
            = private_nh.advertiseService(
                "save_route",
                &Node::saveRoute,
                this
              );
        reset_route_service
            = private_nh.advertiseService(
                "reset_route",
                &Node::resetRoute,
                this
              );
        switch_cancel_service
            = private_nh.advertiseService(
                "switch_cancel",
                &Node::switchCancel,
                this
              );
        next_waypoint_service
            = private_nh.advertiseService(
                "next_waypoint",
                &Node::nextWaypoint,
                this
              );

        is_cancel.store(true);
        regist_goal_id.store(0);

        ROS_INFO("Loading waypoints_file %s", param.waypoints_file.c_str());
        waypoint_map.load(param.waypoints_file);
        ROS_INFO("Success load waypoints_file");

        ROS_INFO("Loading route_file: %s", param.route_file.c_str());
        router.load(param.route_file);
        router.loop(param.enable_loop);
        ROS_INFO("Success load route_file");
        ROS_INFO("Count of skip ids %d", router.getSkipIds());
    }

    void Node::spin() {
        std::thread subscribers_thread{
            [this]() {
                ros::spin();
            }
        };

        ros::Rate rate{param.goal_publish_frequency};

        rate.sleep();
        publishLatchedData();

        while(ros::ok()) {
            publishGoal();
            rate.sleep();
        }
        ros::shutdown();
        subscribers_thread.join();
    }

    void Node::isReachedGoal(const std_msgs::Bool::ConstPtr &msg) {
        if(router.isEmpty()) {
            ROS_WARN("Route size of zero");
            return;
        }
        if(is_cancel.load()) {
            return;
        }
        if(!msg->data) {
            return;
        }
        ROS_INFO(
            "Goal reached %s from waypoint_server_node",
            router.getIndex().c_str()
        );
        if(waypoint_map[router.getIndex()].properties["stop"] == "true") {
            ROS_INFO("Current waypoint properties stop is true");
            ROS_INFO("Please call the ~/next_waypoint service");
            return;
        }
        if(!router.forwardIndex()) {
            ROS_WARN("Not found next index for route");
            is_cancel.store(true);
        }
        else {
            publishGoal();
        }
    }

    Map::Key generateKey(const std::atomic<unsigned int> &id, const std::string &prefix) {
        return prefix
               + std::to_string(id.load())
               + "_"
               + std::to_string(ros::Time::now().toNSec());
    }

    void Node::registGoalPose(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        const Map::Key name = generateKey(regist_goal_id, param.regist_waypoint_prefix);

        waypoint_map[name].goal.x() = msg->pose.position.x;
        waypoint_map[name].goal.y() = msg->pose.position.y;
        waypoint_map[name].goal.z() = msg->pose.position.z;
        waypoint_map[name].quaternion.x() = msg->pose.orientation.x;
        waypoint_map[name].quaternion.y() = msg->pose.orientation.y;
        waypoint_map[name].quaternion.z() = msg->pose.orientation.z;
        waypoint_map[name].quaternion.w() = msg->pose.orientation.w;
        waypoint_map.setQuaternion(name);

        ROS_INFO("Add waypoint %s from pose", name.c_str());

        regist_goal_id.store(regist_goal_id.load() + 1);

        publishLatchedData();
    }

    void Node::registGoalPoint(const geometry_msgs::PointStamped::ConstPtr &msg) {
        const Map::Key name = generateKey(regist_goal_id, param.regist_waypoint_prefix);

        waypoint_map[name].goal.x() = msg->point.x;
        waypoint_map[name].goal.y() = msg->point.y;
        waypoint_map[name].goal.z() = msg->point.z;
        waypoint_map[name].quaternion.x() = 0;
        waypoint_map[name].quaternion.y() = 0;
        waypoint_map[name].quaternion.z() = 0;
        waypoint_map[name].quaternion.w() = 1;
        waypoint_map.setQuaternion(name);

        ROS_INFO("Add waypoint %s from point", name.c_str());

        regist_goal_id.store(regist_goal_id.load() + 1);

        publishLatchedData();
    }

    void Node::eraseGoal(const std_msgs::String::ConstPtr &msg) {
        if(!waypoint_map.hasKey(msg->data)) {
            ROS_INFO("Do not have waypoint id %s", msg->data.c_str());
            return;
        }
        waypoint_map.erase(msg->data);
        router.erase(msg->data);

        ROS_INFO("Removed waypoint id %s", msg->data.c_str());

        publishLatchedData();
    }

    void Node::updateGoalPose(const waypoint_manager_msgs::WaypointStamped::ConstPtr &msg) {
        if(param.global_frame != msg->header.frame_id) {
            ROS_WARN("The frame_id is different, so the goal pose is not updated");
            return;
        }
        decltype(auto) name = msg->waypoint.identity;

        waypoint_map[name].goal.x() = msg->waypoint.pose.position.x;
        waypoint_map[name].goal.y() = msg->waypoint.pose.position.y;
        waypoint_map[name].goal.z() = msg->waypoint.pose.position.z;
        waypoint_map[name].quaternion.x() = msg->waypoint.pose.orientation.x;
        waypoint_map[name].quaternion.y() = msg->waypoint.pose.orientation.y;
        waypoint_map[name].quaternion.z() = msg->waypoint.pose.orientation.z;
        waypoint_map[name].quaternion.w() = msg->waypoint.pose.orientation.w;

        for(const auto &[key, value] : msg->waypoint.properties) {
            waypoint_map[name].properties[key] = value;
        }
        waypoint_map.setQuaternion(name);
        publishLatchedData();
    }

    void Node::appendRoute(const std_msgs::String::ConstPtr &msg) {
        router.append(msg->data);

        publishLatchedData();
    }

    void Node::eraseRoute(const std_msgs::String::ConstPtr &msg) {
        router.erase(msg->data);

        publishLatchedData();
    }
    
    bool Node::save(
        std_srvs::TriggerRequest &request,
        std_srvs::TriggerResponse &response
    ) {
        ROS_INFO("Called save()");
        waypoint_map.save(param.waypoints_file);
        router.save(param.route_file);
        return true;
    }

    bool Node::saveWaypoints(
        std_srvs::TriggerRequest &request,
        std_srvs::TriggerResponse &response
    ) {
        ROS_INFO("Called saveWaypoints()");
        waypoint_map.save(param.waypoints_file);
        return true;
    }

    bool Node::saveRoute(
        std_srvs::TriggerRequest &request,
        std_srvs::TriggerResponse &response
    ) {
        ROS_INFO("Called saveRoute()");
        router.save(param.route_file);
        return true;
    }

    bool Node::resetRoute(
        std_srvs::TriggerRequest &request,
        std_srvs::TriggerResponse &response
    ) {
        ROS_INFO("Called resetRoute()");

        if(router.isEmpty()) {
            ROS_WARN("Route size of zero");
            return false;
        }
        router.resetIndex();
        publishGoal();

        ROS_INFO("Reset of route current goal %s", router.getIndex().c_str());

        return true;
    };

    bool Node::switchCancel(
        std_srvs::TriggerRequest &request,
        std_srvs::TriggerResponse &response
    ) {
        ROS_INFO("Called switchCancel()");
        exchangeCancelState();
        return true;
    }

    bool Node::nextWaypoint(
        std_srvs::TriggerRequest &request,
        std_srvs::TriggerResponse &response
    ) {
        ROS_INFO("Called nextWaypoint()");

        if(!router.forwardIndex()) {
            ROS_WARN("Failed forward index for route");
        }
        publishGoal();

        return true;
    }

    void Node::publishGoal() {
        if(is_cancel.load()) {
            return;
        }
        if(router.isEmpty()) {
            return;
        }
        const auto pose_vector = waypoint_map[router.getIndex()].goal;
        const auto orientation = waypoint_map[router.getIndex()].quaternion;

        waypoint_manager_msgs::Waypoint waypoint;

        waypoint.identity = router.getIndex();
        waypoint.pose.position.x = pose_vector.x();
        waypoint.pose.position.y = pose_vector.y();
        waypoint.pose.position.z = pose_vector.z();
        waypoint.pose.orientation.x = orientation.x();
        waypoint.pose.orientation.y = orientation.y();
        waypoint.pose.orientation.z = orientation.z();
        waypoint.pose.orientation.w = orientation.w();

        for(const auto &[name, value] : waypoint_map[router.getIndex()].properties) {
            waypoint_manager_msgs::Property property;

            property.name = name;
            property.data = value;

            waypoint.properties.push_back(property);
        }

        waypoint_publisher.publish(waypoint);
    }

    void Node::publishWaypoints() {
        waypoint_manager_msgs::Waypoints waypoints_msg;

        for(const auto &[key, waypoint] : waypoint_map.data()) {
            waypoint_manager_msgs::Waypoint waypoint_msg;

            waypoint_msg.identity = key;
            waypoint_msg.pose.position.x = waypoint.goal.x();
            waypoint_msg.pose.position.y = waypoint.goal.y();
            waypoint_msg.pose.position.z = waypoint.goal.z();
            waypoint_msg.pose.orientation.x = waypoint.quaternion.x();
            waypoint_msg.pose.orientation.y = waypoint.quaternion.y();
            waypoint_msg.pose.orientation.z = waypoint.quaternion.z();
            waypoint_msg.pose.orientation.w = waypoint.quaternion.w();

            for(const auto &[name, data] : waypoint.properties) {
                waypoint_manager_msgs::Property property;

                property.name = name;
                property.data = data;

                waypoint_msg.properties.push_back(property);
            }
            waypoints_msg.waypoints.push_back(waypoint_msg);
        }
        waypoints_msg.info.header.frame_id = param.global_frame;
        waypoints_msg.info.header.stamp = ros::Time::now();

        waypoints_publisher.publish(waypoints_msg);
    }

    void Node::publishRoute() {
        waypoint_manager_msgs::Route route_msg;

        for(const auto &id : router.data()) {
            route_msg.identities.push_back(id);
        }
        route_msg.header.frame_id = param.global_frame;
        route_msg.header.stamp = ros::Time::now();

        route_publisher.publish(route_msg);
    }

    void Node::publishLatchedData() {
        publishRoute();
        ros::Duration(param.wait_publish_waypoints_time).sleep();
        publishWaypoints();
    }

    void Node::exchangeCancelState() {
        is_cancel.store(!is_cancel.load());

        if(is_cancel.load()) {
            ROS_INFO("Changed is_cancel true");
        }
        else {
            ROS_INFO("Changed is_cancel false");
        }
    }
}

auto main(int argc, char **argv) -> int {
    ros::init(argc, argv, "waypoint_server_node");

    ROS_INFO("Start waypoint_server_node");

    waypoint_server::Node node{};
    node.spin();

    ROS_INFO("Finish waypoint_server_node");
    return 0;
}
