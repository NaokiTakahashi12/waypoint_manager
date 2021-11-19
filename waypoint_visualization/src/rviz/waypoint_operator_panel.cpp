
#include <QWidget>
#include <QPushButton>

#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_srvs/Trigger.h>

#include "waypoint_operator_panel.h"

namespace waypoint_visualization {
    WaypointOperatorPanel::WaypointOperatorPanel(QWidget *parent)
        : rviz::Panel(parent),
          nh(),
          private_nh("~") {
        hbox_layout = new QHBoxLayout();

        switch_cancel_button = new QPushButton(this);
        switch_cancel_button->setText("Switch Cancel");
        connect(switch_cancel_button, SIGNAL(clicked()), this, SLOT(callSwitchCancel()));
        switch_cancel_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        next_waypoint_button = new QPushButton(this);
        next_waypoint_button->setText("Next Waypoint");
        connect(next_waypoint_button, SIGNAL(clicked()), this, SLOT(callNextWaypoint()));
        next_waypoint_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        hbox_layout->addWidget(switch_cancel_button);
        hbox_layout->addWidget(next_waypoint_button);

        setLayout(hbox_layout);

        switch_cancel_client = nh.serviceClient<std_srvs::Trigger>("waypoint_manager/waypoint_server/switch_cancel");
        next_waypoint_client = nh.serviceClient<std_srvs::Trigger>("waypoint_manager/waypoint_server/next_waypoint");
    }

    WaypointOperatorPanel::~WaypointOperatorPanel() {
    }

    void WaypointOperatorPanel::callSwitchCancel() {
        ROS_INFO("Pushed callSwitchCancel()");
        std_srvs::Trigger trigger;
        switch_cancel_client.call(trigger);
    }

    void WaypointOperatorPanel::callNextWaypoint() {
        ROS_INFO("Pushed callNextWaypoint()");
        std_srvs::Trigger trigger;
        next_waypoint_client.call(trigger);
    }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
    waypoint_visualization::WaypointOperatorPanel,
    rviz::Panel
)
