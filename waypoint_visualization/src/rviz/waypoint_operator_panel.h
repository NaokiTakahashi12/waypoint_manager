
#pragma once

#include <QWidget>
#include <QHBoxLayout>
#include <QPushButton>

#include <ros/ros.h>
#include <rviz/panel.h>

namespace waypoint_visualization {
    class WaypointOperatorPanel : public rviz::Panel {
        Q_OBJECT

        public :
            WaypointOperatorPanel(QWidget *parent = 0);
            virtual ~WaypointOperatorPanel();

        protected Q_SLOTS :
            void callSwitchCancel();
            void callNextWaypoint();

        protected :
            QHBoxLayout *hbox_layout;

            QPushButton *switch_cancel_button,
                        *next_waypoint_button;

            ros::NodeHandle nh,
                            private_nh;

            ros::ServiceClient switch_cancel_client,
                               next_waypoint_client;
    };
}

