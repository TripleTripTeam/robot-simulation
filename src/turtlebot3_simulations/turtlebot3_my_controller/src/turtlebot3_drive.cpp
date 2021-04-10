/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include "turtlebot3_my_controller/turtlebot3_drive.h"
#include "serverHandler/serverHandler.h"
#include <algorithm>

Turtlebot3Drive::Turtlebot3Drive()
        : nh_priv_("~") {
    //Init gazebo ros turtlebot3 node
    ROS_INFO("TurtleBot3 Simulation Node Init");
    auto ret = init();
    ROS_ASSERT(ret);
}

Turtlebot3Drive::~Turtlebot3Drive() {
    updatecommandVelocity(0.0, 0.0);
    ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool Turtlebot3Drive::init() {
    SCL::init();
    // initialize ROS parameter
    std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");
    // initialize variables
    escape_range_ = 30.0 * DEG2RAD;
    check_forward_dist_ = 0.7;
    check_side_dist_ = 0.6;

    tb3_pose_ = 0.0;
    prev_tb3_pose_ = 0.0;

    // initialize publishers
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

    // initialize subscribers
    laser_scan_sub_ = nh_.subscribe("scan", 10, &Turtlebot3Drive::laserScanMsgCallBack, this);
    odom_sub_ = nh_.subscribe("odom", 10, &Turtlebot3Drive::odomMsgCallBack, this);
    camera_image_sub_ = nh_.subscribe("camera/image", 10, &Turtlebot3Drive::cameraImageCallBack, this);

    return true;
}

void Turtlebot3Drive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg) {
    double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                         msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                               msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    _x_pos = msg->pose.pose.position.x;
    _y_pos = msg->pose.pose.position.y;
    _z_pos = msg->pose.pose.position.z;
    tb3_pose_ = atan2(siny, cosy);
}

#include <iostream>

void Turtlebot3Drive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg) {
    std::copy(msg->ranges.begin(), msg->ranges.end(), _lidar_data.begin());
}

void Turtlebot3Drive::cameraImageCallBack(const sensor_msgs::Image::ConstPtr &msg) {
//    std::cout << msg->data.size() << std::endl;
}

void Turtlebot3Drive::updatecommandVelocity(double linear, double angular) {
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = linear;
    cmd_vel.angular.z = angular;

    cmd_vel_pub_.publish(cmd_vel);
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool Turtlebot3Drive::controlLoop() {
#ifdef TURTLEBOT3_SERVER_CONNECTED
    auto vel_com = serverHandler::getControlVector(_last_com_vector);
    updatecommandVelocity(vel_com.first, vel_com.second);
    _last_com_vector = vel_com;
    serverHandler::sendCarTelemetry(_x_pos, _y_pos, _z_pos, _lidar_data);
#endif
    return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "turtlebot3_drive");
    Turtlebot3Drive turtlebot3_drive;
    ros::Rate loop_rate(125);
    serverHandler::init();
    while (ros::ok()) {
        turtlebot3_drive.controlLoop();
        ros::spinOnce();
        loop_rate.sleep();
    }
    serverHandler::deinit();
    return 0;
}
