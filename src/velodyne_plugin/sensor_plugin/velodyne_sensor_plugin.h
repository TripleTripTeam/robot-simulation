//
// Created by vadim on 17.04.2021.
//

#ifndef SRC_VELODYNE_SENSOR_PLUGIN_H
#define SRC_VELODYNE_SENSOR_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo_plugins/PubQueue.h>
#include <gazebo/plugins/GpuRayPlugin.hh>
#include <gazebo/common/Plugin.hh>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "ros/publisher.h"
#include "ros/advertise_options.h"
#include "ros/transport_publisher_link.h"

#include "std_msgs/Float32MultiArray.h"
#include "../msg/VelodyneSensor.h"
#include "../msg/VelodyneModel.h"

using namespace gazebo::sensors;

namespace gazebo {

    class Velodyne_sensor_plugin : public SensorPlugin {

    public:

        Velodyne_sensor_plugin();

        ~Velodyne_sensor_plugin();

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _sensor A pointer to the sensor that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

        void OnUpdate(sensors::RaySensorPtr _sensor);

    private:
        sensors::RaySensorPtr sensor; /// \brief Pointer to the sensor.

        std::unique_ptr<ros::NodeHandle> rosNode; /// \brief A node use for ROS transport
        ros::Subscriber rosSub; /// \brief A ROS subscriber
        ros::Publisher rosPub; /// \brief A ROS publisher

        event::ConnectionPtr connection;

        PubQueue<std_msgs::Float32MultiArray>::Ptr pub_queue_;
        std::thread pub_thread;
        PubMultiQueue pmq;

        std::string _topic_lidar_data_name;

        int subs_count;

        /// \brief Publisher thread function
        void PubThread();

        /// \brief Publisher option connect function
        void LidarTopicConnected();

        /// \brief Publisher option disconnect function
        void LidarTopicDisconnected();
    };

}

#endif //SRC_VELODYNE_SENSOR_PLUGIN_H
