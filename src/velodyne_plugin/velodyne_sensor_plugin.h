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

#include "std_msgs/Float32.h"

using namespace gazebo::sensors;

namespace gazebo {

    class Velodyne_sensor_plugin : public SensorPlugin {

    public:

        Velodyne_sensor_plugin();

        ~Velodyne_sensor_plugin();

        void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

    private:
        sdf::ElementPtr sdf;
    };

}

#endif //SRC_VELODYNE_SENSOR_PLUGIN_H
