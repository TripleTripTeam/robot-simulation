//
// Created by vadim on 17.04.2021.
//

#include "velodyne_sensor_plugin.h"

#ifdef SENSOR_PRINT_CUSTOM_INFO
#define PRINT_CUSTOM_INFO(msg) do { std::cout << "[CUSTOM_INFO]: " << (msg) << std::endl; } while(0)
#else
#define PRINT_CUSTOM_INFO(msg)
#endif

namespace gazebo {
    GZ_REGISTER_SENSOR_PLUGIN(Velodyne_sensor_plugin)

    ////////////////////////////////////////
    ////    Private members
    ////////////////////////////////////////

    void Velodyne_sensor_plugin::PubThread() {
        ros::Rate ros_sleep(100);
        while (this->rosNode->ok() && subs_count) {
            PRINT_CUSTOM_INFO(subs_count);
            ros_sleep.sleep();
        }
    }

    void Velodyne_sensor_plugin::LidarTopicConnected() {
        subs_count++;
        PRINT_CUSTOM_INFO("VELODYNE SENSOR LIDAR DATA TOPIC CONNECTED");
    }

    void Velodyne_sensor_plugin::LidarTopicDisconnected() {
        subs_count--;
        PRINT_CUSTOM_INFO("VELODYNE SENSOR LIDAR DATA TOPIC DISCONNECTED");
    }

    void Velodyne_sensor_plugin::OnUpdate(sensors::RaySensorPtr _sensor) {
        std_msgs::Float32MultiArray msg;
        for (int i = 0; i < _sensor->RangeCount(); ++i)
            msg.data.push_back(_sensor->Range(i));
        rosPub.publish(msg);
    }

    ////////////////////////////////////////
    ////    Public members
    ////////////////////////////////////////

    Velodyne_sensor_plugin::Velodyne_sensor_plugin() {
        PRINT_CUSTOM_INFO("VELODYNE SENSOR CTOR BEGIN");
        _topic_lidar_data_name = "/velodyne/lidar_data";
        PRINT_CUSTOM_INFO("VELODYNE SENSOR CTOR END");
    }

    Velodyne_sensor_plugin::~Velodyne_sensor_plugin() {
        PRINT_CUSTOM_INFO("VELODYNE SENSOR DTOR BEGIN");
        // nothing
        PRINT_CUSTOM_INFO("VELODYNE SENSOR DTOR END");
    }

    void Velodyne_sensor_plugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
        PRINT_CUSTOM_INFO("VELODYNE SENSOR LOAD BEGIN");
        sensor = std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
        _sensor->Load(_sensor->WorldName(), _sdf);

        subs_count = 0;

        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client",
                      ros::init_options::NoSigintHandler);
        }
        this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

        ros::AdvertiseOptions ao =
                ros::AdvertiseOptions::create<std_msgs::Float32MultiArray>(
                        _topic_lidar_data_name + "_ros",
                        10,
                        boost::bind(&Velodyne_sensor_plugin::LidarTopicConnected, this),
                        boost::bind(&Velodyne_sensor_plugin::LidarTopicDisconnected, this),
                        ros::VoidPtr(), NULL);
        this->rosPub = this->rosNode->advertise(ao);

        this->connection = this->sensor->ConnectUpdated(
                std::bind(&Velodyne_sensor_plugin::OnUpdate, this, this->sensor));
        PRINT_CUSTOM_INFO("VELODYNE SENSOR LOAD END");
    }
}