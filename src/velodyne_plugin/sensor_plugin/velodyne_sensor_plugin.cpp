//
// Created by vadim on 17.04.2021.
//

#include "velodyne_sensor_plugin.h"
#include "std_msgs/Float32.h"

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
//        ros::Rate ros_sleep(100);
//        while (this->rosNode->ok() && subs_count) {
//            PRINT_CUSTOM_INFO(subs_count);
//            ros_sleep.sleep();
//        }
    }

    void Velodyne_sensor_plugin::LidarTopicConnected() {
        this->subs_count++;
        PRINT_CUSTOM_INFO("VELODYNE SENSOR LIDAR DATA TOPIC CONNECTED");
    }

    void Velodyne_sensor_plugin::LidarTopicDisconnected() {
        this->subs_count--;
        PRINT_CUSTOM_INFO("VELODYNE SENSOR LIDAR DATA TOPIC DISCONNECTED");
    }

    void Velodyne_sensor_plugin::CheckInitConnected() {
        ros::Duration(1).sleep();
        std::string mfsm_topic = std::string("/turtlebot3_burger/velodyne/angle_ros");
        std::string mfss_topic = std::string("/velodyne/_tmpld_ros");
        this->mfsm.subscribe(*this->rosNode.get(), mfsm_topic, 1);
        this->mfss.subscribe(*this->rosNode.get(), mfss_topic, 1);
        this->sync_.reset(new Sync(MySyncPolicy(10), this->mfsm, this->mfss));
        this->sync_->registerCallback(boost::bind(&Velodyne_sensor_plugin::SyncFunc, this, _1, _2));
        PRINT_CUSTOM_INFO("VELODYNE SENSOR MODEL INITIALIZED CONNECTED");
    }

    void Velodyne_sensor_plugin::CheckInitDisconnect() {
        PRINT_CUSTOM_INFO("VELODYNE SENSOR EXIT APP CONNECTED");
    }

    void Velodyne_sensor_plugin::OnUpdate(sensors::RaySensorPtr _sensor) {
        msg_generatorstd_msgs::VelodyneSensor msg;
        msg.header.stamp = ros::Time().now();
        _sensor->SetActive(false);
        for (int i = 0; i < _sensor->RangeCount(); ++i) {
            msg.dist.push_back(_sensor->Range(i));
        }
        _sensor->SetActive(true);
        rosPubInternal.publish(msg);
    }

    void Velodyne_sensor_plugin::SyncFunc(const msg_generatorstd_msgs::VelodyneModelConstPtr &model,
                                          const msg_generatorstd_msgs::VelodyneSensorConstPtr &sensor) {
        msg_generatorstd_msgs::VelodyneSensor msg;
        msg.header.stamp = sensor->header.stamp;
        msg.angle = model->angle;
        msg.dist = std::move(sensor->dist);
        this->rosPubExternal.publish(msg);
    }

    ////////////////////////////////////////
    ////    Public members
    ////////////////////////////////////////

    Velodyne_sensor_plugin::Velodyne_sensor_plugin() {
        PRINT_CUSTOM_INFO("VELODYNE SENSOR CTOR BEGIN");
        _topic_lidar_data_name = "/velodyne/_tmpld";
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

        this->subs_count = 0;

        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client",
                      ros::init_options::NoSigintHandler);
        }
        this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

        ros::AdvertiseOptions ao =
                ros::AdvertiseOptions::create<msg_generatorstd_msgs::VelodyneSensor>(
                        _topic_lidar_data_name + "_ros",
                        10,
                        boost::bind(&Velodyne_sensor_plugin::LidarTopicConnected, this),
                        boost::bind(&Velodyne_sensor_plugin::LidarTopicDisconnected, this),
                        ros::VoidPtr(), NULL);
        this->rosPubInternal = this->rosNode->advertise(ao);

        ros::AdvertiseOptions ao_ext =
                ros::AdvertiseOptions::create<msg_generatorstd_msgs::VelodyneSensor>(
                        "/velodyne/lidar_data",
                        10,
                        boost::bind(&Velodyne_sensor_plugin::LidarTopicConnected, this),
                        boost::bind(&Velodyne_sensor_plugin::LidarTopicDisconnected, this),
                        ros::VoidPtr(), NULL);
        this->rosPubExternal = this->rosNode->advertise(ao_ext);

        ros::AdvertiseOptions ao_tmp =
                ros::AdvertiseOptions::create<std_msgs::Float32>(
                        "check_initial_topic_ros",
                        10,
                        boost::bind(&Velodyne_sensor_plugin::CheckInitConnected, this),
                        boost::bind(&Velodyne_sensor_plugin::CheckInitDisconnect, this),
                        ros::VoidPtr(), NULL);
        this->rosPubCheckInitialization = this->rosNode->advertise(ao_tmp);

        this->connection = this->sensor->ConnectUpdated(
                std::bind(&Velodyne_sensor_plugin::OnUpdate, this, this->sensor));
        PRINT_CUSTOM_INFO("VELODYNE SENSOR LOAD END");
    }
}