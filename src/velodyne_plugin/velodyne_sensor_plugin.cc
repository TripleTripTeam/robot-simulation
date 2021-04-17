#ifndef PUB_HH
#define PUB_HH

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/Events.hh>
//#include "gazebo/math/gzmath.hh"
//#include <gazebo/msgs/.
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <std_msgs/Float32.h>
#include "ros/callback_queue.h"
#include <iostream>

namespace gazebo {
    class /*GAZEBO_VISIBLE*/ distPublisher : public SensorPlugin {
    public:
        distPublisher() {
//            publisher = nh.advertise<sensor_msgs::LaserScan>("/dist_publish/distance", 1);
//            std::string topic_name = "/turtlebot3_burger/velodyne_model/rotate_angle_ros";
//            ros::SubscribeOptions so =
//                    ros::SubscribeOptions::create<std_msgs::Float32>(
//                            topic_name,
//                            1000,
//                            boost::bind(&distPublisher::SubCallback, this, _1),
//                            ros::VoidPtr(), &this->rosQueue);
//
//            this->subscriber = nh.subscribe(so);
//
//            this->rosQueueThread =
//                    std::thread(std::bind(&distPublisher::QueueThread, this));
//            std::cout << ("sensor plugin started") << std::endl;
        }

    public:
        virtual ~distPublisher() {
//            this->parentSensor->DisconnectUpdated(this->connection);
            this->parentSensor.reset();
        }

    public:
        virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
            std::cout << ("sensor plugin loading") << std::endl;
            publisher = nh.advertise<sensor_msgs::LaserScan>("/dist_publish/distance", 1);
            this->parentSensor = std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
            this->connection = this->parentSensor->ConnectUpdated(
                    std::bind(&distPublisher::OnUpdate, this, this->parentSensor));

            std::string topic_name = "/turtlebot3_burger/velodyne_model/rotate_angle_ros";
            ros::SubscribeOptions so =
                    ros::SubscribeOptions::create<std_msgs::Float32>(
                            topic_name,
                            10,
                            boost::bind(&distPublisher::SubCallback, this, _1),
                            ros::VoidPtr(), &this->rosQueue);

            this->subscriber = nh.subscribe(so);

            this->rosQueueThread =
                    std::thread(std::bind(&distPublisher::QueueThread, this));
            std::cout << ("sensor plugin load end") << std::endl;
        }

    private:
        void SubCallback(const std_msgs::Float32ConstPtr &_msg) {
            std::cout << ("Pos: ") << _msg->data << std::endl;
            return;
        }

    private:
        void QueueThread() {
            static const double timeout = 0.01;
            while (this->nh.ok()) {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

    protected:
        virtual void OnUpdate(sensors::RaySensorPtr _sensor) {
            std::cout << ("Update\n") << std::endl;
            std::vector<double> dists_dbl;
            std::vector<float> dists_flt;
            _sensor->Ranges(dists_dbl);
            sensor_msgs::LaserScan msg;
            for (int i = 0; i < dists_dbl.size(); ++i)
                dists_flt.push_back(dists_dbl[i]);
            msg.ranges = dists_flt;
            publisher.publish(msg);
        }

    protected:
        sensors::RaySensorPtr parentSensor;
    private:
        event::ConnectionPtr connection;
        ros::NodeHandle nh;
        ros::CallbackQueue rosQueue;
        std::thread rosQueueThread;
        ros::Publisher publisher;
        ros::Subscriber subscriber;
    };

    GZ_REGISTER_SENSOR_PLUGIN(distPublisher)
}
#endif