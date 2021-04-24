//
// Created by vadim on 17.04.2021.
//

#ifndef VELODYNE_MODEL_PLUGIN_H
#define VELODYNE_MODEL_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo_plugins/PubQueue.h>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "ros/publisher.h"
#include "ros/advertise_options.h"
#include "ros/transport_publisher_link.h"

#include "std_msgs/Float32.h"
#include "../msg/VelodyneModel.h"

namespace gazebo {

    class Velodyne_model_plugin : public ModelPlugin {

    public:

        Velodyne_model_plugin();

        ~Velodyne_model_plugin();

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

        /// \brief The function witch get angle of top_lidar model
        float GetCurrentAngle();

        /// \brief Set the velocity of the Velodyne
        /// \param[in] _vel New target velocity
        void SetVelocity(const double &_vel);

        /// \brief Handle an incoming message from ROS
        /// \param[in] _msg A float value that is used to set the velocity
        /// of the Velodyne.
        void OnRosMsg(const std_msgs::Float32ConstPtr &_msg);

    private:
        std::unique_ptr<ros::NodeHandle> rosNode; /// \brief A node use for ROS transport
        ros::Subscriber rosSub; /// \brief A ROS subscriber
        ros::Publisher rosPub; /// \brief A ROS publisher
        ros::CallbackQueue rosQueue; /// \brief A ROS callbackqueue that helps process messages
        std::thread rosQueueThread; /// \brief A thread the keeps running the rosQueue

        transport::NodePtr node; /// \brief A node used for transport
        transport::SubscriberPtr sub; /// \brief A subscriber to a named topic.

        PubQueue<msg_generatorstd_msgs::VelodyneModel>::Ptr pub_queue_;
        std::thread pub_thread;
        PubMultiQueue pmq;

        physics::ModelPtr model; /// \brief Pointer to the model.
        physics::JointPtr joint; /// \brief Pointer to the joint.
        common::PID pid; /// \brief A PID controller for the joint.

        std::string _topic_rotate_angle_msg_name;
        std::string _topic_rotate_speed_msg_name;

        /// \brief Handle incoming message
        /// \param[in] _msg Repurpose a vector3 message. This function will
        /// only use the x component.
        void OnMsg(ConstVector3dPtr &_msg);

        /// \brief ROS helper function that processes messages
        void QueueThread();

        /// \brief Publisher thread function
        void PubThread();

        /// \brief Publisher option connect function
        void PosTopicConnected();

        /// \brief Publisher option disconnect function
        void PosTopicDisconnected();
    };
}

#endif  // VELODYNE_MODEL_PLUGIN_H
