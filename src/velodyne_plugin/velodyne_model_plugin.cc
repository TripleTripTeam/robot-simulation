#ifndef _VELODYNE_MODEL_PLUGIN_HH_
#define _VELODYNE_MODEL_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <thread>
#include <gazebo_plugins/PubQueue.h>
#include <cmath>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "ros/publisher.h"
#include "ros/advertise_options.h"
#include "ros/transport_publisher_link.h"
#include "std_msgs/Float32.h"
#include <iostream>
namespace gazebo {
    class VelodyneModulePlugin : public ModelPlugin {
    public:
        VelodyneModulePlugin() {}

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
    public:
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
            std::cout << ("model plugin loading\n");
            if (_model->GetJointCount() == 0) {
                std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
                return;
            }

            this->model = _model;

            this->joint = _model->GetJoints()[0];

            this->pid = common::PID(0.1, 0, 0);

            this->model->GetJointController()->SetVelocityPID(
                    this->joint->GetScopedName(), this->pid);

            double velocity = 0;

            if (_sdf->HasElement("rotate_speed"))
                velocity = _sdf->Get<double>("rotate_speed");

            this->SetVelocity(velocity);

            this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION < 8
            this->node->Init(this->model->GetWorld()->GetName());
#else
            this->node->Init(this->model->GetWorld()->Name());
#endif

            std::string topicRotSpeedName = "~/" + this->model->GetName() + "/velodyne_model/rotate_speed_gazebo";
            std::string topicAngleName = "~/" + this->model->GetName() + "/velodyne_model/rotate_angle_gazebo";

            this->sub = this->node->Subscribe(topicRotSpeedName,
                                              &VelodyneModulePlugin::OnMsg, this);

            this->pub = this->node->Advertise<gazebo::msgs::PoseStamped>(topicAngleName, 1, 1);

            if (!ros::isInitialized()) {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client",
                          ros::init_options::NoSigintHandler);
            }

            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            ros::SubscribeOptions so =
                    ros::SubscribeOptions::create<std_msgs::Float32>(
                            "/" + this->model->GetName() + "/velodyne_model/rotate_speed_ros",
                            1,
                            boost::bind(&VelodyneModulePlugin::OnRosMsg, this, _1),
                            ros::VoidPtr(), &this->rosQueue);

            this->rosSub = this->rosNode->subscribe(so);

            ros::AdvertiseOptions ao =
                    ros::AdvertiseOptions::create<std_msgs::Float32>(
                            "/" + this->model->GetName() + "/velodyne_model/rotate_angle_ros",
                            10,
                            boost::bind(&VelodyneModulePlugin::PosTopicConnected, this),
                            boost::bind(&VelodyneModulePlugin::PosTopicDisconnected, this),
                            ros::VoidPtr(), NULL);

            this->rosPub = this->rosNode->advertise(ao);
            pub_queue_ = pmq.addPub<std_msgs::Float32>();

            this->rosQueueThread =
                    std::thread(std::bind(&VelodyneModulePlugin::QueueThread, this));

            std::cout << ("model plugin load end\n");

        }

    public:
        float GetCurrentAngle() {
            return this->model->GetLink("top_lidar")->WorldPose().Yaw() + M_PIf32;
        }

        /// \brief Set the velocity of the Velodyne
        /// \param[in] _vel New target velocity
    public:
        void SetVelocity(const double &_vel) {
            this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), _vel);
        }

        /// \brief Handle incoming message
        /// \param[in] _msg Repurpose a vector3 message. This function will
        /// only use the x component.
    private:
        void OnMsg(ConstVector3dPtr &_msg) {
            this->SetVelocity(_msg->x());
        }

        /// \brief Handle an incoming message from ROS
        /// \param[in] _msg A float value that is used to set the velocity
        /// of the Velodyne.
    public:
        void OnRosMsg(const std_msgs::Float32ConstPtr &_msg) {
            this->SetVelocity(_msg->data);
        }

        /// \brief ROS helper function that processes messages
    private:
        void QueueThread() {
            static const double timeout = 0.01;
            while (this->rosNode->ok()) {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

    private:
        void PubThread() {
            std_msgs::Float32 msg;
            ros::Rate ros_sleep(100);
            while (this->rosNode->ok()) {
                msg.data = GetCurrentAngle();
                this->rosPub.publish(msg);
                ros_sleep.sleep();
            }
        }

    private:
        void PosTopicConnected() {
            pub_thread = std::thread(std::bind(&VelodyneModulePlugin::PubThread, this));
            ROS_INFO("connected");
        }

    private:
        void PosTopicDisconnected() {
            pub_thread.detach();
        }

        /// \brief A node use for ROS transport
    private:
        std::unique_ptr<ros::NodeHandle> rosNode;

        /// \brief A ROS subscriber
    private:
        ros::Subscriber rosSub;

    private:
        ros::Publisher rosPub;

        /// \brief A ROS callbackqueue that helps process messages
    private:
        ros::CallbackQueue rosQueue;

        /// \brief A thread the keeps running the rosQueue
    private:
        std::thread rosQueueThread;

        /// \brief A node used for transport
    private:
        transport::NodePtr node;

        /// \brief A subscriber to a named topic.
    private:
        transport::SubscriberPtr sub;

    private:
        transport::PublisherPtr pub;

    private:
        PubQueue<std_msgs::Float32>::Ptr pub_queue_;

    private:
        std::thread pub_thread;

    private:
        PubMultiQueue pmq;

        /// \brief Pointer to the model.
    private:
        physics::ModelPtr model;

        /// \brief Pointer to the joint.
    private:
        physics::JointPtr joint;

        /// \brief A PID controller for the joint.
    private:
        common::PID pid;
    };

    GZ_REGISTER_MODEL_PLUGIN(VelodyneModulePlugin)
}
#endif
