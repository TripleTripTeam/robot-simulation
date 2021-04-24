//
// Created by vadim on 17.04.2021.
//

#include "velodyne_model_plugin.h"

#ifdef MODEL_PRINT_CUSTOM_INFO
#define PRINT_CUSTOM_INFO(msg) do { std::cout << "[CUSTOM_INFO]: " << (msg) << std::endl; } while(0)
#else
#define PRINT_CUSTOM_INFO(msg)
#endif

namespace gazebo {
    GZ_REGISTER_MODEL_PLUGIN(Velodyne_model_plugin)

    ////////////////////////////////////////
    ////    Private members
    ////////////////////////////////////////

    void Velodyne_model_plugin::OnMsg(ConstVector3dPtr &_msg) {
        this->SetVelocity(_msg->x());
    }

    void Velodyne_model_plugin::QueueThread() {
        static const double timeout = 0.01;
        while (this->rosNode->ok()) {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
    }

    void Velodyne_model_plugin::PubThread() {
        std_msgs::Float32 msg;
        ros::Rate ros_sleep(100);
        while (this->rosNode->ok()) {
            msg.data = GetCurrentAngle();
            this->rosPub.publish(msg);
            ros_sleep.sleep();
        }
    }

    void Velodyne_model_plugin::PosTopicConnected() {
        pub_thread = std::thread(std::bind(&Velodyne_model_plugin::PubThread, this));
        PRINT_CUSTOM_INFO("VELODYNE MODEL POSITION TOPIC CONNECTED");
    }

    void Velodyne_model_plugin::PosTopicDisconnected() {
        pub_thread.detach();
        PRINT_CUSTOM_INFO("VELODYNE MODEL POSITION TOPIC DISCONNECTED");
    }

    ////////////////////////////////////////
    ////    Public members
    ////////////////////////////////////////

    Velodyne_model_plugin::Velodyne_model_plugin() {
        PRINT_CUSTOM_INFO("VELODYNE MODEL CTOR BEGIN");
        _topic_rotate_angle_msg_name = "/velodyne/angle";
        _topic_rotate_speed_msg_name = "/velodyne/speed";
        PRINT_CUSTOM_INFO("VELODYNE MODEL CTOR END");
    }

    Velodyne_model_plugin::~Velodyne_model_plugin() {
        PRINT_CUSTOM_INFO("VELODYNE MODEL DTOR BEGIN");
        // notring
        PRINT_CUSTOM_INFO("VELODYNE MODEL DTOR END");
    }

    void Velodyne_model_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        PRINT_CUSTOM_INFO("VELODYNE MODEL LOAD BEGIN");

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
        if (_sdf->HasElement("rotate_speed")) {
            velocity = _sdf->Get<double>("rotate_speed");
        }
        this->SetVelocity(velocity);

        this->node = transport::NodePtr(new transport::Node());
        this->node->Init(this->model->GetWorld()->Name());

        std::string topicRotSpeedName = "~/" + this->model->GetName() + _topic_rotate_speed_msg_name + "_gazebo";
        std::string topicAngleName = "~/" + this->model->GetName() + _topic_rotate_angle_msg_name + "_gazebo";

        this->sub = this->node->Subscribe(topicRotSpeedName,
                                          &Velodyne_model_plugin::OnMsg, this);

        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client",
                      ros::init_options::NoSigintHandler);
        }
        this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

        ros::SubscribeOptions so =
                ros::SubscribeOptions::create<std_msgs::Float32>(
                        "/" + this->model->GetName() + _topic_rotate_speed_msg_name + "_ros",
                        1,
                        boost::bind(&Velodyne_model_plugin::OnRosMsg, this, _1),
                        ros::VoidPtr(), &this->rosQueue);
        this->rosSub = this->rosNode->subscribe(so);

        ros::AdvertiseOptions ao =
                ros::AdvertiseOptions::create<std_msgs::Float32>(
                        "/" + this->model->GetName() + _topic_rotate_angle_msg_name + "_ros",
                        10,
                        boost::bind(&Velodyne_model_plugin::PosTopicConnected, this),
                        boost::bind(&Velodyne_model_plugin::PosTopicDisconnected, this),
                        ros::VoidPtr(), NULL);
        this->rosPub = this->rosNode->advertise(ao);
        pub_queue_ = pmq.addPub<std_msgs::Float32>();

        this->rosQueueThread =
                std::thread(std::bind(&Velodyne_model_plugin::QueueThread, this));

        PRINT_CUSTOM_INFO("VELODYNE MODEL LOAD END");
    }

    void Velodyne_model_plugin::OnRosMsg(const std_msgs::Float32ConstPtr &_msg) {
        this->SetVelocity(_msg->data);
    }

    float Velodyne_model_plugin::GetCurrentAngle() {
        return this->model->GetLink("top_lidar")->WorldPose().Yaw() + M_PIf32;
    }

    void Velodyne_model_plugin::SetVelocity(const double &_vel) {
        this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), _vel);
    }
}