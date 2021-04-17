//
// Created by vadim on 17.04.2021.
//

#include "velodyne_sensor_plugin.h"

#ifdef SENSOR_PRINT_CUSTOM_INFO
#define PRINT_CUSTOM_INFO(msg) do { std::cout << "[CUSTOM_INFO]: " << (msg) << std::endl; } while(0);
#else
#define PRINT_CUSTOM_INFO(msg)
#endif

namespace gazebo {
    GZ_REGISTER_SENSOR_PLUGIN(Velodyne_sensor_plugin)

    ////////////////////////////////////////
    ////    Private members
    ////////////////////////////////////////



    ////////////////////////////////////////
    ////    Public members
    ////////////////////////////////////////

    Velodyne_sensor_plugin::Velodyne_sensor_plugin() {
        PRINT_CUSTOM_INFO("VELODYNE SENSOR CTOR BEGIN");
        // nothing
        PRINT_CUSTOM_INFO("VELODYNE SENSOR CTOR END");
    }

    Velodyne_sensor_plugin::~Velodyne_sensor_plugin() {
        PRINT_CUSTOM_INFO("VELODYNE SENSOR DTOR BEGIN");
        // nothing
        PRINT_CUSTOM_INFO("VELODYNE SENSOR DTOR END");
    }

    void Velodyne_sensor_plugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
        PRINT_CUSTOM_INFO("VELODYNE SENSOR LOAD BEGIN");
        std::string str;
        std::cout << _sdf->ToString(str) << std::endl;
        std::cout << _sdf->HasElement("pose") << std::endl;
//        std::cout << _sdf->GetElementImpl("pose")->GetValue()->GetAsString() << std::endl;
//        std::cout << _sdf->GetElementImpl("ray")->GetElementImpl("noise")->GetElementImpl("stddev")->GetValue()->GetAsString() << std::endl;
//        std::cout << _sdf->GetElementImpl("ray")->GetElementImpl("scan")->GetElementImpl("horizontal")->GetElementImpl("max_angl")->GetValue()->GetTypeName() << std::endl;
//        std::cout << _sdf->GetElementImpl("ray")->GetElementImpl("scan")->GetElementImpl("horizontal")->GetElementImpl("max_angl")->GetValue()->GetAsString() << std::endl;
        if (_sdf->HasAttribute("visualize")) {
            PRINT_CUSTOM_INFO("VELODYNE SENSOR pos found");
            auto out = _sdf->Get<std::string>("pose");
            for (auto i : out) {
                std::cout << i;
            }
            std::cout << std::endl;
        }
        PRINT_CUSTOM_INFO("VELODYNE SENSOR LOAD END");
    }
}