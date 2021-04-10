//
// Created by vadim on 10.04.2021.
//

#include "serverHandler/serverHandler.h"

void serverHandler::init() {
    SCL::init();
}

void serverHandler::deinit() {
    SCL::deinit();
}

std::pair<double, double> serverHandler::getControlVector(std::pair<double, double> last_com_vector) {
    auto retv = SCL::send_GET_request("http://192.168.43.25:8000/remote_cars/coordinates/1/move");
    if (retv.empty())
        return last_com_vector;
    return std::pair<double, double>(retv["speed"], retv["angle"]);
}
#include <iostream>
void serverHandler::sendCarTelemetry(double x, double y, double z, std::array<double, 360> data) {
    json tmp = {
            {"x", x},
            {"y", y},
            {"z", z},
            {"data_lidar", data}
    };
    std::cout << tmp << std::endl;
    SCL::send_POST_request("http://192.168.43.25:8000/remote_cars/coordinates/1/send", tmp);
}
