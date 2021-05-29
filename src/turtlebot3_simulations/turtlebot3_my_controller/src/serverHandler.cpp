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
//    auto retv = SCL::send_GET_request("http://192.168.43.25:8000/remote_cars/coordinates/1/move");
    auto retv = SCL::send_GET_request("http://192.168.43.25:8000/moveCar");
    if (retv.empty())
        return last_com_vector;
    return std::pair<double, double>(retv["speed"], retv["angle"]);
}

#include <iostream>

void serverHandler::sendCarTelemetry(double x, double y, double z, double angle, std::vector<double> data) {
    json tmp = {
            {"x", x},
            {"y", y},
            {"z", z},
            {"phi", angle},
            {"dist", data}
    };
    std::cout << tmp["x"] << std::endl << tmp["y"] << std::endl << std::endl;
    SCL::send_POST_request("http://192.168.43.25:8000/sendCoords", tmp);
}
