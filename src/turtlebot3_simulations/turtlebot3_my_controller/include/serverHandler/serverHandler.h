//
// Created by vadim on 10.04.2021.
//

#ifndef SRC_SERVERHANDLER_H
#define SRC_SERVERHANDLER_H

#include "SCL.h"
#include <utility>

namespace serverHandler {

    void init();

    void deinit();

    std::pair<double, double> getControlVector(std::pair<double, double> last_com_vector);

    void sendCarTelemetry(double x, double y, double z, double angle, std::vector<double> data);

}

#endif //SRC_SERVERHANDLER_H
