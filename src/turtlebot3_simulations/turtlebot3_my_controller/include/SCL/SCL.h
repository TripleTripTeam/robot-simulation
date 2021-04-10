//
// Created by vadim on 06.03.2021.
//

#ifndef SCL_H
#define SCL_H

#include <string>
#include <nlohmann/json.hpp>
#include <curl/curl.h>

using json = nlohmann::json;

namespace SCL {

    void init();
    void deinit();

    json send_GET_request(const std::string& url);

    json send_POST_request(const std::string& url, json &req_json);

    CURL* get_curl_instptr();
}
#endif //SCL_H
