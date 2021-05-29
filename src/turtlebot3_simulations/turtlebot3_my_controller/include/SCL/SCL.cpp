//
// Created by vadim on 06.03.2021.
//

#include "SCL.h"
#include <pthread.h>

static CURL *curl_inst;
static pthread_mutex_t scl_mutex;

static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp) {
    ((std::string *) userp)->append((char *) contents, size * nmemb);
    return size * nmemb;
}

void SCL::init() {
    curl_global_init(CURL_GLOBAL_ALL);
    curl_inst = curl_easy_init();
    pthread_mutex_init(&scl_mutex, nullptr);
}

void SCL::deinit() {
    curl_global_cleanup();
    pthread_mutex_destroy(&scl_mutex);
}

#include <iostream>
#include <cstring>

json SCL::send_GET_request(const std::string &url) {
    json retv;

    std::string retv_str;
    CURLcode err;

    pthread_mutex_lock(&scl_mutex);
    {
        curl_easy_setopt(curl_inst, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl_inst, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl_inst, CURLOPT_WRITEDATA, &retv_str);

        err = curl_easy_perform(curl_inst);
    }
    pthread_mutex_unlock(&scl_mutex);

    if (err)
        fprintf(stderr, "curl_easy_perform() failed: %s\n",
                curl_easy_strerror(err));

    if (retv_str == std::string("404 page not found"))
        return retv;

    retv = json::parse(retv_str);

    return retv;
}
#include <iostream>
json SCL::send_POST_request(const std::string &url, json &req_json) {
    json retv;

    std::string msg = req_json.dump();
    std::string retv_str;
    CURLcode err;

    pthread_mutex_lock(&scl_mutex);
    {
        curl_easy_setopt(curl_inst, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl_inst, CURLOPT_POSTFIELDS, msg.c_str());
        curl_easy_setopt(curl_inst, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl_inst, CURLOPT_WRITEDATA, &retv_str);

        err = curl_easy_perform(curl_inst);
    }
    pthread_mutex_unlock(&scl_mutex);

    if (err)
        fprintf(stderr, "curl_easy_perform() failed: %s\n",
                curl_easy_strerror(err));

    if (retv_str == std::string("404 page not found"))
        return retv;

    retv = json::parse(retv_str);

    return retv;
}

CURL *SCL::get_curl_instptr() {
    return curl_inst;
}
