cmake_minimum_required(VERSION 3.15)
project(SCL)

set(CMAKE_CXX_STANDARD 14)
set(JSON_BuildTests OFF CACHE INTERNAL "")

find_package(nlohmann_json 3.2.0 REQUIRED)

add_library(${PROJECT_NAME} SCL.cpp SCL.h)

target_link_libraries(${PROJECT_NAME} PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries(${PROJECT_NAME} PRIVATE curl)