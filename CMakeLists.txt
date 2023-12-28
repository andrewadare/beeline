cmake_minimum_required(VERSION 3.15)

project(beeline VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")

find_package(Eigen3 REQUIRED)
find_package(manif REQUIRED)
find_package(kalmanif REQUIRED)

file(GLOB SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_subdirectory(applications)
