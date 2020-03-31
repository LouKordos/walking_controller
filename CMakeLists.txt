cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project("Biped Walking Controller")

find_package (Threads)
find_package(casadi)

#set(COMMUNICATIONS_LIBS zcm)

add_definitions(-std=c++17 -Wall -Wpedantic -I .. -O3)

add_executable(controller controller.cpp)

#target_link_libraries (controller ${CMAKE_THREAD_LIBS_INIT} ${COMMUNICATIONS_LIBS} casadi)

target_link_libraries (controller ${CMAKE_THREAD_LIBS_INIT} casadi)

message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")