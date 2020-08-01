cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project("Biped Walking Controller")

find_package (Threads)
find_package(casadi)

#set(COMMUNICATIONS_LIBS zcm)

add_definitions(-std=c++17 -Wall -Wpedantic -I .. -I /usr/local/include/ -L /usr/local/lib -O3)

add_executable(controller controller.cpp)
add_executable(nlp_test nlp_test.cpp)
add_executable(eigen_test eigen_test.cpp)

#target_link_libraries (controller ${CMAKE_THREAD_LIBS_INIT} ${COMMUNICATIONS_LIBS} casadi)

target_link_libraries (controller ${CMAKE_THREAD_LIBS_INIT} casadi)
target_link_libraries (nlp_test ${CMAKE_THREAD_LIBS_INIT} casadi)

message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")
