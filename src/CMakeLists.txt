cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project("Biped Walking Controller")

find_package (Threads)
find_package(casadi)

#set(COMMUNICATIONS_LIBS zcm)

add_compile_options(-std=c++17 -Wpedantic -L /usr/local/lib)
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("/usr/local/include/")

set_source_files_properties(model_functions.cpp PROPERTIES COMPILE_FLAGS "-O3")

add_executable(controller controller.cpp model_functions.cpp Leg.cpp Helpers.cpp CartesianTrajectory.cpp PolyCoefficients4D.cpp SimState.cpp ContactState.cpp)

#target_link_libraries (controller ${CMAKE_THREAD_LIBS_INIT} casadi model_functions helpers leg)
target_link_libraries(controller ${CMAKE_THREAD_LIBS_INIT} casadi)

message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
message("CMAKE_CXX_FLAGS is ${CMAKE_CXX_FLAGS}")
message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")
message("PROJECT_SOURCE_DIR: " ${PROJECT_SOURCE_DIR})
message("CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}")

# Current compilation commands:

# g++ -O3 -pthread -I ../include/ ../controller.cpp -c
# g++ -O3 -pthread -I ../include/ ../model_functions.cpp -c
# g++ -O3 -pthread -I ../include/ ../Leg.cpp -c
# g++ -O3 -pthread -I ../include/ ../Helpers.cpp -c
# g++ -O3 -pthread ./controller.o ./model_functions.o ./Leg.o ./Helpers.o -lcasadi -o controller
