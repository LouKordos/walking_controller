cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project("Biped Walking Controller")

find_package (Threads)
find_package(casadi)

#set(COMMUNICATIONS_LIBS zcm)

<<<<<<< Updated upstream
add_compile_options(-std=c++17 -Wpedantic -I /usr/local/include/ -L /usr/local/lib -O3)
=======
add_compile_options(-std=c++17 -Wpedantic -I .. -I /usr/local/include/ -L /usr/local/lib -O3)
>>>>>>> Stashed changes

add_executable(controller controller.cpp)
add_executable(nlp_test nlp_test.cpp)
add_executable(eigen_test eigen_test.cpp)
#add_executable(header_test header_test.cpp)

set(SOURCES ${PROJECT_SOURCE_DIR}/Helpers.cpp ${PROJECT_SOURCE_DIR}/Leg.cpp ${PROJECT_SOURCE_DIR}/model_functions.cpp)

include_directories("${PROJECT_SOURCE_DIR}/include")

<<<<<<< Updated upstream
add_library(Sources STATIC ${SOURCES})
=======
#add_library(model_functions SHARED model_functions.cpp)
add_library(Sources SHARED ${SOURCES})
>>>>>>> Stashed changes

#target_link_libraries (controller ${CMAKE_THREAD_LIBS_INIT} casadi model_functions helpers leg)
target_link_libraries(controller ${CMAKE_THREAD_LIBS_INIT} casadi ${SOURCES})
target_link_libraries (nlp_test ${CMAKE_THREAD_LIBS_INIT} casadi)

message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")
message("PROJECT_SOURCE_DIR: " ${PROJECT_SOURCE_DIR})

# Commands to manually link so far:
<<<<<<< Updated upstream
# g++ -fPIC -O3 -c -Wall c++_filename.cpp
# g++ -shared -o libfilename.so c++_filename.o
#g++ -O3 -Wall -L$(pwd) -Wl,-rpath=$(pwd) ../main_c++_filename.cpp -lfilename -o main_c++_filename
=======
# g++ -fPIC -O3 -c -Wall c++_filename.cpp 
# g++ -shared -o libfilename.so c++_filename.o
#g++ -O3 -Wall -L$(pwd) -Wl,-rpath=$(pwd) ../main_c++_filename.cpp -lfilename -o main_c++_filename.cpp
>>>>>>> Stashed changes
