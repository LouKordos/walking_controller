#include <iostream>
#include <dirent.h>
#include <typeinfo>

#include <chrono>
#include <thread>
#include <functional>

#include <fstream>

#include "nameof.h"

#include <string>
#include <random>
#include <ctime>
#include <cmath>

#include <stdlib.h>

#include <boost/algorithm/string.hpp>

#include <unistd.h>
// #include <zcm/zcm-cpp.hpp>
//#include <sys/types .h>

//#include "leg_state.hpp"
//#include "torque_setpoint.hpp"

#include <errno.h> //It defines macros for reporting and retrieving error conditions through error codes
#include <time.h> //contains various functions for manipulating date and time
#include <unistd.h> //contains various constants
#include <sys/types.h> //contains a number of basic derived types that should be used whenever appropriate
#include <arpa/inet.h> // defines in_addr structure
#include <sys/socket.h> // for socket creation
#include <netinet/in.h> //contains constants and structures needed for internet domain addresses

#include <iomanip>
#include "casadi/casadi.hpp"
#include <eigen3/unsupported/Eigen/MatrixFunctions>

using namespace std;
using namespace std::chrono;

int main() {

    Eigen::Matrix<double, 3, 3> test = (Eigen::Matrix<double, 3, 3>() << 1,2,3,4,5,6,7,8,9).finished();

    std::cout << "test after init:\n" << test << std::endl;

    Eigen::Matrix<double, 2, 2> test2;
    
    test2 = test.block<2,2>(0, 0).eval();

    std::cout << "test2 atfer init:\n" << test2 << std::endl;

    Eigen::Matrix<double, 2, 2> change = (Eigen::Matrix<double, 2, 2>() << 12,13,14,15).finished();

    test.block<2, 2>(0, 0) = change;

    test(0, 1) = 23;

    std::cout << "test after change:\n" << test << std::endl;

    std::cout << "test2 after change:\n" << test2 << std::endl;

    return 0;
}