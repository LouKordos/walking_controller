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
#include <boost/date_time/posix_time/posix_time.hpp>

#include <unistd.h>

#include <errno.h> // It defines macros for reporting and retrieving error conditions through error codes
#include <time.h> // contains various functions for manipulating date and time
#include <unistd.h> // contains various constants
#include <sys/types.h> // contains a number of basic derived types that should be used whenever appropriate
#include <arpa/inet.h> // defines in_addr structure
#include <sys/socket.h> // for socket creation
#include <netinet/in.h> //contains constants and structures needed for internet domain addresses

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <mutex>
#include <iostream>

using Eigen::MatrixXd;

#include "leg_model_functions/C_matrix.hpp"
// #include "model_functions.cpp"

int main() {
    update_C_left_leg(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0);
    std::cout << "this ran" << std::endl;
    std::cout << C << std::endl;
}