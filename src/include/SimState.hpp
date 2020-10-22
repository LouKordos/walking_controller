#include <mutex>
#include <thread>
#include <functional>
#include <chrono>

#include <errno.h> //It defines macros for reporting and retrieving error conditions through error codes
#include <time.h> //contains various functions for manipulating date and time
#include <unistd.h> //contains various constants
#include <sys/types.h> //contains a number of basic derived types that should be used whenever appropriate
#include <arpa/inet.h> // defines in_addr structure
#include <sys/socket.h> // for socket creation
#include <netinet/in.h> //contains constants and structures needed for internet domain addresses

#include <string> // std strings
#include <boost/algorithm/string.hpp> // Split strings and needed for memset

#include <iostream>

#include "Helpers.hpp"

using namespace std::chrono;

class SimState {
    private: bool pause_state;
    private: double sim_time;

    private: std::thread update_thread;
    private: std::mutex pause_state_mutex, sim_time_mutex;

    public: void update_pause_state();

    private: const int udp_buffer_size = 4096;

    public: SimState(const int port);

    public: bool isPaused();
    public: double getSimTime();
    public: int port;
};