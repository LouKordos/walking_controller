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

class ContactState {
    public: ContactState(int port);
    public: bool hasContact();
    private: void updateContactState();

    public: int port;
    public: int udp_buffer_size = 4096;
    
    private: std::thread update_thread;
    private: std::mutex contact_mutex;

    private: bool contact_state;
};