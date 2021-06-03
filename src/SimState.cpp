#include "include/SimState.hpp"

SimState::SimState(const int port) {
    this->port = port;
    update_thread = std::thread([this] { update_pause_state(); });
}

void SimState::update_pause_state() {
    long long iteration_counter = 0;
    
    // Setting up UDP server socket... Beware that server and client code are two very different things and waste a lot of time on debugging!!! The Code below is for *receiving* messages!!!
    int sockfd;
    char buffer[udp_buffer_size];
    struct sockaddr_in servaddr, cliaddr; 
      
    // Creating socket file descriptor 
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("Pause State Manager Thread Socket creation failed."); 
        exit(EXIT_FAILURE);
    }
    
    memset(&servaddr, 0, sizeof(servaddr)); 
    memset(&cliaddr, 0, sizeof(cliaddr)); 

    // Filling server information  
    servaddr.sin_family = AF_INET; // IPv4 
    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1"); 
    servaddr.sin_port = htons(port);
    
    // Bind the socket with the server address 
    if ( bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 ) 
    {
        perror("Pause State Manager thread socket bind failed"); 
        exit(EXIT_FAILURE);
    }
    
    int msg_length; 
    socklen_t len;

    while(true) {
        msg_length = recvfrom(sockfd, (char *)buffer, udp_buffer_size, 0, ( struct sockaddr *) &cliaddr, &len); // Receive message over UDP containing pause state as 0 or 1
        buffer[msg_length] = '\0'; // Add string ending delimiter to end of string (msg_length is length of message)
        std::string msg(buffer); // Create string from buffer char array
        std::vector<string> split_msg = split_string(msg, '|');
        pause_state_mutex.lock();
        pause_state = split_msg[0] == "1" ? true : false;
        pause_state_mutex.unlock();

        sim_time_mutex.lock();
        sim_time = atof(split_msg[1].c_str());
        // std::cout << "paused: " << isPaused() <<  ", sim_time: " << sim_time << std::endl;
        sim_time_mutex.unlock();

        iteration_counter++; // Increment iteration counter
    }
}

bool SimState::isPaused() {
    pause_state_mutex.lock();
    bool temp_value = pause_state;
    pause_state_mutex.unlock(); 

    return temp_value;
}

double SimState::getSimTime() {
    sim_time_mutex.lock();
    double temp_value = sim_time;
    sim_time_mutex.unlock();

    return temp_value;
}