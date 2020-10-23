#include "include/ContactState.hpp"

ContactState::ContactState(int port) {
    this->port = port;

    update_thread = std::thread([this] { updateContactState(); });
}

void ContactState::updateContactState() {

    long long iteration_counter = 0;
    
    // Setting up UDP server socket... Beware that server and client code are two very different things and waste a lot of time on debugging!!! The Code below is for *receiving* messages!!!
    int sockfd;
    char buffer[udp_buffer_size];
    struct sockaddr_in servaddr, cliaddr; 
      
    // Creating socket file descriptor 
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("Contact State Manager Thread Socket creation failed."); 
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
        perror("Contact State Manager thread socket bind failed"); 
        exit(EXIT_FAILURE);
    }
    
    int msg_length; 
    socklen_t len;

    while(true) {
        msg_length = recvfrom(sockfd, (char *)buffer, udp_buffer_size, 0, ( struct sockaddr *) &cliaddr, &len); // Receive message over UDP containing contact state as 0 or 1 (1 is swing_phase = false)
        buffer[msg_length] = '\0'; // Add string ending delimiter to end of string (msg_length is length of message)
        std::string msg(buffer); // Create string from buffer char array

        contact_mutex.lock();
        contact_state = msg == "1" ? true : false;
        std::cout << "contact_state: " << contact_state << std::endl;
        contact_mutex.unlock();

        iteration_counter++; // Increment iteration counter
    }
}

bool ContactState::hasContact() {
    contact_mutex.lock();
    bool temp_value = contact_state;
    contact_mutex.unlock();

    return temp_value;
}