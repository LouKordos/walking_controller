#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
  
const int port = 4200;
const int udp_buffer_size = 1024;
  
// Driver code 
int main() { 
    int sockfd; 
    char buffer[udp_buffer_size]; 
    char *hello = "Hello from client"; 
    struct sockaddr_in     servaddr; 
  
    // Creating socket file descriptor 
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    } 
  
    memset(&servaddr, 0, sizeof(servaddr)); 
      
    // Filling server information 
    servaddr.sin_family = AF_INET; 
    servaddr.sin_port = htons(port); 
    servaddr.sin_addr.s_addr = INADDR_ANY; 
      
    int n; 
    socklen_t len;
    
    while(true) {
        sendto(sockfd, (const char *)hello, strlen(hello), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
        printf("Hello message sent.\n"); 
            
        n = recvfrom(sockfd, (char *)buffer, udp_buffer_size, MSG_WAITALL, (struct sockaddr *) &servaddr, &len); 
        buffer[n] = '\0';
        printf("Server sent: %s\n", buffer);
    }
    close(sockfd);
    return 0; 
} 