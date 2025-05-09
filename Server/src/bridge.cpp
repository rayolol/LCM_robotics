#include <lcm/lcm-cpp.hpp>
#include "my_types/ledState.hpp"

#include <arpa/inet.h> //for inet_addr
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <stdint.h>


class Handler {
private:
    int sockfd;
    struct sockaddr_in servaddr;
public:
    Handler(const char* udp_ip, int udp_port) {

        sockfd = socket(AF_INET, SOCK_DGRAM, 0);

        if(sockfd < 0) {
            perror("socket creation failed");
            exit(EXIT_FAILURE);
        }
        printf("succesfully created socket");

        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(udp_port);
        servaddr.sin_addr.s_addr = inet_addr(udp_ip);
    }

    ~Handler() {
        close(sockfd);
    }

    void handleMessage(
        const lcm::ReceiveBuffer* buff, 
        const std::string& channel, 
        const my_types::ledState* msg 
    ) {
        std::cout << "received angles on channel: " << channel << std::endl;
        uint8_t value = msg->isOn ? 1 : 0;
        sendto(sockfd, &value , sizeof(bool), 0,
         (const struct sockaddr*)&servaddr, sizeof(servaddr));
    }
};


int main() {
    lcm::LCM lcm;   
    if (!lcm.good()) {
        return 1;
    }

    Handler handler("192.168.0.158", 12345); //change later for corresponding ESP32 port and IP

    lcm.subscribe("ledState", &Handler::handleMessage, &handler);

    while(0 == lcm.handle());

    return 0;
}