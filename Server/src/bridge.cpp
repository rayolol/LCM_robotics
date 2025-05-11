#include <lcm/lcm-cpp.hpp>
#include "LCM_types/Angles/MotorCommand.hpp"

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
        const Angles::MotorCommand* msg 
    ) {
        std::cout << "received angles on channel: " << channel << std::endl;
        uint8_t buffer[56];
        float angles[6];
        float target_speed[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        for(int i = 0; i < 6; i++) {
            std::cout << msg->target_angles[i] << "\n";
            angles[i] = msg->target_angles[i];
        }
        int64_t timestamp = htobe64(msg->timestamp);
        size_t offest = 0;

        memcpy(buffer + offest, &timestamp, sizeof(int64_t));
        offest += sizeof(int64_t);
        memcpy(buffer + offest, angles, sizeof(float) * 6);
        offest += sizeof(float) * 6;
        memcpy(buffer + offest, target_speed, sizeof(float) * 6);
        offest += sizeof(float) * 6;

        std::cout << "sending angles to ESP32" << std::endl;
        size_t n = sendto(sockfd, buffer , offest , 0,
         (const struct sockaddr*)&servaddr, sizeof(servaddr));
        printf("sent %zu bytes\n", n);
        if (n < 0) {
            perror("sendto");
            exit(EXIT_FAILURE);
        }
    }
};


int main() {
    lcm::LCM lcm;   
    if (!lcm.good()) {
        return 1;
    }

    Handler handler("192.168.0.158", 12345); //change later for corresponding ESP32 port and IP

    lcm.subscribe("MotorAngles", &Handler::handleMessage, &handler);

    while(0 == lcm.handle());

    return 0;
}