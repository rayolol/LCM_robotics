#include <lcm/lcm-cpp.hpp>
#include "my_types/example.hpp"

class handler {
public:
    void handleMessage(
        const lcm::ReceiveBuffer* buffer,
        const std::string& chan,
        const my_types::example* msg
     ) {
        printf("received message on channel: %s",chan.c_str());
        printf("id -> %d\n", msg->id);
        printf("name -> %s\n", msg->name.c_str());
     }   
};


int main() {
    lcm::LCM lcm;
    if(!lcm.good()) {
        return 1;
    }

    handler handlerObject;
    lcm.subscribe("example channel", &handler::handleMessage, &handlerObject);


    while(0 == lcm.handle());

    
    return 0;
}