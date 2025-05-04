#include <lcm/lcm-cpp.hpp>
#include "my_types/example.hpp"

int main() {
    lcm::LCM lcm;
    if (!lcm.good()) {
        return 1;
    }

    my_types::example msg;
    msg.id = 42;
    msg.name = "hello world!";
    
    for(int i = 0; i < 100; i++){
        lcm.publish("example channel", &msg);
    }

    return 0;

}