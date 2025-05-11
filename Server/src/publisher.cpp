#include <lcm/lcm-cpp.hpp>
#include "LCM_types/Angles/MotorCommand.hpp"
#include <chrono>


int main() {
    lcm::LCM lcm;
    if (!lcm.good()) {
        return 1;
    }
    float target_angles[6] = {45.0, 90.0, 25.0, 55.0, 44.0, 93.0};
    int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();

    Angles::MotorCommand msg;
    for (int i = 0; i < 6; i++) {
        msg.target_angles[i] = target_angles[i];
    }
    msg.timestamp = timestamp;
    
    lcm.publish("MotorAngles", &msg);

    return 0;

}