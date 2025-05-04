#include <lcm/lcm-cpp.hpp>
#include "my_types/ledState.hpp"
#include <csignal>
#include <iostream>

void signalHandler(int signal) {
    printf("interrupt signal (%i) received\n", signal);
    exit(signal);
}

int main(int argc, char* argv[]) {
    lcm::LCM lcm;
    if (!lcm.good()) {
        return 1;
    }

    std::signal(SIGINT, signalHandler);
    my_types::ledState msg;
    std::string input;
    msg.isOn = true;

    while (true) {
        try {
            std::cout << "Enter 'toggle' to change LED state or 'exit' to quit: ";
            std::cin >> input;

            if (input == "toggle") {
                msg.isOn = !msg.isOn;
                lcm.publish("ledState", &msg);
                std::cout << "LED state toggled to: " << (msg.isOn ? "ON" : "OFF") << std::endl;
            } else if (input == "exit") {
                break;
            } else {
                std::cout << "Invalid input. Please enter 'toggle' or 'exit'." << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "Exception caught: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "Unknown exception caught" << std::endl;
        }
    }

    return 0;
}