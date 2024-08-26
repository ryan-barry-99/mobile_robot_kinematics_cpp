#include <iostream>
#include <vector> // Include this for std::vector
#include <cmath> // Include this for M_PI
#include "Wheel.hpp"
#include "MobileRobot.hpp"

int main(){
    // Create a set of wheels for the robot
    // Example: Kiwi Drive Robot with 3 omni-directional wheels
    Wheel wheel1("Wheel1", 0.1, 0.2, 0.0, 0.0, M_PI);
    Wheel wheel2("Wheel2", 0.1, 0.2, 2*M_PI/3, 0.0, M_PI);
    Wheel wheel3("Wheel3", 0.1, 0.2, 4*M_PI/3, 0.0, M_PI);

    MobileRobot robot({wheel1, wheel2, wheel3});
}
