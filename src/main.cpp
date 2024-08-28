#include <vector> // Include this for std::vector
#include <cmath> // Include this for M_PI
#include <Eigen/Dense> // Include this for Eigen library
#include "Wheel.hpp"
#include "MobileRobot.hpp"
#include <iostream>

int main(){
    // Create a set of wheels for the robot
    // Example: Kiwi Drive Robot with 3 omni-directional wheels
    Wheel wheel1("Wheel1", 0.1, 0.2, -M_PI/2, 0.0, 0);
    Wheel wheel2("Wheel2", 0.1, 0.2, M_PI/2, 0.0, 0);
    // Wheel wheel3("Wheel3", 0.1, 0.2, -M_PI/3, 0.0, 0);

    MobileRobot robot = MobileRobot();
    robot.addWheel(wheel1);
    robot.addWheel(wheel2);
    // robot.addWheel(wheel3);
    
    Eigen::MatrixXd zetaDot(3,1);
    zetaDot << 0.1, 0.0, 0.0;
    Eigen::MatrixXd phiDot = robot.kinematics.inverseKinematics(zetaDot);
    std::cout << "Wheel Velocities: " << phiDot << std::endl;
}
