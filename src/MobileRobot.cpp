#include "MobileRobot.hpp"

MobileRobot::MobileRobot(vector<Wheel> wheels) : 
    m_wheels(wheels), 
    kinematics(MobileRobotKinematics(&m_wheels)) {}

void MobileRobot::addWheel(Wheel wheel){
    m_wheels.push_back(wheel);
    kinematics.updateWheels(&m_wheels);
}

void MobileRobot::removeWheel(string name){
    for (int i = 0; i < m_wheels.size(); i++){
        if (m_wheels[i].getName() == name){
            m_wheels.erase(m_wheels.begin() + i);
            kinematics.updateWheels(&m_wheels);
            break;
        }
    }
}