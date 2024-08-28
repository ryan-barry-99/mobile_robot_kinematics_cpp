/**
 * @file MobileRobot.hpp
 * @brief Header file for the MobileRobot class.
 *
 * This file contains the definition of the `MobileRobot` class, which represents a mobile robot
 * equipped with a set of fixed wheels. The class manages the wheels, performs kinematic calculations,
 * and updates the kinematic model as the wheel configuration changes.
 *
 * The `MobileRobot` class:
 * - Manages a collection of `Wheel` objects.
 * - Handles kinematic calculations through the `MobileRobotKinematics` class.
 * - Provides methods to add and remove wheels and update the kinematics model accordingly.
 *
 * Includes:
 * - `MobileRobotKinematics.hpp`: Header for the kinematic calculations related to the robot.
 * - `Wheel.hpp`: Header for the `Wheel` class, representing individual wheels of the robot.
 *
 * Usage:
 * To use this class, instantiate a `MobileRobot` object with a vector of `Wheel` objects. 
 * Utilize the provided methods to manage wheels and perform kinematic calculations as needed.
 */
#ifndef MOBILE_ROBOT_HPP
#define MOBILE_ROBOT_HPP


#include "MobileRobotKinematics.hpp"
#include "Wheel.hpp"
#include <vector>
#include <string>
#include <iostream>


/**
 * @class MobileRobot
 * @brief Represents a mobile robot with a set of wheels and handles kinematic calculations.
 *
 * The MobileRobot class manages a collection of Wheel objects and performs kinematic 
 * calculations using the MobileRobotKinematics class. It supports adding and removing 
 * wheels, and updates the kinematic model accordingly.
 */
class MobileRobot{
public:
    /**
     * @brief Constructs a MobileRobot object with an initial set of wheels.
     * 
     * Initializes the MobileRobot object and creates an instance of MobileRobotKinematics 
     * using a pointer to the internal vector of wheels. This allows the kinematics to be 
     * dynamically updated as the wheels are added or removed.
     *
     * @param wheels A vector of Wheel objects representing the initial set of wheels for the robot.
     */
    MobileRobot() : kinematics(MobileRobotKinematics(&m_wheels)) {}

    /**
     * @brief Adds a new wheel to the robot and updates the kinematics.
     * 
     * This function adds a new Wheel object to the robot's internal vector of wheels 
     * and updates the kinematics model to reflect the new configuration. It ensures 
     * that the kinematics calculations are based on the current set of wheels.
     *
     * @param wheel A Wheel object representing the new wheel to be added.
     */
    void addWheel(Wheel wheel){
        m_wheels.push_back(wheel);
        kinematics.updateWheels();
    }

    /**
     * @brief Removes a wheel from the robot by name and updates the kinematics.
     * 
     * This function searches for a Wheel object in the robot's internal vector of wheels 
     * by its name. If a match is found, it removes the wheel from the vector and updates 
     * the kinematics model to reflect the change. This ensures that the kinematics 
     * calculations are accurate for the current set of wheels.
     *
     * @param name A string representing the name of the wheel to be removed.
     */
    void removeWheel(std::string name){
        for (int i = 0; i < m_wheels.size(); i++){
            if (m_wheels[i].getName() == name){
                m_wheels.erase(m_wheels.begin() + i);
                kinematics.updateWheels();
                break;
            }
        }
    }


    /**
     * @brief Handles kinematic calculations for the robot.
     */
    MobileRobotKinematics kinematics;

    
private:
    /**
     * @brief List of wheels attached to the robot.
     */
    std::vector<Wheel> m_wheels;

};

#endif // MOBILE_ROBOT_HPP