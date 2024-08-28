/**
 * @file Wheel.hpp
 * 
 * @brief Defines the Wheel class, which represents a single wheel in a mobile robot's kinematic system. 
 *        This class encapsulates properties related to the wheel, including its physical dimensions, 
 *        orientation parameters, and velocity information.
 * 
 * The Wheel class includes:
 * - Member variables for wheel properties such as name, radius, and orientation parameters (L, alpha, beta, gamma).
 * - Methods for setting and getting the wheel's target and current velocities.
 * 
 * @class Wheel
 * 
 * @details The Wheel class is designed to handle the basic attributes and functionalities of a wheel within 
 *          a mobile robot's kinematic framework. It supports operations for managing the wheel's velocity 
 *          and provides methods for accessing its various physical and operational parameters.
 * 
 * @author Ryan Barry
 * @date Created: August 25, 2024
 */

#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <string>
#include <Eigen/Dense>

using namespace std;

class Wheel {
public:
    // Constructor
    Wheel(string name, double radius, double L, double alpha, double beta, double gamma) :
        m_name(name), 
        m_radius(radius), 
        m_L(L), 
        m_alpha(alpha), 
        m_beta(beta), 
        m_gamma(gamma), 
        m_currentVelocity(0), 
        m_targetVelocity(0),
        m_J1(3,1),    // Initialize m_J1 as a 3x1 matrix
        m_C1(3,1)     // Initialize m_C1 as a 3x1 matrix
    {
        // Empty Constructor
    }


    /**
     * @brief Sets the target velocity for the wheel.
     * @param velocity The target velocity to be set.
     */
    void setTargetVelocity(double velocity) { m_targetVelocity = velocity; }

    /**
     * @brief Sets the current velocity for the wheel.
     * @param velocity The current velocity to be set.
     */
    void setCurrentVelocity(double velocity) { m_currentVelocity = velocity; }


    // Getter Functions

    /**
     * @brief Returns the target velocity of the wheel.
     * @return double The current target velocity.
     */
    double getTargetVelocity() const { return m_targetVelocity; }

    /**
     * @brief Returns the current velocity of the wheel.
     * @return double The current velocity.
     */
    double getCurrentVelocity() const { return m_currentVelocity; }

    /**
     * @brief Returns the name of the wheel.
     * @return std::string The name of the wheel.
     */
    std::string getName() const { return m_name; }

    /**
     * @brief Returns the radius of the wheel.
     * @return double The radius of the wheel.
     */
    double getRadius() const { return m_radius; }

    /**
     * @brief Returns the length L associated with the wheel.
     * @return double The length L of the wheel.
     */
    double getL() const { return m_L; }

    /**
     * @brief Returns the alpha angle of the wheel.
     * @return double The alpha angle.
     */
    double getAlpha() const { return m_alpha; }

    /**
     * @brief Returns the beta angle of the wheel.
     * @return double The beta angle.
     */
    double getBeta() const { return m_beta; }

    /**
     * @brief Returns the gamma angle of the wheel.
     * @return double The gamma angle.
     */
    double getGamma() const { return m_gamma; }


    // Kinematic Functions
    
    /**
     * @brief Calculates the Jacobian matrix (J1) for the wheel.
     * 
     * This method computes the Jacobian matrix (J1) for the wheel based on its
     * geometric parameters. The Jacobian matrix maps changes in wheel configuration
     * to changes in the robot's state. The calculation is based on the wheel's
     * parameters such as alpha (wheel angle), beta (angle between wheel and axis),
     * and gamma (roller angle).
     * 
     * @return Eigen::MatrixXd The computed Jacobian matrix for the wheel.
     */
    Eigen::MatrixXd calcJ1(){
        m_J1 << sin(m_alpha + m_beta + m_gamma), 
                -cos(m_alpha + m_beta + m_gamma), 
                -m_L/m_radius*cos(m_beta + m_gamma);

        return m_J1;
    }

    /**
     * @brief Calculates the Coriolis matrix (C1) for the wheel.
     * 
     * This method computes the Coriolis matrix (C1) for the wheel, which represents
     * the effect of Coriolis forces on the wheel's dynamics. The calculation is based
     * on the wheel's geometric parameters and its current velocity. This matrix is
     * used to account for the forces due to changes in wheel velocity and orientation.
     * 
     * @return Eigen::MatrixXd The computed Coriolis matrix for the wheel.
     */
    Eigen::MatrixXd calcC1(){
        m_C1 << cos(m_alpha + m_beta + m_gamma), 
                sin(m_alpha + m_beta + m_gamma), 
                m_L*m_currentVelocity*sin(m_beta + m_gamma);

        return m_C1;
    }



private:
    string m_name;           ///< Name of the wheel
    double m_radius;         ///< Radius of the wheel in meters
    double m_L;              ///< The distance from the center of the robot to the center of the wheel in meters.
    double m_alpha;          ///< The angle offset from the robot's x-axis
    double m_beta;           ///< The angle between center of robot and wheel rotation axis
    double m_gamma;          ///< The angle between the wheel's roller rotation axis and the wheel's center (0 if no rollers).
    double m_currentVelocity; ///< Current velocity of the wheel
    double m_targetVelocity;  ///< Target velocity of the wheel
    Eigen::MatrixXd m_J1;    ///< Jacobian matrix for the wheel
    Eigen::MatrixXd m_C1;    ///< Coriolis matrix for the wheel
};

#endif // WHEEL_HPP
