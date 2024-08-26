/**
 * @file MobileRobotKinematics.hpp
 * 
 * @brief Defines the MobileRobotKinematics class, which represents a mobile robot's kinematic system. 
 *        This class encapsulates properties related to the robot's wheels, including their physical dimensions, 
 *        orientation parameters, and velocity information.
 * 
 * The MobileRobotKinematics class includes:
 * - Member variables for wheel properties such as name, radius, and orientation parameters (L, alpha, beta, gamma).
 * - Methods for setting and getting the wheel's target and current velocities.
 * 
 * @class MobileRobotKinematics
 * 
 * @details The MobileRobotKinematics class is designed to handle the basic attributes and functionalities of a mobile robot's 
 *          kinematic framework. It supports operations for managing the robot's wheels' velocities and provides methods for 
 *          accessing their various physical and operational parameters.
 * 
 * @author Ryan Barry
 * @date Created: August 25, 2024
 */

#ifndef MOBILE_ROBOT_KINEMATICS_HPP
#define MOBILE_ROBOT_KINEMATICS_HPP

#include <string>
#include <vector>
#include <Eigen/Dense>
#include "Wheel.hpp"

using namespace std;


class MobileRobotKinematics {
public:
    /**
     * @brief Constructs a MobileRobotKinematics object with a pointer to a vector of wheels.
     * 
     * Initializes the MobileRobotKinematics object and sets the initial orientation angle to zero.
     * The provided pointer to the vector of wheels is used to set up the kinematics matrices.
     *
     * @param wheels Pointer to a vector of Wheel objects representing the robot's wheels.
     */
    MobileRobotKinematics(std::vector<Wheel>* wheels) : m_theta(0)
    {
        this->updateWheels(wheels);
    }

    /**
     * @brief Updates the internal state of the kinematics object with a new set of wheels.
     * 
     * This function resets the internal state of the MobileRobotKinematics object by updating 
     * the pointer to the vector of wheels and recalculating the number of wheels and kinematics 
     * matrices. It is useful for dynamically changing the wheel configuration of the robot.
     *
     * @param wheels Pointer to a vector of Wheel objects representing the new set of wheels.
     */
    void updateWheels(std::vector<Wheel>* wheels){
        m_wheels = wheels;
        m_numWheels = wheels->size();
        m_rTheta = Eigen::MatrixXd::Identity(m_numWheels, m_numWheels);
        m_J1 = Eigen::MatrixXd::Zero(3, m_numWheels);
        m_C1 = Eigen::MatrixXd::Zero(3, m_numWheels);
        m_J2 = Eigen::MatrixXd::Zero(m_numWheels, m_numWheels);
        m_zetaDot = Eigen::MatrixXd::Zero(3, 1);
        m_phiDot = Eigen::MatrixXd::Zero(m_numWheels, 1);
        this->computeMatrices();
    }

    /**
     * @brief Updates the rotation matrix based on the given angle.
     *
     * This function updates the rotation matrix (`m_rTheta`) of the mobile robot using the provided
     * rotation angle (`theta`). The rotation matrix is computed as a 2D rotation in the XY plane,
     * while leaving the Z axis unchanged.
     *
     * The matrix is updated to:
     *  - Rotate points in the XY plane by the angle `theta`.
     *  - Maintain the Z axis as is.
     *
     * @param theta The rotation angle in radians. This angle is used to compute the cosine and sine
     *              values required for updating the rotation matrix.
     *
     * @note This function assumes that the rotation is only in the XY plane, and the Z axis remains unchanged.
     */
    void updateRotation(double theta){
        m_theta = theta;
        double cTheta = cos(m_theta);
        double sTheta = sin(m_theta);
        m_rTheta << cTheta, -sTheta, 0,
                    sTheta, cTheta, 0,
                    0, 0, 1;
    }

    /**
     * @brief Computes the forward kinematics of the mobile robot.
     *
     * Given the angular velocities of the wheels (`phiDot`), this function calculates the resulting
     * linear and angular velocities of the robot (`zetaDot`). The calculation is performed using
     * the inverse of the rotation matrix (`m_rTheta`), the Jacobian matrix (`m_J1`), and the provided
     * wheel angular velocities.
     *
     * @param phiDot An Eigen::MatrixXd object representing the angular velocities of the wheels.
     *               The matrix should be of size (numWheels, 1), where each element represents the
     *               angular velocity of a wheel.
     *
     * @return An Eigen::Matrix3d object representing the computed linear and angular velocities of the robot.
     *         The matrix is of size (3, 1), where:
     *         - The first element represents the linear velocity in the x direction (x_dot).
     *         - The second element represents the linear velocity in the y direction (y_dot).
     *         - The third element represents the angular velocity about the z axis (phi_dot).
     */
    Eigen::Matrix3d forwardKinematics(Eigen::MatrixXd phiDot){
        m_phiDot = phiDot;
        m_zetaDot = m_rTheta.inverse() * m_J1 * m_phiDot;
        return m_zetaDot;
    }

    /**
     * @brief Computes the inverse kinematics of the mobile robot.
     *
     * Given the desired linear and angular velocities of the robot (`zetaDot`), this function calculates
     * the required angular velocities (`phiDot`) for each wheel to achieve those desired velocities.
     * The calculation is performed using the inverse of the wheel velocity transformation matrix (`m_J2`),
     * the Jacobian matrix (`m_J1`), and the rotation matrix (`m_rTheta`).
     *
     * @param zetaDot An Eigen::MatrixXd object representing the desired linear and angular velocities of the robot.
     *                The matrix should be of size (3, 1), where:
     *                - zetaDot(0, 0) represents the linear velocity in the x direction (x_dot).
     *                - zetaDot(1, 0) represents the linear velocity in the y direction (y_dot).
     *                - zetaDot(2, 0) represents the angular velocity about the z axis (phi_dot).
     *
     * @return An Eigen::MatrixXd object representing the computed angular velocities for the wheels.
     *         The matrix is of size (numWheels, 1), where each element corresponds to the angular velocity of a wheel.
     */
    Eigen::MatrixXd inverseKinematics(Eigen::MatrixXd zetaDot){
        m_zetaDot = zetaDot;
        m_phiDot = m_J2.inverse() * m_J1 * m_rTheta * m_zetaDot;
        return m_phiDot;
    }


private:
    /**
     * @brief Computes the Jacobian matrix (J1), Coriolis matrix (C1), and wheel velocity transformation matrix (J2) for the mobile robot.
     * 
     * This method iterates through all wheels of the robot and calculates:
     * - J1: The Jacobian matrix which maps wheel velocities to the robot's linear and angular velocities.
     * - C1: The Coriolis matrix which accounts for the velocity-dependent forces in the system.
     * - J2: The wheel velocity transformation matrix which relates wheel velocities to robot velocities.
     * 
     * The computed matrices are stored as member variables for use in kinematic calculations.
     */
    void computeMatrices(){
        for (int i = 0; i < m_numWheels; i++){
            Wheel* p_wheel = &((*m_wheels)[i]);
            m_J1.col(i) = p_wheel->calcJ1();
            m_C1.col(i) = p_wheel->calcC1();
            m_J2(i, i) = p_wheel->getRadius();
        }
    }

    vector<Wheel>* m_wheels; ///< Vector of Wheel pointer objects representing the robot's wheels
    int m_numWheels;        ///< Number of wheels in the robot's kinematic system
    int m_theta;            ///< Angle of the robot's orientation
    Eigen::MatrixXd m_rTheta; ///< Rotation matrix for the robot's orientation
    Eigen::MatrixXd m_J1;     ///< Jacobian matrix for the robot's kinematic system
    Eigen::MatrixXd m_C1;   ///< Coriolis matrix for the robot's kinematic system
    Eigen::MatrixXd m_J2;   ///< Wheel Velocity Transformation Matrix
    Eigen::MatrixXd m_zetaDot; ///< Vector of robot velocities
    Eigen::MatrixXd m_phiDot; ///< Vector of wheel velocities
};
#endif // MOBILE_ROBOT_KINEMATICS_HPP