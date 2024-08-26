#include "MobileRobotKinematics.hpp"

MobileRobotKinematics::MobileRobotKinematics(vector<Wheel>* wheels) : m_theta(0)
{
    this->updateWheels(wheels);
}

void MobileRobotKinematics::updateWheels(vector<Wheel>* wheels){
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

void MobileRobotKinematics::updateRotation(double theta){
    m_theta = theta;
    double cTheta = cos(m_theta);
    double sTheta = sin(m_theta);
    m_rTheta << cTheta, -sTheta, 0,
                sTheta, cTheta, 0,
                0, 0, 1;
}

void MobileRobotKinematics::computeMatrices(){
    for (int i = 0; i < m_numWheels; i++){
        Wheel* p_wheel = &((*m_wheels)[i]);
        m_J1.col(i) = p_wheel->calcJ1();
        m_C1.col(i) = p_wheel->calcC1();
        m_J2(i, i) = p_wheel->getRadius();
    }
}


Eigen::Matrix3d MobileRobotKinematics::forwardKinematics(Eigen::MatrixXd phiDot){
    m_phiDot = phiDot;
    m_zetaDot = m_rTheta.inverse() * m_J1 * m_phiDot;
    return m_zetaDot;
}

Eigen::MatrixXd MobileRobotKinematics::inverseKinematics(Eigen::MatrixXd zetaDot){
    m_zetaDot = zetaDot;
    m_phiDot = m_J2.inverse() * m_J1 * m_rTheta * m_zetaDot;
    return m_phiDot;
}
