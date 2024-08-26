#include "Wheel.hpp"

Wheel::Wheel(string name, double radius, double L, double alpha, double beta, double gamma) :
    m_name(name), 
    m_radius(radius), 
    m_L(L), 
    m_alpha(alpha), 
    m_beta(beta), 
    m_gamma(gamma), 
    m_currentVelocity(0), 
    m_targetVelocity(0)
{
    // Empty Constructor
}

// Setter functions
void Wheel::setTargetVelocity(double velocity) { m_targetVelocity = velocity; }
void Wheel::setCurrentVelocity(double velocity) { m_currentVelocity = velocity; }

// Getter functions
double Wheel::getTargetVelocity() const { return m_targetVelocity; }
double Wheel::getCurrentVelocity() const { return m_currentVelocity; }
string Wheel::getName() const { return m_name; }
double Wheel::getRadius() const { return m_radius; }
double Wheel::getL() const { return m_L; }
double Wheel::getAlpha() const { return m_alpha; }
double Wheel::getBeta() const { return m_beta; }
double Wheel::getGamma() const { return m_gamma; }


Eigen::MatrixXd Wheel::calcJ1(){
    m_J1 << sin(m_alpha + m_beta + m_gamma), 
            -cos(m_alpha + m_beta + m_gamma), 
            -m_L*cos(m_beta + m_gamma);

    return m_J1;
}

Eigen::MatrixXd Wheel::calcC1(){
    m_C1 << cos(m_alpha + m_beta + m_gamma), 
            sin(m_alpha + m_beta + m_gamma), 
            m_L*m_currentVelocity*sin(m_beta + m_gamma);

    return m_C1;
}
