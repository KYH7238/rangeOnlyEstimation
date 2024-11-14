#pragma once
#include "types.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

class RPE {
public:
    RPE();
    ~ RPE();
    void setDt(const double deltaT);
    void setZ (const UwbData &uwbData)
    void prediction();
    STATE correction();
    void motionModel();
    void motionModelJacobian();
    void measurementModel();
    void measurementModelJacobian();
    void setState();
    STATE getState();

private:
    Eigen::MatrixXd covP, covQ, covR, jacobianF, jacobianH;
    Eigen::Vector<doubel, 8> vecZ, vecH;
    double dt, TOL;
    STATE state;
    
}