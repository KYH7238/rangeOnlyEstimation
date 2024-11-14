/**
 * Author: Yonghee Kim
 * Date: 2024-11-14
 * brief: UGV/UAV Range Only Relative Pose Estimation using filtering method (EKF)
 */
#include "relativePoseEstimationEKF.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

RPE::RPE(){
    covP = Matrix<double,5,5>::Identity()*0.01;
    covQ = Matrix<double,5,5>::Identity()*0.001;
    covR = Matrix<double,8,8>::Identity()*0.015;

    jacobianF = Matrix<double,5,5>::Identity();
    jacobianH = Matrix<double,8,5>::setZero();
    covR = Matrix<double,8,8>::Identity()*0.015;
    
    vecZ = Vector::setZero();
    vecH = Vector::setZero();

    dt = 0;
    TOL = 1e-9;
    state.Xji = 0;
    state.Yji = 0;
    state.Thetaji = 0;
    state.Vji = 0;
    state.Wji = 0;

}

RPE::~RPE(){}

void RPE::setState() {
    
}

STATE RPE::getState() {

}

void RPE::setDt(const double deltaT) {
    dt = deltaT;
}

void RPE::setZ(const UwbData &uwbData) {
    for (i = 0; i < vecZ.rows(); i++){
        vecZ(i) = uwbData.ranges(i);
    }
}

void RPE::motionModel() {
    state.x += state.Vj * dt * cos(state.Thetaji);
    state.y += state.Vj * dt * sin(state.Thetaji);
    state.Thetaji += state.Wj * dt;
    state.Vji = state.Vji; 
    state.Wji = state.Wji; 
}

void RPE::motionModelJacobian() {
    jacobianF(0,2) = -state.Vj*dt*sin(state.Thetaji);
    jacobianF(0,3) = dt*cos(state.Thetaji);    
    jacobianF(1,2) = state.Vj*dt*cos(state.Thetaji);
    jacobianF(1,3) = dt*sin(state.Thetaji);    
    jacobianF(2,4) = dt;    
}

void RPE::prediction() {
    motionModel();
    motionModelJacobian();
    covP = jacobianF*covP*jacobianF.transpose() + covQ;
}

void RPE::measurementModel() {
    for (i = 0; i < ){
        
    }
}

void RPE::measurementModelJacobian() {
    
}

STATE RPE::correction() {
    measurementModel();
    measurementModelJacobian();
}




