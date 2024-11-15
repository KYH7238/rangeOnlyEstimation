#include "msg/UwbRange"
#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <queue>

class RelativePoseEstimation {
private:

    Matrix<double,5,5> covP::Identity()*0.01;
    Matrix<double,5,5> covQ::Identity()*0.001;
    Matrix<double,8,8> covR::Identity()*0.015;

    Matrix<double,5,5> jacobianF::Identity();
    Matrix<double,8,5> jacobianH::setZero();
    Matrix<double,8,8> covR::Identity()*0.015;
    
    Vector<double,8> vecZ::setZero();
    Vector<double,8> vecH::setZero();
    Matrix<double,4, 4> uavUwbPositions;
    Matrix<double, 2, 3> ugvUwbPositions;
    
    double dt, TOL, Vi, Wi;

public:
    RelativePoseEstimation() {

        uavUwbPositions << 0.13, 0.13, 0,
                        0.13, -0.13, 0,
                        -0.13, 0.13, 0,
                        -0.13, -0.13, 0;

        ugvUwbPositions << 0.25, 0, 0,
                        -0.25, 0, 0;

        dt = 0;
        TOL = 1e-9;
        Vi = 0;
        Wi = 0;
        
        STATE state;
        
        state.Xji = 0;
        state.Yji = 0;
        state.Thetaji = 0;
        state.Vj = 0;
        state.Wj = 0;

    void setZ() {
        return vecZ;
    }

    void resultPub() {
        Matrix<double, 3, 3> R << cos(theta), -sin(theta), 0,
                                  sin(theta), cos(theta), 0,
                                  0,            0,        1;
    
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "ekf";
        pose.header.stamp = ros::Time::now();

        pose.pose.position.x = state.Xji;
        pose.pose.position.y = state.Xji;
        pose.pose.position.z = state.Thetaji;
        Quaterniond quaternion(R);
        pose.pose.orientation.x = quaternion.x();
        pose.pose.orientation.y = quaternion.y();
        pose.pose.orientation.z = quaternion.z();
        pose.pose.orientation.w = quaternion.w();
    }

    void motionModel(){
        state.x += (state.Vj*cos(state.Thetaji) + state.Yji*Wi - Vi)*dt;
        state.y += (state.Vj*sin(state.Thetaji) - state.Xji*Wi)*dt;
        state.Thetaji += (state.Wj-Wi) * dt;
        state.Vji = state.Vji; 
        state.Wji = state.Wji; 
    }

    void motionModelJacobian() {
        jacobianF << 1, Wi*dt, -state.Vj*sin(state.Thetaji)*dt, cos(state.Thetaji)*dt, 0,
                    -Wi*dt, 1, state.Vj*cos(state.Thetaji)*dt, sin(state.Thetaji)*dt, 0,
                    0, 0, 1, 0, dt,
                    0, 0, 0, 1, 0,
                    0, 0, 0, 0, 1;
    }

    void prediction() {
        motionModel();
        motionModelJacobian();
        covP = jacobianF*covP*jacobianF.transpose() + covQ;
    }

    void measurementModel() {
        double theta = state.Thetaji;
        int idx = 0;
        Vector3d relPose << state.Xji, state.Yji, 0;
        Matrix<double, 3, 3> R << cos(theta), -sin(theta), 0,
                                  sin(theta), cos(theta), 0,
                                  0,            0,        1;

        for (int i = 0; i < uavUwbPositions.rows(); i++) {
            Vector pi = uavUwbPositions(i,:);

            for (int j = 0; j < ugvUwbPositions.rows(); j++) {
                Vector3d pj = ugvUwbPositions(j,:);
                vector3d pjInUav = R*pj + relPose;
                vecH(idx) = (pi - pjInUav).norm();
                jacobianH(idx, :) = measurementModelJacobian(pi, pj, pjInUav, idx);
            }
            idx++;
        }
    }

    Vector3d measurementModelJacobian(const double &pi, const double &pj, const Vector3d &pjInUav, idx) {
        Vector<double, 5> dH;
        double theta = state.Thetaji;
        dH(0) = (pjInUav(0) - pi(0))/vecH(idx);
        dH(1) = (pjInUav(1) - pi(1))/vecH(idx);
        dH(2) = ((-sin(theta)*pj(0)-cos(theta)*pj(1))*(pjInUav(0) - pi(0)) + 
                 (cos(theta)*pj(0)-sin(theta)*pj(1))*(pjInUav(1) - pi(1))) / vecH(idx);
        dH(3) = 0;
        dH(4) = 0;
        
        return dH.transpose();
    }

    void correction() {
        measurementModel();
        measurementModelJacobian();
        Vector<double, 8> residual = vecZ - vecH;
        Matrix<double, 8, 8> K;
        
        K = covP*jacobianH.transpose()*(jacobianH*covP*jacobianH.transpose() + covR).inverse()
        state +=K*(residual);
        covP = (Matrix<double, 5 ,5>::Identity()-(K*jacobianH))*covP;
        resultPub();
    }
    }

}

class STATE {
    public:
        double Xji, Yji, Thetaji, Vj, Wj;
}

class EKFNode {

private:
    ros::NodeHandle n_;
    ros::Publisher pub;
    ros::Subscriber sub;
    RelativePoseEstimation ekf;
    double prevTime = 0;

public:
    EKFNode();
    {
        sub = n_.subscriber("ranges", 1 ,uwbCallback, this);
    }
    void uwbCallback(const UwbRange &msg) {
        ros::Time currTime = msg->header.stamp;
        ros::Duration duration = currTime - prevTime;
        double dt = duration.toSec();
        if (dt!=0) {
            ekf.setZ() << msg->ranges;
            ekf.prediction();
            ekf.correction();
            prevTime = currTime;
        }
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "RPE");
    ros::NodeHandle nh;
    EKFNode object;
    ros::spin();
    return 0;
}
