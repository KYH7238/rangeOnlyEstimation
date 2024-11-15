#include <relative/UwbRange.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <queue>

class State {
public:
    double xJi, yJi, thetaJi, vJ, wJ;

    State() : xJi(0), yJi(0), thetaJi(0), vJ(0), wJ(0) {}
};

class RelativePoseEstimation {
private:
    Eigen::Matrix<double, 5, 5> covP;
    Eigen::Matrix<double, 5, 5> covQ;
    Eigen::Matrix<double, 8, 8> covR;

    Eigen::Matrix<double, 5, 5> jacobianF;
    Eigen::Matrix<double, 8, 5> jacobianH;

    Eigen::VectorXd vecZ;
    Eigen::VectorXd vecH;
    Eigen::Matrix<double, 4, 3> uavUwbPositions;
    Eigen::Matrix<double, 2, 3> ugvUwbPositions;

    double dt, tol, vI, wI;

    State state;

public:
    RelativePoseEstimation() {
        covP = Eigen::Matrix<double, 5, 5>::Identity() * 0.01;
        covQ = Eigen::Matrix<double, 5, 5>::Identity() * 0.001;
        covR = Eigen::Matrix<double, 8, 8>::Identity() * 0.015;

        jacobianF = Eigen::Matrix<double, 5, 5>::Identity();
        jacobianH = Eigen::Matrix<double, 8, 5>::Zero();

        vecZ = Eigen::VectorXd::Zero(8);
        vecH = Eigen::VectorXd::Zero(8);

        uavUwbPositions << 0.13,  0.13, 0,
                           0.13, -0.13, 0,
                          -0.13,  0.13, 0,
                          -0.13, -0.13, 0;

        ugvUwbPositions << 0.25, 0, 0,
                          -0.25, 0, 0;

        dt = 0;
        tol = 1e-9;
        vI = 0;
        wI = 0;
    }

    void setZ(const Eigen::VectorXd& z) {
        vecZ = z;
    }

    void setDt(const double deltaT) {
        dt = deltaT;
    }

    void resultPub() {
        double theta = state.thetaJi;

        Eigen::Matrix3d R;
        R << cos(theta), -sin(theta), 0,
             sin(theta),  cos(theta), 0,
                  0,           0,     1;

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "ekf";
        pose.header.stamp = ros::Time::now();

        pose.pose.position.x = state.xJi;
        pose.pose.position.y = state.yJi;
        pose.pose.position.z = 0;

        Eigen::Quaterniond quaternion(R);
        pose.pose.orientation.x = quaternion.x();
        pose.pose.orientation.y = quaternion.y();
        pose.pose.orientation.z = quaternion.z();
        pose.pose.orientation.w = quaternion.w();

        // pub 추가해야댐
    }

    void motionModel() {
        state.xJi += (state.vJ * cos(state.thetaJi) + state.yJi * wI - vI) * dt;
        state.yJi += (state.vJ * sin(state.thetaJi) - state.xJi * wI) * dt;
        state.thetaJi += (state.wJ - wI) * dt;
    }

    void motionModelJacobian() {
        jacobianF << 1, wI * dt, -state.vJ * sin(state.thetaJi) * dt, cos(state.thetaJi) * dt, 0,
                    -wI * dt, 1, state.vJ * cos(state.thetaJi) * dt, sin(state.thetaJi) * dt, 0,
                     0,       0, 1, 0, dt,
                     0,       0, 0, 1, 0,
                     0,       0, 0, 0, 1;
    }

    void prediction() {
        motionModel();
        motionModelJacobian();
        covP = jacobianF * covP * jacobianF.transpose() + covQ;
    }

    void measurementModel() {
        double theta = state.thetaJi; // + drone Yaw?
        int idx = 0;
        Eigen::Vector3d relPose(state.xJi, state.yJi, 0);
        Eigen::Matrix3d R;
        R << cos(theta), -sin(theta), 0,
             sin(theta),  cos(theta), 0,
                  0,           0,     1;

        for (int i = 0; i < uavUwbPositions.rows(); i++) {
            Eigen::Vector3d pi = uavUwbPositions.row(i).transpose();

            for (int j = 0; j < ugvUwbPositions.rows(); j++) {
                Eigen::Vector3d pj = ugvUwbPositions.row(j).transpose();
                Eigen::Vector3d pjInUav = R * pj + relPose;
                vecH(idx) = (pi - pjInUav).norm();
                jacobianH.row(idx) = measurementModelJacobian(pi, pj, pjInUav, idx);
                idx++;
            }
        }
    }

    Eigen::RowVectorXd measurementModelJacobian(const Eigen::Vector3d& pi, const Eigen::Vector3d& pj, const Eigen::Vector3d& pjInUav, int idx) {
        Eigen::RowVectorXd dH(5);
        double theta = state.thetaJi;
        double dist = vecH(idx);
        if (dist < 1e-4) dist = 1e-4;

        dH(0) = (pjInUav(0) - pi(0)) / dist;
        dH(1) = (pjInUav(1) - pi(1)) / dist;
        dH(2) = ((-sin(theta) * pj(0) - cos(theta) * pj(1)) * (pjInUav(0) - pi(0)) +
                 ( cos(theta) * pj(0) - sin(theta) * pj(1)) * (pjInUav(1) - pi(1))) / dist;
        dH(3) = 0;
        dH(4) = 0;

        return dH;
    }

    void correction() {
        measurementModel();
        Eigen::VectorXd residual = vecZ - vecH;

        Eigen::MatrixXd S = jacobianH * covP * jacobianH.transpose() + covR;
        Eigen::MatrixXd K = covP * jacobianH.transpose() * S.inverse();

        Eigen::VectorXd stateVec(5);
        stateVec << state.xJi, state.yJi, state.thetaJi, state.vJ, state.wJ;
        stateVec += K * residual;

        state.xJi = stateVec(0);
        state.yJi = stateVec(1);
        state.thetaJi = stateVec(2);
        state.vJ = stateVec(3);
        state.wJ = stateVec(4);

        covP = (Eigen::Matrix<double, 5, 5>::Identity() - K * jacobianH) * covP;

        resultPub();
    }
};

class EKFNode {
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    RelativePoseEstimation ekf_;
    ros::Time prevTime_;

public:
    EKFNode() {
        sub_ = nh_.subscribe("ranges", 1, &EKFNode::uwbCallback, this);
        pub_ = nh_.advertise<geometry_msgs::PoseStamped>("estimated_pose", 1);
        prevTime_ = ros::Time::now();
    }

    void uwbCallback(const relative::UwbRange::ConstPtr& msg) {
        ros::Time currTime = msg->header.stamp;
        ros::Duration duration = currTime - prevTime_;
        double dt = duration.toSec();
        if (dt > 0) {
            Eigen::Map<const Eigen::VectorXf> zf(msg->ranges.data(), msg->ranges.size());
            Eigen::VectorXd z = zf.cast<double>();
            ekf_.setZ(z);
            ekf_.setDt(dt);
            ekf_.prediction();
            ekf_.correction();
            prevTime_ = currTime;
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "relative_pose_estimation");
    EKFNode ekfNode;
    ros::spin();
    return 0;
}
