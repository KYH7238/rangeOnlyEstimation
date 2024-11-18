#include <relative/UwbRange.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
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
    ros::Publisher pub_;
    ros::NodeHandle nh_;
    
    Eigen::Matrix<double, 5, 5> jacobianF;
    Eigen::Matrix<double, 8, 5> jacobianH;

    Eigen::VectorXd vecZ;
    Eigen::VectorXd vecH;
    Eigen::Matrix<double, 4, 3> uavUwbPositions;
    Eigen::Matrix<double, 2, 3> ugvUwbPositions;
    
    Eigen::Vector3d vI, wI;

    double dt, tol;

    State state;

public:
    RelativePoseEstimation() {
        pub_ = nh_.advertise<geometry_msgs::PoseStamped>("estimated_state", 1);
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
        vI = Eigen::Vector3d::Zero();
        wI = Eigen::Vector3d::Zero();
    }
    
    void setViWi(const Eigen::VectorXd &vi, const Eigen::VectorXd &wi) {
        vI = vi;
        wI = wi;
    }

    void setZ(const Eigen::VectorXd& z) {
        vecZ = z;
        vecH = Eigen::VectorXd::Zero(z.size());
        jacobianH = Eigen::MatrixXd::Zero(z.size(), 5);
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
        pub_.publish(pose);
    }

    void motionModel() {
        state.xJi += (state.vJ * cos(state.thetaJi) + state.yJi * wI(2) - vI(0)) * dt;
        state.yJi += (state.vJ * sin(state.thetaJi) - state.xJi * wI(2)) * dt;
        state.thetaJi += (state.wJ - wI(2)) * dt;
    }

    void motionModelJacobian() {
        jacobianF << 1, wI(2) * dt, -state.vJ * sin(state.thetaJi) * dt, cos(state.thetaJi) * dt, 0,
                    -wI(2) * dt, 1, state.vJ * cos(state.thetaJi) * dt, sin(state.thetaJi) * dt, 0,
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
        double theta = state.thetaJi;
        int idx = 0;
        Eigen::Vector3d relPose(state.xJi, state.yJi, 0);
        Eigen::Matrix3d R;
        R << cos(theta), -sin(theta), 0,
            sin(theta),  cos(theta), 0,
                0,           0,     1;

        for (int i = 0; i < uavUwbPositions.rows(); i++) {
            Eigen::Vector3d pi = uavUwbPositions.row(i).transpose();

            for (int j = 0; j < ugvUwbPositions.rows(); j++) {
                if (idx >= vecH.size()) {
                    ROS_ERROR("Index idx (%d) exceeds vecH size (%ld)", idx, vecH.size());
                    return;
                }

                Eigen::Vector3d pj = ugvUwbPositions.row(j).transpose();
                Eigen::Vector3d pjInUav = R * pj + relPose;
                vecH(idx) = (pi - pjInUav).norm();
                jacobianH.row(idx) = measurementModelJacobian(pi, pj, pjInUav);
                idx++;
            }
        }
    }

    Eigen::RowVectorXd measurementModelJacobian(const Eigen::Vector3d& pi, const Eigen::Vector3d& pj, const Eigen::Vector3d& pjInUav) {
        Eigen::RowVectorXd dH(5);
        double theta = state.thetaJi;
        double dist = (pi - pjInUav).norm();
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
    ros::Subscriber sub_;
    ros::Subscriber sub;
    RelativePoseEstimation ekf_;
    ros::Time prevTime_;
    
    Eigen::VectorXd vi;
    Eigen::VectorXd wi;

public:
    EKFNode() {
        sub_ = nh_.subscribe("ranges", 1, &EKFNode::uwbCallback, this);
        sub = nh_.subscribe("mavros/local_position/odom",1, &EKFNode::localOodmCallback, this);
        prevTime_ = ros::Time::now();
    }

    void uwbCallback(const relative::UwbRange::ConstPtr& msg) {
        ros::Time currTime = ros::Time::now();
        ros::Duration duration = currTime - prevTime_;
        double dt = duration.toSec();

        if (dt > 0) {
            if (msg->ranges.size() != 8) {
                ROS_ERROR("UWB ranges size is not 8, but %zu", msg->ranges.size());
                return;
            }
            Eigen::Map<const Eigen::VectorXf> zf(msg->ranges.data(), msg->ranges.size());
            Eigen::VectorXd z = zf.cast<double>();

            ekf_.setZ(z);
            ekf_.setDt(dt);
            ekf_.prediction();
            ekf_.correction();
            prevTime_ = currTime;
        }
    }

    void localOodmCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        vi = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        wi = Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
        ekf_.setViWi(vi, wi);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "relative_pose_estimation");
    EKFNode ekfNode;
    ros::spin();
    return 0;
}
