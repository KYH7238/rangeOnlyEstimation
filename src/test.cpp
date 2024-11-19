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
#include <tf/tf.h> // 추가: tf::getYaw 사용을 위해 필요

class State {
public:
    double xJi, yJi, thetaJi; // 상대 로봇의 위치 및 yaw 각도 (드론 좌표계 기준)
    State() : xJi(0), yJi(0), thetaJi(0) {}
};

class RelativePoseEstimation {
private:
    Eigen::Matrix3d covP;
    Eigen::Matrix3d covQ;
    Eigen::MatrixXd covR;
    ros::Publisher pub_;
    ros::NodeHandle nh_;

    Eigen::Matrix3d jacobianF;
    Eigen::MatrixXd jacobianH;

    Eigen::VectorXd vecZ;
    Eigen::VectorXd vecH;

    Eigen::Matrix<double, 4, 3> uavUwbPositions;
    Eigen::Matrix<double, 2, 3> ugvUwbPositions;

    Eigen::Vector3d vI; // 드론의 속도 (드론 좌표계 기준)
    double wI;          // 드론의 yaw 속도

    double dt;

    State state;

    double droneYaw;    // 드론의 현재 yaw 각도

public:
    RelativePoseEstimation() {
        pub_ = nh_.advertise<geometry_msgs::PoseStamped>("estimated_state", 1);
        covP = Eigen::Matrix3d::Identity() * 0.01;
        covQ = Eigen::Matrix3d::Identity() * 0.001;
        covR = Eigen::MatrixXd::Identity(8, 8) * 0.015;

        jacobianF = Eigen::Matrix3d::Identity();
        jacobianH = Eigen::MatrixXd::Zero(8, 3);

        vecZ = Eigen::VectorXd::Zero(8);
        vecH = Eigen::VectorXd::Zero(8);

        uavUwbPositions << 0.13,  0.13, 0,
                           0.13, -0.13, 0,
                          -0.13,  0.13, 0,
                          -0.13, -0.13, 0;

        ugvUwbPositions << 0.25, 0, 0,
                          -0.25, 0, 0;

        dt = 0;
        vI = Eigen::Vector3d::Zero();
        wI = 0.0;
        droneYaw = 0.0;
    }

    void setViWi(const Eigen::Vector3d &vi, double wi) {
        vI = vi;
        wI = wi;
    }

    void setDroneYaw(double yaw) {
        droneYaw = yaw;
    }

    void setZ(const Eigen::VectorXd& z) {
        vecZ = z;
        vecH = Eigen::VectorXd::Zero(z.size());
        jacobianH = Eigen::MatrixXd::Zero(z.size(), 3);
    }

    void setDt(const double deltaT) {
        dt = deltaT;
    }

    void resultPub() {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "ekf";
        pose.header.stamp = ros::Time::now();

        pose.pose.position.x = state.xJi;
        pose.pose.position.y = state.yJi;
        pose.pose.position.z = 0;

        Eigen::Matrix3d R;
        double theta = state.thetaJi;
        R << cos(theta), -sin(theta), 0,
             sin(theta),  cos(theta), 0,
                  0,           0,     1;

        Eigen::Quaterniond quaternion(R);
        pose.pose.orientation.x = quaternion.x();
        pose.pose.orientation.y = quaternion.y();
        pose.pose.orientation.z = quaternion.z();
        pose.pose.orientation.w = quaternion.w();
        pub_.publish(pose);
    }

    void motionModel() {
        // 드론의 yaw를 고려하여 드론 좌표계에서 속도를 계산
        double thetaI = droneYaw;
        Eigen::Matrix2d RI;
        RI << cos(thetaI), -sin(thetaI),
              sin(thetaI),  cos(thetaI);

        // 드론의 속도 (월드 좌표계 기준)
        Eigen::Vector2d vI_world = RI * vI.head<2>();

        // 상대 로봇의 속도 업데이트 (드론 좌표계 기준)
        // 여기서는 상대 로봇의 움직임을 드론의 움직임에 대한 상대적인 변화로 표현합니다.
        state.xJi -= vI_world(0) * dt;
        state.yJi -= vI_world(1) * dt;
        state.thetaJi -= wI * dt; // 드론의 yaw 변화 반영
    }

    void motionModelJacobian() {
        jacobianF = Eigen::Matrix3d::Identity();
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
                Eigen::Vector3d pj = ugvUwbPositions.row(j).transpose();
                Eigen::Vector3d pjInUav = R * pj + relPose;
                vecH(idx) = (pi - pjInUav).norm();
                jacobianH.row(idx) = measurementModelJacobian(pi, pj, pjInUav);
                idx++;
            }
        }
    }

    Eigen::RowVectorXd measurementModelJacobian(const Eigen::Vector3d& pi, const Eigen::Vector3d& pj, const Eigen::Vector3d& pjInUav) {
        Eigen::RowVectorXd dH(3);
        double theta = state.thetaJi;
        Eigen::Vector2d delta = pi.head<2>() - pjInUav.head<2>();
        double dist = delta.norm();
        if (dist < 1e-4) dist = 1e-4;

        double dx = delta(0);
        double dy = delta(1);

        dH(0) = dx / dist; // ∂h/∂xJi
        dH(1) = dy / dist; // ∂h/∂yJi
        dH(2) = ((-sin(theta) * pj(0) - cos(theta) * pj(1)) * dx +
                 ( cos(theta) * pj(0) - sin(theta) * pj(1)) * dy) / dist; // ∂h/∂thetaJi

        return dH;
    }

    void correction() {
        measurementModel();
        Eigen::VectorXd residual = vecZ - vecH;

        Eigen::MatrixXd S = jacobianH * covP * jacobianH.transpose() + covR;
        Eigen::MatrixXd K = covP * jacobianH.transpose() * S.inverse();

        Eigen::VectorXd stateVec(3);
        stateVec << state.xJi, state.yJi, state.thetaJi;
        stateVec += K * residual;

        state.xJi = stateVec(0);
        state.yJi = stateVec(1);
        state.thetaJi = stateVec(2);

        covP = (Eigen::Matrix3d::Identity() - K * jacobianH) * covP;

        resultPub();
    }
};

class EKFNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber uwb_sub_;
    ros::Subscriber odom_sub_;
    RelativePoseEstimation ekf_;
    ros::Time prevTime_;

public:
    EKFNode() {
        uwb_sub_ = nh_.subscribe("ranges", 1, &EKFNode::uwbCallback, this);
        odom_sub_ = nh_.subscribe("mavros/local_position/odom",1, &EKFNode::odomCallback, this);
        prevTime_ = ros::Time::now();
    }

    void uwbCallback(const relative::UwbRange::ConstPtr& msg) {
        ros::Time currTime = msg->header.stamp;
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

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 드론의 선속도 및 각속도
        Eigen::Vector3d vi(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        double wi = msg->twist.twist.angular.z;

        // 드론의 yaw 각도
        double droneYaw = tf::getYaw(msg->pose.pose.orientation);

        ekf_.setViWi(vi, wi);
        ekf_.setDroneYaw(droneYaw);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "relative_pose_estimation");
    EKFNode ekfNode;
    ros::spin();
    return 0;
}
