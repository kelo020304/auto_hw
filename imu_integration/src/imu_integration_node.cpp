//
// Created by ubuntu on 24-9-22.
//
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <sys/stat.h>

#include "visualization_msgs/Marker.h"
#include "string.h"


ros::Subscriber sub_imu;
ros::Subscriber sub_wheel;
ros::Publisher pub_path, pub_odometry;
Eigen::Vector3d gravity;
nav_msgs::Path path;
ros::Publisher meshPub;

bool isInit = false;

struct State {
    Eigen::Quaterniond q;
    Eigen::Vector3d p;
//    Eigen::Vector3d p_;
    Eigen::Vector3d v;
//    Eigen::Vector3d v_;
//    Eigen::Vector3d error_v;
//    Eigen::Vector3d error_p;
    double time;
};

State state;



void InitState(const double &time) {
    ROS_WARN("init state");
    state.q = Eigen::Quaterniond(1, 0, 0, 0);
    state.p = Eigen::Vector3d(0, 0, 0);
    state.v = Eigen::Vector3d(0, 0, 0);
//    state.v_ = Eigen::Vector3d(0, 0, 0);
//    state.p_ = Eigen::Vector3d(0, 0, 0);
//    state.error_v = Eigen::Vector3d(0, 0, 0);
//    state.error_p = Eigen::Vector3d(0, 0, 0);
    state.time = time;
    gravity = Eigen::Vector3d(0, 0, -9.805);
    isInit = true;
}



void ImuIntegration(const Eigen::Vector3d &imuAcc, const Eigen::Vector3d &imuGyro, const double &t) {
    // mean prediction
    // update p v q of the state through IMU integration
    // *****

    // YOUR CODE
    const double dt = t - state.time;
    const Eigen::Vector3d qa = state.q * (imuAcc) + gravity;
    const Eigen::Vector3d omg = (imuGyro) * dt / 2.0;
    Eigen::Quaterniond dq(1.0, omg[0] , omg[1], omg[2]);
    state.q = (state.q * dq ).normalized();
    const Eigen::Vector3d qaa = state.q * (imuAcc) + gravity;
    const Eigen::Vector3d acc = 0.5 * (qa + qaa);

    state.p = state.p + state.v * dt + 0.5 * qa * dt * dt;
    state.v = state.v + qa * dt;

//    state.v_ = state.v_ + acc * dt;
//    state.p_ = state.p_ + state.v * dt + 0.5 * acc * dt * dt;
//    state.error_v = state.v - state.v_;
//    state.error_p = state.p - state.p_;
//
//    ROS_INFO_STREAM("Velocity Error: [" << state.error_v[0] << ", " << state.error_v[1] << ", " << state.error_v[2] << "]");
//    ROS_INFO_STREAM("Position Error: [" << state.error_p[0] << ", " << state.error_p[1] << ", " << state.error_p[2] << "]");
    state.time = t;
    // *****
}



void Publish() {
    double time = state.time;
    Eigen::Vector3d position = state.p;
    Eigen::Quaterniond q = state.q;
    Eigen::Vector3d velocity = state.v;
    nav_msgs::Odometry odometry;
    odometry.header.frame_id = "world";
    odometry.header.stamp = ros::Time(time);
    odometry.pose.pose.position.x = position(0);
    odometry.pose.pose.position.y = position(1);
    odometry.pose.pose.position.z = position(2);
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    odometry.pose.pose.orientation.w = q.w();
    odometry.twist.twist.linear.x = velocity(0);
    odometry.twist.twist.linear.y = velocity(1);
    odometry.twist.twist.linear.z = velocity(2);

    pub_odometry.publish(odometry);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    pose_stamped.header.stamp = ros::Time(time);
    pose_stamped.pose = odometry.pose.pose;
    path.poses.push_back(pose_stamped);
    pub_path.publish(path);

    // Mesh model
    visualization_msgs::Marker meshROS;
    meshROS.header.frame_id = std::string("world");
    meshROS.header.stamp = ros::Time(time);
    meshROS.ns = "mesh";
    meshROS.id = 0;
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    meshROS.pose.position.x = position(0);
    meshROS.pose.position.y = position(1);
    meshROS.pose.position.z = position(2);
    Eigen::Matrix3d rot;
    rot << 0, 0, -1, 0, -1, 0, -1, 0, 0;
    Eigen::Quaterniond Q;
    //Q = q * rot;
    Q = q;
    meshROS.pose.orientation.w = Q.w();
    meshROS.pose.orientation.x = Q.x();
    meshROS.pose.orientation.y = Q.y();
    meshROS.pose.orientation.z = Q.z();
    meshROS.scale.x = 2;
    meshROS.scale.y = 2;
    meshROS.scale.z = 2;
    meshROS.color.a = 1.0;
    meshROS.color.r = 1.0;
    meshROS.color.g = 0.0;
    meshROS.color.b = 0.0;
    std::string mesh_resource = "package://imu_integration/meshes/hummingbird.mesh";
    meshROS.mesh_resource = mesh_resource;
    meshPub.publish(meshROS);
}

void ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {

    double time = imu_msg->header.stamp.toSec();
    if (!isInit) {
        InitState(time);
    }
    Eigen::Vector3d acc(imu_msg->linear_acceleration.x,
                        imu_msg->linear_acceleration.y,
                        imu_msg->linear_acceleration.z);
    Eigen::Vector3d gyro(imu_msg->angular_velocity.x,
                         imu_msg->angular_velocity.y,
                         imu_msg->angular_velocity.z);

    ImuIntegration(acc, gyro, time);
    Publish();
}

void WheelCallback(const nav_msgs::OdometryConstPtr &wheel_msg) {
    Eigen::Vector3d wheelSpeed(wheel_msg->twist.twist.linear.x,
                               wheel_msg->twist.twist.linear.y,
                               wheel_msg->twist.twist.linear.z);

    double time = wheel_msg->header.stamp.toSec();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_integration");
    ros::NodeHandle n("~");

    sub_imu = n.subscribe("/data_generator/imu", 2000, &ImuCallback,
                           ros::TransportHints().tcpNoDelay());
    sub_wheel = n.subscribe("/data_generator/odometry", 2000, &WheelCallback,
                             ros::TransportHints().tcpNoDelay());
    pub_path = n.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    meshPub   = n.advertise<visualization_msgs::Marker>("robot", 100, true);
    path.header.frame_id = "world";

    ros::spin();
}
