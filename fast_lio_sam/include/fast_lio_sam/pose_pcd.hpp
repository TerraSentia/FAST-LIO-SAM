#pragma once
///// coded headers
#include "utilities.hpp"

struct PosePcd
{
    pcl::PointCloud<pcl::PointXYZI> pcd_;
    Eigen::Matrix4d pose_eig_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pose_corrected_eig_ = Eigen::Matrix4d::Identity();
    double timestamp_;
    int idx_;
    bool processed_ = false;
    PosePcd() {}
    PosePcd(const nav_msgs::msg::Odometry &odom_in,
            const sensor_msgs::msg::PointCloud2 &pcd_in,
            const int &idx_in);
};

inline PosePcd::PosePcd(const nav_msgs::msg::Odometry &odom_in,
                        const sensor_msgs::msg::PointCloud2 &pcd_in,
                        const int &idx_in)
{
    tf2::Quaternion q(odom_in.pose.pose.orientation.x,
                     odom_in.pose.pose.orientation.y,
                     odom_in.pose.pose.orientation.z,
                     odom_in.pose.pose.orientation.w);
    tf2::Matrix3x3 rot_mat_tf(q);
    Eigen::Matrix3d rot_mat_eig;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            rot_mat_eig(i, j) = rot_mat_tf[i][j];
        }
    }
    pose_eig_.block<3, 3>(0, 0) = rot_mat_eig;
    pose_eig_(0, 3) = odom_in.pose.pose.position.x;
    pose_eig_(1, 3) = odom_in.pose.pose.position.y;
    pose_eig_(2, 3) = odom_in.pose.pose.position.z;
    pose_corrected_eig_ = pose_eig_;
    pcl::PointCloud<pcl::PointXYZI> tmp_pcd;
    pcl::fromROSMsg(pcd_in, tmp_pcd);
    pcd_ = transformPcd(tmp_pcd, pose_eig_.inverse()); // FAST-LIO publish data in world frame,
                                                       // so save it in LiDAR frame
    timestamp_ = odom_in.header.stamp.sec;
    idx_ = idx_in;
}