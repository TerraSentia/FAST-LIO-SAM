#pragma once

#include <ctime>
#include <cmath>
#include <chrono> //time check
#include <vector>
#include <memory>
#include <deque>
#include <mutex>
#include <string>
#include <utility> // pair, make_pair
#include <tuple>
#include <filesystem>
#include <fstream>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "pose_pcd.hpp"
#include "utilities.hpp"

#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>


typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2> odom_pcd_sync_pol;


class FastLioSam : public rclcpp::Node
{
public:
    FastLioSam(const rclcpp::NodeOptions & options);
    ~FastLioSam();

    void loadParams();
    void initPublishers();
    void initSubscribers();
    void initTimers();

    void loopTimerCallback();
    void visTimerCallback();

private:
    LoopClosureConfig lc_config_;

    std::string map_frame_;
    std::string package_path_;
    std::string seq_name_;

    std::mutex realtime_pose_mutex_, keyframes_mutex_, graph_mutex_, vis_mutex_;

    Eigen:Matrix4d last_corrected_pose_ = Eigen::Matrix4d::Identity();
    Eigen:Matrix4d odom_delta_ = Eigen::Matrix4d::Identity();
    
    PosePcd current_frame_;

    std::vector<PosePcd> keyframes_;
    std::vector<std::pair<size_t, size_t>> loop_idx_pairs_; // for vis

    int current_keyframe_idx_ = 0;
    int sub_key_num_;

    bool is_initialized_ = false;
    bool loop_added_flag_ = false;
    bool loop_added_flag_vis_ = false;
    bool global_map_vis_switch_ = true;
    bool save_map_bag_ = false, save_map_pcd_ = false, save_in_kitti_format_ = false;

    std::shared_ptr<gtsam::ISAM2> isam_handler_ = nullptr;

    gtsam::NonlinearFactorGraph gtsam_graph_;
    gtsam::Values init_esti_;
    gtsam::Values corrected_esti_;
    
    double keyframe_thr_;
    double voxel_res_;
    double loop_update_hz_;
    double vis_hz_;
    // tf::TransformBroadcaster broadcaster_;
    
    pcl::PointCloud<pcl::PointXYZ> odoms_, corrected_odoms_;
    
    nav_msgs::Path odom_path_, corrected_path_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corrected_odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corrected_pcd_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corrected_current_pcd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_src_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_dst_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_fine_aligned_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr realtime_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr corrected_path_pub_;

    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pcd_sub_;

    std::unique_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>> sub_odom_pcd_sync_;
    // not so important for now
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr save_flag_sub_;
    
    rclcpp::TimerBase::SharedPtr loop_timer_;
    rclcpp::TimerBase::SharedPtr vis_timer_;

    void updateOdomsAndPaths(const PosePcd &pose_pcd_in);
    bool checkIfKeyFrame(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd);
    // one method involving markers
    // visualization_msgs::Marker getLoopMarkers(const gtsam::Values &corrected_esti_in);
    void odomPcdCallback(const nav_msgs::msg::Odometry::SharedPtr& odom_msg, const sensor_msgs::msg::PointCloud2::SharedPtr& pcd_msg);
    void saveFlagCallback(const std_msgs::msg::String::SharedPtr msg);
    
    
    
};
