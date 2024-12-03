#include "fast_lio_sam/fast_lio_sam_2.h"

using namespace std::placeholders;
using namespace std::chrono_literals;

FastLioSam::FastLioSam(const rclcpp::NodeOptions & options) : Node("fast_lio_sam_node", options)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    loadParams();

    initPublishers();

    initSubscribers();

    initTimers();

    loop_closure_.reset(new LoopClosure(lc_config_));

    gtsam::ISAM2Params isam_params_;
    isam_params_.relinearizeThreshold = 0.01;
    isam_params_.relinearizeSkip = 1;
    isam_handler_ = std::make_shared<gtsam::ISAM2>(isam_params_);
    /* ROS things */
    odom_path_.header.frame_id = map_frame_;
    corrected_path_.header.frame_id = map_frame_;

    RCLCPP_INFO(this->get_logger(), "Main class, starting node..");


}

FastLioSam::~FastLioSam()
{
    // // save map
    // if (save_map_bag_)
    // {
    //     rosbag::Bag bag;
    //     bag.open(package_path_ + "/result.bag", rosbag::bagmode::Write);
    //     {
    //         std::lock_guard<std::mutex> lock(keyframes_mutex_);
    //         for (size_t i = 0; i < keyframes_.size(); ++i)
    //         {
    //             ros::Time time;
    //             time.fromSec(keyframes_[i].timestamp_);
    //             bag.write("/keyframe_pcd", time, pclToPclRos(keyframes_[i].pcd_, map_frame_));
    //             bag.write("/keyframe_pose", time, poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_));
    //         }
    //     }
    //     bag.close();
    //     ROS_INFO("\033[36;1mResult saved in .bag format!!!\033[0m");
    // }
    // if (save_map_pcd_)
    // {
    //     pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
    //     corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size()); // it's an approximated size
    //     {
    //         std::lock_guard<std::mutex> lock(keyframes_mutex_);
    //         for (size_t i = 0; i < keyframes_.size(); ++i)
    //         {
    //             *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
    //         }
    //     }
    //     const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
    //     pcl::io::savePCDFileASCII<PointType>(package_path_ + "/result.pcd", *voxelized_map);
    //     ROS_INFO("\033[32;1mResult saved in .pcd format!!!\033[0m");
    // }
}


void FastLioSam::loadParams()
{
    this->declare_parameter("basic.map_frame", "map");
    this->declare_parameter("basic.loop_update_hz", 1.0);
    this->declare_parameter("basic.vis_hz", 0.5);

    this->declare_parameter("keyframe.keyframe_threshold", 1.0);
    this->declare_parameter("keyframe.num_submap_keyframes", 5);
    
    this->declare_parameter("loop.loop_detection_radius", 15.0);
    this->declare_parameter("loop.loop_detection_timediff_threshold", 10.0);
    
    this->declare_parameter("icp.icp_voxel_resolution", 0.3);
    this->declare_parameter("icp.icp_score_threshold", 0.3);

    this->declare_parameter("result.save_voxel_resolution", 0.3);
    this->declare_parameter("result.save_map_pcd", false);
    this->declare_parameter("result.save_map_bag", false);
    this->declare_parameter("result.save_in_kitti_format", false);
    this->declare_parameter("result.seq_name", "");


    this->get_parameter("basic.map_frame", map_frame_);
    this->get_parameter("basic.loop_update_hz", loop_update_hz_);
    this->get_parameter("basic.vis_hz", vis_hz_);

    this->get_parameter("keyframe.keyframe_threshold", keyframe_thr_);
    this->get_parameter("keyframe.num_submap_keyframes", lc_config_.num_submap_keyframes_);
    
    this->get_parameter("loop.loop_detection_radius", lc_config_.loop_detection_radius_);
    this->get_parameter("loop.loop_detection_timediff_threshold", lc_config_.loop_detection_timediff_threshold_);
    lc_config.icp_max_corr_dist_ = lc_config.loop_detection_radius_ * 1.5;
    
    this->get_parameter("icp.icp_voxel_resolution", lc_config_.voxel_res_);
    this->get_parameter("icp.icp_score_threshold", lc_config_.icp_score_threshold_);
    
    this->get_parameter("result.save_voxel_resolution", voxel_res_);
    this->get_parameter("result.save_map_pcd", save_map_pcd_);
    this->get_parameter("result.save_map_bag", save_map_bag_);
    this->get_parameter("result.save_in_kitti_format", save_in_kitti_format_);
    this->get_parameter("result.seq_name", seq_name_);
}

void FastLioSam::initPublishers()
{
    odom_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ori_odom", 10);
    corrected_odom_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("corrected_odom", 10);
    corrected_pcd_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("corrected_map", 10);
    corrected_current_pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("corrected_current_pcd", 10);
    debug_src_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("src", 10);
    debug_dst_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("dst", 10);
    debug_fine_aligned_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("aligned", 10);
    realtime_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_stamped", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("ori_path", 10);
    corrected_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("corrected_path", 10);
    
}

void FastLioSam::initSubscribers()
{
    odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, "odometry");
    pcd_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "cloud_registered");
    sub_odom_pcd_sync_ = std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(odom_sub_, pcd_sub_, 10);
    sub_odom_pcd_sync_->registerCallback(std::bind(&FastLioSam::odomPcdCallback, this, _1, _2));

    sub_save_flag_ = this->create_subscription<std_msgs::msg::SharedPtr>("save_dir", 1, std::bind(&FastLioSam::saveFlagCallback, this, _1));
}

void FastLioSam::initTimers()
{
    loop_timer_ = this->create_wall_timer(500ms, std::bind(&FastLioSam::loopTimerCallback, this));
    vis_timer_ = this->create_wall_timer(500ms, std::bind(&FastLioSam::visTimerCallback, this));
}

void FastLioSam::odomPcdCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg, const sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg)
{
    Eigen::Matrix4d last_odom_tf;
    last_odom_tf = current_frame_.pose_eig_;
    current_frame_ = PosePcd(*odom_msg, pcd_msg*, current_keyframe_idx_);
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    {
        std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
        odom_delta_ = odom_delta_ * last_odom_tf_.inverse() * current_frame_.pose_eig_;
        current_frame_.pose_corrected_eig_ = last_corrected_pose_ * odom_delta_;
        realtime_pose_pub_.publish(poseEigToPoseStamped(current_frame_.pose_corrected_eig_, map_frame_));
        // broadcaster
    }
    corrected_current_pcd_pub_->publish(pclToPclRos(transformPcd(current_frame_.pcd_, current_frame_.pose_corrected_eig_), map_frame_));

    if (!is_initialized_)
    {
        keyframes_.push_back(current_frame_);
        updateOdomsAndPaths(current_frame_);
        auto variance_vector = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished(); // rad*rad,
                                                                                                    // meter*meter
        gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
        gtsam_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, poseEigToGtsamPose(current_frame_.pose_eig_), prior_noise));
        init_esti_.insert(current_keyframe_idx_, poseEigToGtsamPose(current_frame_.pose_eig_));
        current_keyframe_idx_++;
        is_initialized_ = true;
    }
    else
    {
        //// 2. check if keyframe
        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        if (checkIfKeyframe(current_frame_, keyframes_.back()))
        {
            // 2-2. if so, save
            {
                std::lock_guard<std::mutex> lock(keyframes_mutex_);
                keyframes_.push_back(current_frame_);
            }
            // 2-3. if so, add to graph
            auto variance_vector = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
            gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
            gtsam::Pose3 pose_from = poseEigToGtsamPose(keyframes_[current_keyframe_idx_ - 1].pose_corrected_eig_);
            gtsam::Pose3 pose_to = poseEigToGtsamPose(current_frame_.pose_corrected_eig_);
            {
                std::lock_guard<std::mutex> lock(graph_mutex_);
                gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(current_keyframe_idx_ - 1,
                                                                    current_keyframe_idx_,
                                                                    pose_from.between(pose_to),
                                                                    odom_noise));
                init_esti_.insert(current_keyframe_idx_, pose_to);
            }
            current_keyframe_idx_++;

            //// 3. vis
            high_resolution_clock::time_point t3 = high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lock(vis_mutex_);
                updateOdomsAndPaths(current_frame_);
            }

            //// 4. optimize with graph
            high_resolution_clock::time_point t4 = high_resolution_clock::now();
            // m_corrected_esti = gtsam::LevenbergMarquardtOptimizer(m_gtsam_graph, init_esti_).optimize(); // cf. isam.update vs values.LM.optimize
            {
                std::lock_guard<std::mutex> lock(graph_mutex_);
                isam_handler_->update(gtsam_graph_, init_esti_);
                isam_handler_->update();
                if (loop_added_flag_) // https://github.com/TixiaoShan/LIO-SAM/issues/5#issuecomment-653752936
                {
                    isam_handler_->update();
                    isam_handler_->update();
                    isam_handler_->update();
                }
                gtsam_graph_.resize(0);
                init_esti_.clear();
            }

            //// 5. handle corrected results
            // get corrected poses and reset odom delta (for realtime pose pub)
            high_resolution_clock::time_point t5 = high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
                corrected_esti_ = isam_handler_->calculateEstimate();
                last_corrected_pose_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(corrected_esti_.size() - 1));
                odom_delta_ = Eigen::Matrix4d::Identity();
            }
            // correct poses in keyframes
            if (loop_added_flag_)
            {
                std::lock_guard<std::mutex> lock(keyframes_mutex_);
                for (size_t i = 0; i < corrected_esti_.size(); ++i)
                {
                    keyframes_[i].pose_corrected_eig_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(i));
                }
                loop_added_flag_ = false;
            }
            high_resolution_clock::time_point t6 = high_resolution_clock::now();

            RCLCPP_INFO(this->get_logger(), "real: %.1f, key_add: %.1f, vis: %.1f, opt: %.1f, res: %.1f, tot: %.1fms",
                     duration_cast<microseconds>(t2 - t1).count() / 1e3,
                     duration_cast<microseconds>(t3 - t2).count() / 1e3,
                     duration_cast<microseconds>(t4 - t3).count() / 1e3,
                     duration_cast<microseconds>(t5 - t4).count() / 1e3,
                     duration_cast<microseconds>(t6 - t5).count() / 1e3,
                     duration_cast<microseconds>(t6 - t1).count() / 1e3);
        }
    }
    return;
}

void FastLioSam::loopTimerCallback()
{
    auto &latest_keyframe = keyframes_.back();
    if (!is_initialized_ || keyframes_.empty() || latest_keyframe.processed_) { return; }
    latest_keyframe.processed_ = true;

    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    const inst closest_keyframe_idx = loop_closure_->fetchClosestKeyframeIdx(latest_keyframe, keyframes_);
    if (closest_keyframe_idx < 0)
    {
        return;
    }

    const RegistrationOutput &reg_output = loop_closure_->performLoopClosure(latest_keyframe, keyframes_, closest_keyframe_idx);
    if (reg_output.is_valid_)
    {
        ROS_INFO("\033[1;32mLoop closure accepted. Score: %.3f\033[0m", reg_output.score_);
        const auto &score = reg_output.score_;
        gtsam::Pose3 pose_from = poseEigToGtsamPose(reg_output.pose_between_eig_ * latest_keyframe.pose_corrected_eig_); // IMPORTANT: take care of the order
        gtsam::Pose3 pose_to = poseEigToGtsamPose(keyframes_[closest_keyframe_idx].pose_corrected_eig_);
        auto variance_vector = (gtsam::Vector(6) << score, score, score, score, score, score).finished();
        gtsam::noiseModel::Diagonal::shared_ptr loop_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
        {
            std::lock_guard<std::mutex> lock(graph_mutex_);
            gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(latest_keyframe.idx_,
                                                                closest_keyframe_idx,
                                                                pose_from.between(pose_to),
                                                                loop_noise));
        }
        loop_idx_pairs_.push_back({latest_keyframe.idx_, closest_keyframe_idx}); // for vis
        loop_added_flag_vis_ = true;
        loop_added_flag_ = true;
    }
    else
    {
        ROS_WARN("Loop closure rejected. Score: %.3f", reg_output.score_);
    }
    high_resolution_clock::time_point t2 = high_resolution_clock::now();

    debug_src_pub_->publish(pclToPclRos(loop_closure_->getSourceCloud(), map_frame_));
    debug_dst_pub_->publish(pclToPclRos(loop_closure_->getTargetCloud(), map_frame_));
    debug_fine_aligned_pub_->publish(pclToPclRos(loop_closure_->getFinalAlignedCloud(), map_frame_));

    RCLCPP_INFO(this->get_logger(), "loop: %.1f", duration_cast<microseconds>(t2 - t1).count() / 1e3);
    return;
}

void FastLioSam::visTimerCallback()
{
    if (!is_initialized_)
    {
        return;
    }

    high_resolution_clock::time_point tv1 = high_resolution_clock::now();
    //// 1. if loop closed, correct vis data
    if (loop_added_flag_vis_)
    // copy and ready
    {
        gtsam::Values corrected_esti_copied;
        pcl::PointCloud<pcl::PointXYZ> corrected_odoms;
        nav_msgs::msg::Path corrected_path;
        {
            std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
            corrected_esti_copied = corrected_esti_;
        }
        // correct pose and path
        for (size_t i = 0; i < corrected_esti_copied.size(); ++i)
        {
            gtsam::Pose3 pose_ = corrected_esti_copied.at<gtsam::Pose3>(i);
            corrected_odoms.points.emplace_back(pose_.translation().x(), pose_.translation().y(), pose_.translation().z());
            corrected_path.poses.push_back(gtsamPoseToPoseStamped(pose_, map_frame_));
        }
        // update vis of loop constraints
        if (!loop_idx_pairs_.empty())
        {
            loop_detection_pub_->publish(getLoopMarkers(corrected_esti_copied));
        }
        // update with corrected data
        {
            std::lock_guard<std::mutex> lock(vis_mutex_);
            corrected_odoms_ = corrected_odoms;
            corrected_path_.poses = corrected_path.poses;
        }
        loop_added_flag_vis_ = false;
    }
    //// 2. publish odoms, paths
    {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        odom_pub_->publish(pclToPclRos(odoms_, map_frame_));
        path_pub_->publish(odom_path_);
        corrected_odom_pub_->publish(pclToPclRos(corrected_odoms_, map_frame_));
        corrected_path_pub_->publish(corrected_path_);
    }

    //// 3. global map
    if (global_map_vis_switch_ && corrected_pcd_map_pub_->get_subscription_count() > 0) // save time, only once
    {
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
        corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size()); // it's an approximated size
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
            }
        }
        const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
        corrected_pcd_map_pub_->publish(pclToPclRos(*voxelized_map, map_frame_));
        global_map_vis_switch_ = false;
    }
    if (!global_map_vis_switch_ && corrected_pcd_map_pub_->get_subscription_count() == 0)
    {
        global_map_vis_switch_ = true;
    }
    high_resolution_clock::time_point tv2 = high_resolution_clock::now();
    RCLCPP_INFO(this->get_logger(), "vis: %.1fms", duration_cast<microseconds>(tv2 - tv1).count() / 1e3);
    return;
}

void FastLioSam::saveFlagCallback(const std_msgs::String::SharedPtr msg)
{
    std::string save_dir = msg->data != "" ? msg->data : package_path_;

    // save scans as individual pcd files and poses in KITTI format
    // Delete the scans folder if it exists and create a new one
    std::string seq_directory = save_dir + "/" + seq_name_;
    std::string scans_directory = seq_directory + "/scans";
    if (save_in_kitti_format_)
    {
        RCLCPP_INFO(this->get_logger(), "\033[32;1mScans are saved in %s, following the KITTI and TUM format\033[0m", scans_directory.c_str());
        if (fs::exists(seq_directory))
        {
            fs::remove_all(seq_directory);
        }
        fs::create_directories(scans_directory);

        std::ofstream kitti_pose_file(seq_directory + "/poses_kitti.txt");
        std::ofstream tum_pose_file(seq_directory + "/poses_tum.txt");
        tum_pose_file << "#timestamp x y z qx qy qz qw\n";
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                // Save the point cloud
                std::stringstream ss_;
                ss_ << scans_directory << "/" << std::setw(6) << std::setfill('0') << i << ".pcd";
                RCLCPP_INFO(this->get_logger(), "Saving %s...", ss_.str().c_str());
                pcl::io::savePCDFileASCII<PointType>(ss_.str(), keyframes_[i].pcd_);

                // Save the pose in KITTI format
                const auto &pose_ = keyframes_[i].pose_corrected_eig_;
                kitti_pose_file << pose_(0, 0) << " " << pose_(0, 1) << " " << pose_(0, 2) << " "
                                << pose_(0, 3) << " " << pose_(1, 0) << " " << pose_(1, 1) << " "
                                << pose_(1, 2) << " " << pose_(1, 3) << " " << pose_(2, 0) << " "
                                << pose_(2, 1) << " " << pose_(2, 2) << " " << pose_(2, 3) << "\n";

                const auto &lidar_optim_pose_ = poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_);
                tum_pose_file << std::fixed << std::setprecision(8) << keyframes_[i].timestamp_
                              << " " << lidar_optim_pose_.pose.position.x << " "
                              << lidar_optim_pose_.pose.position.y << " "
                              << lidar_optim_pose_.pose.position.z << " "
                              << lidar_optim_pose_.pose.orientation.x << " "
                              << lidar_optim_pose_.pose.orientation.y << " "
                              << lidar_optim_pose_.pose.orientation.z << " "
                              << lidar_optim_pose_.pose.orientation.w << "\n";
            }
        }
        kitti_pose_file.close();
        tum_pose_file.close();
        ROS_INFO("\033[32;1mScans and poses saved in .pcd and KITTI format\033[0m");
    }

    // if (save_map_bag_)
    // {
    //     rosbag::Bag bag;
    //     bag.open(package_path_ + "/result.bag", rosbag::bagmode::Write);
    //     {
    //         std::lock_guard<std::mutex> lock(keyframes_mutex_);
    //         for (size_t i = 0; i < keyframes_.size(); ++i)
    //         {
    //             ros::Time time;
    //             time.fromSec(keyframes_[i].timestamp_);
    //             bag.write("/keyframe_pcd", time, pclToPclRos(keyframes_[i].pcd_, map_frame_));
    //             bag.write("/keyframe_pose", time, poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_));
    //         }
    //     }
    //     bag.close();
    //     ROS_INFO("\033[36;1mResult saved in .bag format!!!\033[0m");
    // }

    // if (save_map_pcd_)
    // {
    //     pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
    //     corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size()); // it's an approximated size
    //     {
    //         std::lock_guard<std::mutex> lock(keyframes_mutex_);
    //         for (size_t i = 0; i < keyframes_.size(); ++i)
    //         {
    //             *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
    //         }
    //     }
    //     const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
    //     pcl::io::savePCDFileASCII<PointType>(seq_directory + "/" + seq_name_ + "_map.pcd", *voxelized_map);
    //     ROS_INFO("\033[32;1mAccumulated map cloud saved in .pcd format\033[0m");
    // }
}








void FastLioSam::updateOdomsAndPaths(const PosePcd &pose_pcd_in)
{
    odoms_.points.emplace_back(pose_pcd_in.pose_eig_(0, 3),
                               pose_pcd_in.pose_eig_(1, 3),
                               pose_pcd_in.pose_eig_(2, 3));
    corrected_odoms_.points.emplace_back(pose_pcd_in.pose_corrected_eig_(0, 3),
                                         pose_pcd_in.pose_corrected_eig_(1, 3),
                                         pose_pcd_in.pose_corrected_eig_(2, 3));
    odom_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_eig_, map_frame_));
    corrected_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig_, map_frame_));
    return;
}

bool FastLioSam::checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd)
{
    return keyframe_thr_ < (latest_pose_pcd.pose_corrected_eig_.block<3, 1>(0, 3) - pose_pcd_in.pose_corrected_eig_.block<3, 1>(0, 3)).norm();
}