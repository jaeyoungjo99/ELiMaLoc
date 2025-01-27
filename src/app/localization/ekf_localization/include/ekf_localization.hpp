/**
 * @file ekf_localization.hpp
 * @author Jaeyoung Jo <wodud3743@gmail.com>
 * @brief Extended Kalman Filter for various state estimation
 * @version 0.1
 * @date 2024-10-28
 *
 * @copyright Copyright (c) 2024
 */

#ifndef __EKF_LOCALIZATION__
#define __EKF_LOCALIZATION__
#pragma once

// STD header
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// System Header
#include "ini_parser.h"

// ROS header
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

// Utility header
#include <boost/filesystem.hpp>

// InertialPoseLib
#include <InertialPoseLib/PoseEstimation.hpp>

// Algorithm header

#include "ekf_localization_config.hpp"
#include "localization_struct.hpp"
#include "localization_functions.hpp"

#include <GeographicLib/LocalCartesian.hpp>
#include "GeographicLib/UTMUPS.hpp"
#include "GeographicLib/Geocentric.hpp"

using namespace ros;
using namespace tf;
using namespace std;

class EkfLocalization {
public:
    // Constructor
    explicit EkfLocalization(std::string node_name, double period);
    // Destructor
    virtual ~EkfLocalization();

    void Init();
    void Run();
    void PublishInThread(); // 실제로 사용되는 publish 함수
    void ProcessINI();

    bool GnssTimeCompensation(const InertialPoseLib::GnssStruct& i_gnss, 
                                InertialPoseLib::GnssStruct& o_gnss);

    void UpdateEgoMarker(InertialPoseLib::EkfState ego_ekf_state);
    void UpdateGpsEgoMarker(InertialPoseLib::GnssStruct ego_gps_state);
    void UpdateTF(InertialPoseLib::EkfState ego_ekf_state);
    void UpdateEkfOdom(InertialPoseLib::EkfState ego_ekf_state);
    void UpdateGpsOdom(InertialPoseLib::GnssStruct gnss);
    void UpdateEkfText(const InertialPoseLib::EkfState ego_ekf_state);

    void CallbackNavsatFix(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void CallbackCAN(const geometry_msgs::TwistStampedConstPtr& msg);
    void CallbackImu(const sensor_msgs::Imu::ConstPtr& msg);
    void CallbackPcmOdom(const nav_msgs::Odometry::ConstPtr& msg);
    void CallbackPcmInitOdom(const nav_msgs::Odometry::ConstPtr& msg);

    Eigen::Vector3d ProjectGpsPoint(const double& lat, const double& lon, const double& height);

    void Exec(int num_thread=4);
    void MainLoop();

private:
    ros::Subscriber sub_navsatfix_;
    ros::Subscriber sub_can_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_pcm_odom_;
    ros::Subscriber sub_pcm_init_odom_;

    ros::Publisher pub_vehicle_state_;
    ros::Publisher pub_reference_;
    ros::Publisher pub_ekf_pose_odom_;
    ros::Publisher pub_ekf_ego_marker_;
    ros::Publisher pub_gps_ego_marker_;
    ros::Publisher pub_rviz_ekf_text_;

    ros::Publisher pub_x_plot_;
    ros::Publisher pub_y_plot_;
    ros::Publisher pub_z_plot_;
    ros::Publisher pub_vx_plot_;
    ros::Publisher pub_vy_plot_;
    ros::Publisher pub_vz_plot_;
    ros::Publisher pub_ax_plot_;
    ros::Publisher pub_ay_plot_;
    ros::Publisher pub_az_plot_;
    ros::Publisher pub_roll_deg_plot_;
    ros::Publisher pub_pitch_deg_plot_;
    ros::Publisher pub_yaw_deg_plot_;

    ros::Publisher pub_satellite_nav_fix_;

    ros::Publisher pub_gps_pose_odom_;

private: // algorithm
    InertialPoseLib::PoseEstimation pose_estimation_;
    InertialPoseLib::PoseEstimationParams pose_estimation_params_;

private: // config
    IniParser util_ini_parser_;
    IniParser util_calib_ini_parser_;
    EkfLocalizationConfig cfg_;

private: // mutex
    std::mutex mutex_vehicle_can_;
    std::mutex mutex_ekf_state_deque_;

private: // Variables
    // Data
    geometry_msgs::TwistStamped i_can_;

    std::deque<InertialPoseLib::EkfState> deq_ekf_state_;

    std::vector<InertialPoseLib::EkfState> vec_ego_state_;
    std::vector<std::pair<InertialPoseLib::EkfState, double>> vec_ekf_time_;

    bool b_bestpos_used_ = false;

    // msgs
    nav_msgs::Odometry o_ekf_pose_odom_;
    visualization_msgs::Marker o_ekf_ego_cov_marker_msgs_;
    jsk_rviz_plugins::OverlayText o_rviz_ekf_text_;

    nav_msgs::Odometry o_gps_pose_odom_;
    visualization_msgs::Marker o_gps_cov_marker_msgs_;

    tf::TransformBroadcaster tf_broadcaster_;

    std::string task_name_;
    double task_period_;
    double task_rate_;
    ros::Time update_time_;
    ros::Duration execution_time_;
};

#endif