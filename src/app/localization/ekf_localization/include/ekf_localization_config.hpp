/**
 * @file        ekf_localization_config.hpp
 * @brief       configuration hpp file for ekf localization node
 *
 * @authors     Jaeyoung Jo (wodud3743@gmail.com)
 *
 * @date        2024-11-08 created by Jaeyoung Jo
 *
 */

#ifndef __EKF_LOCALIZATION_CONFIG_HPP__
#define __EKF_LOCALIZATION_CONFIG_HPP__
#pragma once

// STD Header
#include <string>

enum class GpsType { NAVSATFIX = 0, ODOMETRY };

typedef struct {
    // Node
    bool b_debug_print = false;
    int i_vehicle_origin = 0;
    std::string projection_mode = "";
    double ref_latitude = 0.0;
    double ref_longitude = 0.0;
    double ref_altitude = 0.0;

    double gnss_uncertainy_max_m;

    std::string s_can_topic_name = "";
    std::string s_imu_topic_name = "";
    std::string s_navsatfix_topic_name = "";

    GpsType i_gps_type;
    bool b_use_gps;
    bool b_use_can;
    bool b_use_imu;
    bool b_use_pcm_matching;

    std::vector<double> vec_d_ego_to_imu_trans;
    std::vector<double> vec_d_ego_to_imu_rot;
    Eigen::Vector3d vec_ego_to_imu_trans;
    Eigen::Matrix3d mat_ego_to_imu_rot;

    std::vector<double> vec_d_ego_to_gps_trans;
    std::vector<double> vec_d_ego_to_gps_rot;
    Eigen::Vector3d vec_ego_to_gps_trans;
    Eigen::Matrix3d mat_ego_to_gps_rot;

    // Algorithm
    bool b_debug_imu_print;

    double d_can_vel_scale_factor;

    double d_imu_gravity;
    bool b_imu_estimate_gravity;
    bool b_imu_estimate_calibration;

    bool b_use_zupt;
    bool b_use_complementary_filter;

    double d_ekf_init_x_m;
    double d_ekf_init_y_m;
    double d_ekf_init_z_m;
    double d_ekf_init_roll_deg;
    double d_ekf_init_pitch_deg;
    double d_ekf_init_yaw_deg;

    double d_state_std_pos_m;
    double d_state_std_rot_deg;
    double d_state_std_vel_mps;
    double d_state_std_gyro_dps;
    double d_state_std_acc_mps;

    double d_imu_std_gyro_dps;
    double d_imu_std_acc_mps;

    double d_ekf_imu_bias_cov_gyro;
    double d_ekf_imu_bias_cov_acc;

    double d_ekf_gnss_min_cov_x_m;
    double d_ekf_gnss_min_cov_y_m;
    double d_ekf_gnss_min_cov_z_m;
    double d_ekf_gnss_min_cov_roll_deg;
    double d_ekf_gnss_min_cov_pitch_deg;
    double d_ekf_gnss_min_cov_yaw_deg;

    double d_ekf_can_meas_uncertainty_vel_mps;
    double d_ekf_can_meas_uncertainty_yaw_rate_deg;

    double d_ekf_bestvel_meas_uncertainty_vel_mps;

} EkfLocalizationConfig;

#endif // __EKF_LOCALIZATION_CONFIG_HPP__