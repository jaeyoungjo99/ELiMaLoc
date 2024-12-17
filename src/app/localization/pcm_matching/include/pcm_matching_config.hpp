/**
 * @file        pcm_matching_config.hpp
 * @brief       configuration hpp file for Point Cloud Map matching node
 * 
 * @authors     Jaeyoung Jo (wodud3743@gmail.com)          
 * 
 * @date        2024-11-08 created by Jaeyoung Jo
 * 
 */

#ifndef __PCM_MATCHING_CONFIG_HPP__
#define __PCM_MATCHING_CONFIG_HPP__
#pragma once

// STD Header
#include <string>
#include <vector>
#include <Eigen/Core>

typedef struct {  

    std::string s_lidar_type;
    bool b_lidar_scan_time_end;
    std::string s_lidar_topic_name;
    std::string s_imu_topic_name;
    std::vector<double> vec_d_ego_to_lidar_trans;
    std::vector<double> vec_d_ego_to_lidar_rot;
    std::vector<double> vec_d_ego_to_imu_rot;
    double d_lidar_time_delay;
    
    bool b_debug_print;
    
    std::string pcm_file_path;
    double d_pcm_voxel_size;
    int i_pcm_voxel_max_point;
    bool b_run_deskew;
    double d_input_max_dist;
    int i_input_index_sampling;
    double d_input_voxel_ds_m;
    double d_icp_pose_std_m;
    double d_icp_orientation_std_deg;

    Eigen::Matrix4d tf_ego_to_lidar;

} PcmMatchingConfig;

#endif // __PCM_MATCHING_CONFIG_HPP__