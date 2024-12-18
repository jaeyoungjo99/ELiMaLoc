/**
 * @file pcm_matching.hpp
 * @author Jaeyoung Jo <wodud3743@gmail.com>
 * @brief Pointcloud map matching algorithm with deskewing
 * @version 0.1
 * @date 2024-10-28
 *
 * @copyright Copyright (c) 2024
 */

#ifndef __PCM_MATCHING_HPP__
#define __PCM_MATCHING_HPP__
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
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

// Utility header
#include <boost/array.hpp>
#include <boost/filesystem.hpp>
// PCL
#include <pcl/common/common_headers.h>
#include <pcl/common/io.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/impl/passthrough.hpp>

#include <opencv2/opencv.hpp>

#include "localization_functions.hpp"
#include "localization_struct.hpp"
#include "pcm_matching_config.hpp"

// Algorithm Header
#include "registration.hpp"
#include "voxel_hash_map.hpp"

/* Types */
struct PointXYZIT {
    PCL_ADD_POINT4D;
    float intensity;
    float time;                     // point time after scan start
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT, (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                                      intensity)(float, time, time))

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint16_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t, t)(
                                          uint16_t, reflectivity,
                                          reflectivity)(uint16_t, ring, ring)(uint16_t, ambient, ambient)(uint32_t,
                                                                                                          range, range))

using namespace ros;
using namespace tf;
using namespace std;

typedef pcl::PointXYZINormal PointType;
const int i_queue_length_ = 2000;

class PcmMatching {
public:
    explicit PcmMatching(std::string node_name, double period);
    virtual ~PcmMatching();

    void Init();
    void Run();
    void Publish();
    void ProcessINI();

    void CallbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void CallbackImu(const sensor_msgs::Imu::ConstPtr &msg);
    void CallbackEkfState(const nav_msgs::Odometry::ConstPtr &msg);
    void CallbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

    void Exec(int num_thread=4);
    void MainLoop();

private: // Deskewing
    bool DeskewPointCloud(const pcl::PointCloud<PointXYZIT>::Ptr &distort_pcptr, double timestamp);
    void ImuDeskewInfo();
    void OdomDeskewInfo();
    void FindRotation(double d_point_time, float *f_rot_x_cur, float *f_rot_y_cur, float *f_rot_z_cur);
    void FindPosition(double d_rel_time, float *f_pos_x_cur, float *f_pos_y_cur, float *f_pos_z_cur);
    PointType DeskewPoint(PointType *pcl_point, double d_rel_time);

private: // Visualization
    void SortEigenvaluesAndEigenvectors(Eigen::Vector3d &vec_eigenvalues, Eigen::Matrix3d &mat_eigenvectors);
    void VisualizeCovMap(ros::Publisher &marker_pub, const std::vector<CovStruct> &vec_cov_map, ros::Time thisStamp,
                         std::string thisFrame);
    void PublishPcmOdom(Eigen::Matrix4d icp_ego_pose, ros::Time thisStamp, std::string thisFrame);
    void BroadcastStaticTf(ros::Time thisStamp, std::string lidarFrame, std::string imuFrame);

private: // utils
    pcl::PointCloud<PointXYZIT> OusterCloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg);
    pcl::PointCloud<PointXYZIT> Cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg);

    bool GetInterpolatedPose(double d_time_scan_cur_, Eigen::Affine3f &o_transform);
    void FilterPointsByDistance(pcl::PointCloud<PointXYZIT>::Ptr &input_pcptr);

    template <typename T>
    sensor_msgs::PointCloud2 PublishCloud(const ros::Publisher &thisPub, const T &thisCloud, ros::Time thisStamp,
                                          std::string thisFrame) {
        sensor_msgs::PointCloud2 tempCloud;
        pcl::toROSMsg(*thisCloud, tempCloud);
        tempCloud.header.stamp = thisStamp;
        tempCloud.header.frame_id = thisFrame;
        thisPub.publish(tempCloud);
        return tempCloud;
    }

    void PointStruct2Pcl(const std::vector<PointStruct> &vec_points, pcl::PointCloud<PointType>::Ptr &pcl_points,
                         bool b_cal_norm = false) {
        pcl_points->clear();
        pcl_points->points.resize(vec_points.size());

        // 각 PointStruct를 pcl_points에 직접 설정
        for (size_t i = 0; i < vec_points.size(); ++i) {
            const auto &point_struct = vec_points[i];
            PointType &pcl_point = pcl_points->points[i];

            pcl_point.x = static_cast<float>(point_struct.pose.x());
            pcl_point.y = static_cast<float>(point_struct.pose.y());
            pcl_point.z = static_cast<float>(point_struct.pose.z());
            pcl_point.intensity = static_cast<float>(point_struct.intensity);

            if (b_cal_norm == true) {
                const CovStruct &cov_data = vec_points[i].covariance;

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_solver(cov_data.cov);
                Eigen::Vector3d eigenvalues = eig_solver.eigenvalues();
                Eigen::Matrix3d eigenvectors = eig_solver.eigenvectors();

                // sort eigenvalues and eigenvectors by size
                SortEigenvaluesAndEigenvectors(eigenvalues, eigenvectors);

                // set color based on eigenvector direction
                Eigen::Vector3d main_axis = eigenvectors.col(2); // main axis (eigenvector corresponding to largest eigenvalue)
                pcl_point.normal_x = fabs(main_axis(0));         // reflect X-axis component in red
                pcl_point.normal_y = fabs(main_axis(1));         // reflect Y-axis component in green
                pcl_point.normal_z = fabs(main_axis(2));         // reflect Z-axis component in blue
            }
        }

        // update size and properties
        pcl_points->width = pcl_points->points.size();
        pcl_points->height = 1;
        pcl_points->is_dense = true;
    }

    void Pcl2PointStruct(const pcl::PointCloud<PointType>::Ptr &pcl_points, std::vector<PointStruct> &vec_points) {
        // allocate memory for vector to minimize memory allocation
        vec_points.clear();
        vec_points.reserve(pcl_points->points.size());

        // convert point cloud data to PointStruct and insert into vector
        for (const auto &pcl_point : pcl_points->points) {
            PointStruct point_struct;
            point_struct.pose = Eigen::Vector3d(pcl_point.x, pcl_point.y, pcl_point.z);
            point_struct.local = point_struct.pose; // set same value
            point_struct.intensity = pcl_point.intensity;

            vec_points.emplace_back(std::move(point_struct));
        }

    }

    Eigen::Vector3d NormalizeDiagonalCovariance(const Eigen::Matrix3d &rotation_covariance) {
        // Extract the diagonal elements as a vector
        Eigen::Vector3d diag = rotation_covariance.diagonal();

        // Find the smallest diagonal value
        double min_diag = diag.minCoeff();

        // Avoid division by zero or excessively small values
        const double min_threshold = 1e-9;
        if (min_diag < min_threshold) {
            diag *= 1e9; // Scale up all elements
            min_diag = diag.minCoeff();
            if (min_diag < min_threshold) {
                min_diag = min_threshold; // Ensure the minimum threshold is respected
            }
        }

        // Normalize each diagonal value by the smallest value
        Eigen::Vector3d normalized = diag / min_diag;

        // Clamp values to a reasonable range
        const double max_cap = 5.0;
        const double min_cap = 1.0;
        normalized = normalized.cwiseMin(max_cap).cwiseMax(min_cap);

        return normalized;
    }

    Eigen::Matrix3d NormalizeCovariance(const Eigen::Matrix3d &i_covariance) {
        // Copy input covariance to a local matrix
        Eigen::Matrix3d i_cov = i_covariance;

        // Find the smallest diagonal value
        double min_diag = std::min({i_cov(0, 0), i_cov(1, 1), i_cov(2, 2)});

        // Ensure the minimum value is not too small to avoid division by zero
        const double min_threshold = 1e-9;
        if (min_diag <= min_threshold) {
            i_cov *= 1e9;

            min_diag = std::min({i_cov(0, 0), i_cov(1, 1), i_cov(2, 2)});
            if (min_diag < min_threshold) min_diag = min_threshold;
        }

        // Normalize all elements of the covariance matrix
        Eigen::Matrix3d norm_cov = i_cov / min_diag;
        // Cap the maximum value for each element in the matrix
        const double max_cap = 5.0; // Set your desired maximum cap
        norm_cov = norm_cov.unaryExpr([max_cap](double elem) { return std::min(elem, max_cap); });

        return norm_cov;
    }

    void UpdateCovarianceField(boost::array<double, 36> &covariance, const Eigen::Matrix3d &translation_cov_norm_mat,
                               const Eigen::Matrix3d &rotation_cov_norm_mat) {
        // Assign translation covariance to the top-left 3x3 block
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                covariance[row * 6 + col] = translation_cov_norm_mat(row, col);
            }
        }

        // Assign rotation covariance to the bottom-right 3x3 block
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                covariance[(row + 3) * 6 + (col + 3)] = rotation_cov_norm_mat(row, col);
            }
        }
    }

    // config
private:
    IniParser util_ini_parser_;
    IniParser util_calib_ini_parser_;
    PcmMatchingConfig cfg_;
    RegistrationConfig registration_config_;

    // ROS I/O
private:
    ros::Subscriber sub_point_cloud_;
    ros::Subscriber sub_ekf_geo_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_initial_pose_;

    ros::Publisher pub_pcm_gnss_;
    ros::Publisher pub_pcm_odom_;
    ros::Publisher pub_pcm_init_odom_;
    ros::Publisher pub_pcm_init_pc_;
    ros::Publisher pub_undistort_pc_;
    ros::Publisher pub_voxel_map_pc_;
    ros::Publisher pub_voxel_map_cov_;
    ros::Publisher pub_icp_map_pc_;
    ros::Publisher pub_icp_full_map_pc_;

    nav_msgs::Odometry i_ekf_state_;
    sensor_msgs::PointCloud2 i_lidar_point_cloud_msg_;

    pcl::PointCloud<PointType>::Ptr o_undistort_pcptr_;

    std::deque<EgoState> deq_ego_state_struct_;
    pcl::PointCloud<PointXYZIT>::Ptr i_lidar_pcptr_;
    std::tuple<pcl::PointCloud<PointXYZIT>::Ptr, std::string, ros::Time> i_lidar_tuple_;

    pcl::PointCloud<PointType>::Ptr o_voxel_map_pcptr_;
    sensor_msgs::PointCloud2 o_voxel_map_msg_;

    pcl::PointCloud<PointType>::Ptr o_icp_map_pcptr_;
    pcl::PointCloud<PointType>::Ptr o_icp_full_map_pcptr_;
    sensor_msgs::PointCloud2 o_icp_map_pc_msg_;

    // Data avialable flag
    bool b_is_imu_available_ = false;
    bool b_is_odom_available_ = false;

    Eigen::Matrix6d icp_local_cov_;

    // mutex
private:
    std::mutex mutex_imu_;
    std::mutex mutex_odom_;
    std::mutex mutex_pcl_;

    bool b_initializing_status_ = false;
    bool b_get_first_odom_ = false;
    // tbb control
private:
    std::unique_ptr<tbb::global_control> tbb_control_;

private: // deskew
    std::deque<sensor_msgs::Imu> deq_imu_;
    std::deque<nav_msgs::Odometry> deq_odom_;

    bool b_first_point_ = true;
    int i_imu_pointer_cur_;
    float f_odom_incre_x_;
    float f_odom_incre_y_;
    float f_odom_incre_z_;
    Eigen::Affine3f mat_trans_start_inverse_;
    double d_time_scan_cur_;
    double d_time_scan_end_;

    double *vec_d_imu_time_ = new double[i_queue_length_];
    double *vec_d_imu_rot_x_ = new double[i_queue_length_];
    double *vec_d_imu_rot_y_ = new double[i_queue_length_];
    double *vec_d_imu_rot_z_ = new double[i_queue_length_];

    pcl::PointCloud<PointType>::Ptr undistort_pcptr_;

private: // map matching
    VoxelHashMap local_map_;
    Registration registration_;

    std::string task_name_;
    double task_period_;
    double task_rate_;
    ros::Time update_time_;
    ros::Duration execution_time_;
};

#endif // __PCM_MATCHING_HPP__