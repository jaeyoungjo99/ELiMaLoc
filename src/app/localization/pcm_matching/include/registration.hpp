/****************************************************************************/
// Module:      registration.hpp
// Description: registration algorithm
//
// Authors: Jaeyoung Jo (wodud3743@gmail.com)
// Version: 0.1
//
// Revision History
//      Oct 18, 2024: Jaeyoung Jo - Created.
//      XXXX XX, 2023: XXXXXXX XX -
/****************************************************************************/

#ifndef __REGISTRATION_HPP__
#define __REGISTRATION_HPP__
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

// PCL
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/eigen.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/impl/angles.hpp>

#include "localization_functions.hpp"
#include "localization_struct.hpp"

#include "voxel_hash_map.hpp"

namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

using Matrix3_3d = Eigen::Matrix<double, 3, 3>;
using Matrix2_3d = Eigen::Matrix<double, 2, 3>;
using Matrix1_3d = Eigen::Matrix<double, 1, 3>;
using Matrix1_4d = Eigen::Matrix<double, 1, 4>;
using Matrix1_6d = Eigen::Matrix<double, 1, 6>;
using Matrix4_6d = Eigen::Matrix<double, 4, 6>;

// using Matrix3d = Eigen::Matrix<double, 3, 1>;
} // namespace Eigen

using Correspondences = std::vector<std::pair<PointStruct, PointStruct>>;

typedef enum { P2P, GICP, VGICP, AVGICP } IcpMethod;

struct RegistrationConfig {
    // Parameters
    int i_max_thread;
    IcpMethod icp_method;
    int voxel_search_method;
    double gicp_cov_search_dist;
    bool use_radar_cov;
    int max_iteration;
    double max_search_dist;
    double lm_lambda;
    double icp_termination_threshold_m;
    double min_overlap_ratio;
    double max_fitness_score;

    double doppler_trans_lambda;
    double range_variance_m;
    double azimuth_variance_deg;
    double elevation_variance_deg;
    Eigen::Vector3d ego_to_lidar_trans;
    Eigen::Matrix3d ego_to_lidar_rot;
    Eigen::Matrix3d ego_to_imu_rot;

    bool b_debug_print;
};

struct Velocity {
    Eigen::Vector3d linear;  // 선형 속도
    Eigen::Vector3d angular; // 각속도

    double time_diff_sec;

    // 기본 생성자Q - 멤버 변수를 0으로 초기화
    Velocity() : linear(Eigen::Vector3d::Zero()), angular(Eigen::Vector3d::Zero()), time_diff_sec(0.1) {}

    // 매개변수를 받아서 초기화하는 생성자
    Velocity(const Eigen::Vector3d& lin, const Eigen::Vector3d& ang, const double& time_diff)
        : linear(lin), angular(ang), time_diff_sec(time_diff) {}
};

struct Registration {
    Registration() {}
    Registration(RegistrationConfig config) { config_ = config; }
    void Init(RegistrationConfig config) { config_ = config; }

    Eigen::Matrix4d AlignCloudsLocal(std::vector<PointStruct>& source_global,
                                     const std::vector<PointStruct>& target_global, Eigen::Matrix4d& last_icp_pose,
                                     double trans_th, RegistrationConfig m_config);
    Eigen::Matrix4d AlignCloudsLocalSeperated(std::vector<PointStruct>& source_global,
                                              const std::vector<PointStruct>& target_global,
                                              Eigen::Matrix4d& last_icp_pose, double trans_th,
                                              RegistrationConfig m_config);
    Eigen::Matrix4d AlignCloudsLocalPointCov(std::vector<PointStruct>& source_global,
                                             const std::vector<PointStruct>& target_global, Eigen::Matrix6d& local_cov,
                                             Eigen::Matrix4d& last_icp_pose, double trans_th,
                                             RegistrationConfig m_config);
    Eigen::Matrix4d AlignCloudsLocalVoxelCov(std::vector<PointStruct>& source_global,
                                             const std::vector<CovStruct>& target_cov_global,
                                             Eigen::Matrix4d& last_icp_pose, double trans_th,
                                             RegistrationConfig m_config);
    Eigen::Matrix4d AlignCloudsGlobal(std::vector<PointStruct>& source_global,
                                      const std::vector<PointStruct>& target_global, double trans_th,
                                      RegistrationConfig m_config);

    // Local 좌표계의 frame과, voxel mmap, 초기위치 initial_guess
    Eigen::Matrix4d RunRegister(const std::vector<PointStruct>& source_local, const VoxelHashMap& voxel_map,
                                const Eigen::Matrix4d& initial_guess, RegistrationConfig m_config, bool& is_success,
                                double& fitness_score, Eigen::Matrix6d& local_cov);

    inline void TransformPoints(const Eigen::Matrix4d& T, std::vector<PointStruct>& points) {
        std::transform(points.cbegin(), points.cend(), points.begin(), [&](const auto& point) {
            Eigen::Vector4d p(point.pose.x(), point.pose.y(), point.pose.z(), 1.0);
            Eigen::Vector4d p_transformed = T * p;
            PointStruct transformed_point = point; // 모든 속성 복사
            transformed_point.pose = p_transformed.head<3>();
            return transformed_point;
        });
    }

    inline void TransformPoints(const Eigen::Matrix4d& T, const std::vector<PointStruct>& points,
                                std::vector<PointStruct>& o_points) {
        // assert(points.size() == o_points.size()); // points와 o_points가 같은 크기를 가지는지 확인
        o_points.resize(points.size());

        std::transform(points.cbegin(), points.cend(), o_points.begin(), [&](const auto& point) {
            Eigen::Vector4d p(point.pose.x(), point.pose.y(), point.pose.z(), 1.0);
            Eigen::Vector4d p_transformed = T * p;
            PointStruct transformed_point = point; // 모든 속성 복사
            transformed_point.pose = p_transformed.head<3>();
            return transformed_point;
        });
    }

    inline void SeperatePointsZ(const std::vector<PointStruct>& i_points, std::vector<PointStruct>& up_points,
                                std::vector<PointStruct>& down_points, double z) {
        // up_points와 down_points를 먼저 비웁니다.
        up_points.clear();
        down_points.clear();

        // i_points의 각 포인트를 순회하면서 z 좌표에 따라 분리합니다.
        for (const auto& point : i_points) {
            if (point.pose.z() > z) {
                up_points.push_back(point); // 기준보다 높으면 up_points에 추가
            }
            else {
                down_points.push_back(point); // 기준보다 낮거나 같으면 down_points에 추가
            }
        }
    }

    inline Velocity CalculateVelocity(const Eigen::Matrix4d& transform, double delta_t_sec) {
        // 회전 행렬 R
        Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
        // 평행 이동 벡터 t
        Eigen::Vector3d translation = transform.block<3, 1>(0, 3);

        // 선형 속도 v (t 변화를 시간으로 나눔)
        Eigen::Vector3d linear_vel = translation / delta_t_sec;

        // 각속도 행렬 omega 계산 (logarithm map 사용)
        Eigen::AngleAxisd angle_axis(rotation);
        Eigen::Vector3d angular_vel = angle_axis.angle() * angle_axis.axis() / delta_t_sec;

        // Velocity 구조체 생성 및 반환
        Velocity velocity(linear_vel, angular_vel, delta_t_sec);

        return velocity;
    }

    inline PointStruct CalPointCov(const PointStruct point, double range_var_m, double azim_var_deg,
                                   double ele_var_deg) {
        PointStruct cov_point = point;
        double dist = sqrt(point.pose.x() * point.pose.x() + point.pose.y() * point.pose.y());
        // double s_x = std::max(dist * range_var_m, 0.1);
        double s_x = range_var_m;
        double s_y = std::max(0.1, dist * sin(azim_var_deg / 180 * M_PI)); // 0.00873
        double s_z = std::max(0.1, dist * sin(ele_var_deg / 180 * M_PI));  // 0.01745

        double ele_angle = atan2(point.pose.z(), dist);
        double azi_angle = atan2(point.pose.y(), point.pose.x());
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(ele_angle, Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(azi_angle, Eigen::Vector3d::UnitZ()));
        Eigen::Matrix3d R; // Rotation matrix
        R = yawAngle * pitchAngle;

        Eigen::Matrix3d S; // Scaling matix
        S << s_x, 0.0, 0.0, 0.0, s_y, 0.0, 0.0, 0.0, s_z;

        Eigen::Matrix3d cov = R * S;
        cov_point.covariance.cov = cov;

        return cov_point;
    }

    inline void CalFramePointCov(std::vector<PointStruct>& points, double range_var_m, double azim_var_deg,
                                 double ele_var_deg) {
        tbb::parallel_for(tbb::blocked_range<size_t>(0, points.size()), [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i != range.end(); ++i) {
                points[i] = CalPointCov(points[i], range_var_m, azim_var_deg, ele_var_deg);
            }
        });
    }

    inline double square(double x) { return x * x; }

    inline Eigen::Matrix3d vectorToSkewSymmetricMatrix(const Eigen::Vector3d& vec) {
        Eigen::Matrix3d skew_symmetric;
        skew_symmetric << 0, -vec.z(), vec.y(), vec.z(), 0, -vec.x(), -vec.y(), vec.x(), 0;
        return skew_symmetric;
    }

    RegistrationConfig config_;

    double d_fitness_score_ = 0.0;
};

#endif // __REGISTRATION_HPP__