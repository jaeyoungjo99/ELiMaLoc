/**
 * @file localization_struct.hpp
 * @author Jaeyoung Jo <wodud3743@gmail.com>
 * @brief Various functions for EKF based localization
 * @version 0.1
 * @date 2024-10-28
 *
 * @copyright Copyright (c) 2024
 */

#pragma once
#ifndef _EKF_COMMON_FUNCTIONS_HPP_
#define _EKF_COMMON_FUNCTIONS_HPP_
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Header.h>

#include <opencv2/opencv.hpp> // for opencv4

#include <tf/tf.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// Libraries
#include <Eigen/Dense>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/parallel_for_each.h>

#include <tbb/global_control.h>
#include <tbb/task_arena.h>
#include <tbb/task_group.h>

#include <chrono>

#include "localization_struct.hpp"

// COUT 매크로
#define START_TIMER auto start_time_##__LINE__ = std::chrono::high_resolution_clock::now()
#define STOP_TIMER(name) std::cout << name << " Time: " \
                                  << GetDurationMs(start_time_##__LINE__) << " ms" << RESET << std::endl

#define START_TIMER_NAMED(timer_name) auto start_time_##timer_name = std::chrono::high_resolution_clock::now()
#define STOP_TIMER_NAMED(timer_name, name) std::cout << name << " Time: " \
                                          << GetDurationMs(start_time_##timer_name) << " ms" << RESET << std::endl


// cout 색상 표기를 위한 지정
const std::string RESET = "\033[0m";
const std::string RED = "\033[31m";     // ERROR
const std::string GREEN = "\033[32m";   // Update, Correction
const std::string YELLOW = "\033[33m";  // WARN
const std::string BLUE = "\033[34m";    // blue (PCM)
const std::string MAGENTA = "\033[35m"; // Purple (IMU)
const std::string CYAN = "\033[36m";    // mint (CAN)
const std::string WHITE = "\033[37m";   // white

const std::string BOLD = "\033[1m";
const std::string DIM = "\033[2m";
const std::string UNDERLINE = "\033[4m";
const std::string BLINK = "\033[5m";
const std::string REVERSE = "\033[7m";


/**
 * 시간 측정 함수 (나노초를 밀리초로 변환)
 * @param start_time 시작 시간
 * @return 경과 시간 (밀리초)
 */
inline double GetDurationMs(const std::chrono::high_resolution_clock::time_point& start_time) {
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::high_resolution_clock::now() - start_time).count();
    return duration / 1e6;
}

/**
 * IMU 메시지의 각속도를 ROS 형식으로 변환
 * @param thisImuMsg IMU 메시지
 * @param angular_x,y,z 각 축의 각속도 (출력)
 */
template <typename T>
inline void ImuAngular2RosAngular(sensor_msgs::Imu* thisImuMsg, T* angular_x, T* angular_y, T* angular_z) {
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

/**
 * IMU 데이터를 차량 좌표계로 변환 (회전만 고려)
 * @param imu_in 입력 IMU 메시지
 * @param rotation_calibration ego에서 IMU로의 회전 행렬
 * @return 차량 좌표계 기준 IMU 구조체
 */
inline ImuStruct ImuStructConverter(const sensor_msgs::Imu& imu_in, const Eigen::Matrix3d& rotation_calibration) {
    // rotation_calibration 는 ego --> imu
    // 출력 imu는 ego 기준

    ImuStruct imu_out;
    imu_out.timestamp = imu_in.header.stamp.toSec();

    // Rotate linear acceleration
    Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
    imu_out.acc = rotation_calibration * acc;

    // Rotate angular velocity
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    imu_out.gyro = rotation_calibration * gyr;

    return imu_out;
}

/**
 * IMU 데이터를 차량 좌표계로 변환 (회전과 병진 모두 고려)
 * @param imu_in 입력 IMU 메시지
 * @param rotation_calibration ego에서 IMU로의 회전 행렬
 * @param translation_calibration ego에서 IMU로의 변위 벡터
 * @return 차량 좌표계 기준 IMU 구조체 (원심력 효과 포함)
 */
inline ImuStruct ImuStructConverter(const sensor_msgs::Imu& imu_in, 
                                  const Eigen::Matrix3d& rotation_calibration,
                                  const Eigen::Vector3d& translation_calibration) {
    // rotation_calibration: ego --> imu
    // translation_calibration: ego --> imu (ego 좌표계 기준)
    // 출력 imu는 ego 기준

    ImuStruct imu_out;
    imu_out.timestamp = imu_in.header.stamp.toSec();

    // imu --> ego rotation matrix (rotation_calibration의 역행렬)
    Eigen::Matrix3d R_imu_to_ego = rotation_calibration;

    // 각속도 변환 (gyro)
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    // 회전 변환
    gyr = R_imu_to_ego * gyr;
    imu_out.gyro = gyr;

    // 가속도 변환 (acceleration)
    Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);

    // 1. 기본 회전 변환
    acc = R_imu_to_ego * acc;

    // 2. 원심력 효과 추가 (ω × (ω × r))
    Eigen::Vector3d centrifugal = gyr.cross(gyr.cross(-translation_calibration));

    // 최종 가속도 = 회전 변환된 가속도 + 원심력
    imu_out.acc = acc + centrifugal;

    return imu_out;
}

/**
 * IMU 데이터를 ROS 메시지로 변환
 * @param imu_in 입력 IMU 메시지
 * @param rotation_calibration ego에서 IMU로의 회전 행렬
 * @return 변환된 ROS IMU 메시지
 */
inline sensor_msgs::Imu ImuConverterToSensorMsg(const sensor_msgs::Imu& imu_in,
                                              const Eigen::Matrix3d& rotation_calibration) {
    // rotation_calibration 는 ego --> imu
    // 출력 imu는 ego 기준

    sensor_msgs::Imu imu_out = imu_in; // imu_in의 다른 필드값을 그대로 사용하기 위해 초기화

    // Rotate linear acceleration
    Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
    acc = rotation_calibration * acc;
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();

    // Rotate angular velocity
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    gyr = rotation_calibration * gyr;
    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();

    return imu_out;
}

/**
 * 두 변환 행렬 사이의 보간된 변환 계산
 * @param affine_trans_between 시작과 끝 변환 사이의 변환
 * @param dt_scan 현재 시간과 시작 시간의 차이
 * @param dt_trans 끝 시간과 시작 시간의 차이
 * @return 보간된 변환 행렬
 */
inline Eigen::Affine3f InterpolateTfWithTime(const Eigen::Affine3f& affine_trans_between, double dt_scan, double dt_trans) {
    // Check if dt_trans is zero to avoid division by zero
    if (dt_trans == 0.0) {
        return Eigen::Affine3f::Identity();
    }

    // Compute the interpolation ratio
    double ratio = dt_scan / dt_trans;

    // Interpolate translation part
    Eigen::Vector3f translation = affine_trans_between.translation() * ratio;

    // Interpolate rotation part using quaternion slerp (for smooth interpolation)
    Eigen::Quaternionf rotation(affine_trans_between.rotation());
    Eigen::Quaternionf interpolatedRotation = Eigen::Quaternionf::Identity().slerp(ratio, rotation);

    // Combine translation and rotation to create the final transformation
    Eigen::Affine3f affine_interpolated_transform = Eigen::Affine3f::Identity();
    affine_interpolated_transform.translate(translation);
    affine_interpolated_transform.rotate(interpolatedRotation);

    return affine_interpolated_transform;
}

/**
 * 각도를 0~360도 사이로 정규화
 * @param d_angle_deg 입력 각도(도)
 * @return 정규화된 각도(도)
 */
inline double NormAngleDeg(double d_angle_deg) {
    double d_angle_norm_deg = d_angle_deg;

    // Set the input angle into the 0~pi
    while (d_angle_norm_deg > 360.) d_angle_norm_deg -= 360.;
    while (d_angle_norm_deg < 0.) d_angle_norm_deg += 360.;

    return d_angle_norm_deg;
}

/**
 * 각도를 -π~π 사이로 정규화
 * @param d_angle_rad 입력 각도(라디안)
 * @return 정규화된 각도(라디안)
 */
inline double NormAngleRad(double d_angle_rad) {
    double d_angle_norm_rad = d_angle_rad;

    // Set the input angle into the 0~pi
    while (d_angle_norm_rad > M_PI) d_angle_norm_rad -= M_PI * 2.;
    while (d_angle_norm_rad < -M_PI) d_angle_norm_rad += M_PI * 2.;

    return d_angle_norm_rad;
}

/**
 * 두 각도의 차이 계산 (-180~180도 사이)
 * @param d_ref_deg 기준 각도(도)
 * @param d_rel_deg 상대 각도(도)
 * @return 각도 차이(도)
 */
inline double AngleDiffDeg(double d_ref_deg, double d_rel_deg) {
    double d_angle_diff_deg = d_rel_deg - d_ref_deg;

    // calculate angle difference
    while (d_angle_diff_deg > 180.) d_angle_diff_deg = d_angle_diff_deg - 360.;
    while (d_angle_diff_deg < -180.) d_angle_diff_deg = d_angle_diff_deg + 360.;

    return d_angle_diff_deg;
}

/**
 * 두 각도의 차이 계산 (-π~π 사이)
 * @param d_ref_rad 기준 각도(라디안)
 * @param d_rel_rad 상대 각도(라디안)
 * @return 각도 차이(라디안)
 */
inline double AngleDiffRad(double d_ref_rad, double d_rel_rad) {
    double d_angle_diff_rad = d_rel_rad - d_ref_rad;

    // calculate angle difference
    while (d_angle_diff_rad > M_PI) d_angle_diff_rad = d_angle_diff_rad - 2. * M_PI;
    while (d_angle_diff_rad < -M_PI) d_angle_diff_rad = d_angle_diff_rad + 2. * M_PI;

    return d_angle_diff_rad;
}


// Rotation Matrix를 오일러 각으로 변환 (짐벌락 예방)
/**
 * 회전 행렬을 오일러 각으로 변환 (짐벌락 방지)
 * @param R 3x3 회전 행렬
 * @return roll, pitch, yaw를 포함하는 3차원 벡터
 */
inline Eigen::Vector3d RotToVec(const Eigen::Matrix3d& R) {
    Eigen::Vector3d angles;

    // 특별한 경우 처리 (gimbal lock 감지)
    if (std::abs(R(2, 0)) > 0.998) { // gimbal lock이 발생한 경우
        angles(2) = std::atan2(-R(1, 2), R(1, 1));
        angles(1) = M_PI / 2 * (R(2, 0) >= 0 ? 1 : -1);
        angles(0) = 0;
    }
    else {
        angles(1) = std::asin(-R(2, 0));
        angles(0) = std::atan2(R(2, 1) / std::cos(angles(1)), R(2, 2) / std::cos(angles(1)));
        angles(2) = std::atan2(R(1, 0) / std::cos(angles(1)), R(0, 0) / std::cos(angles(1)));
    }

    // 각도가 -π에서 π 사이에 있도록 정규화
    angles(0) = std::fmod(angles(0) + M_PI, 2 * M_PI) - M_PI;
    angles(1) = std::fmod(angles(1) + M_PI, 2 * M_PI) - M_PI;
    angles(2) = std::fmod(angles(2) + M_PI, 2 * M_PI) - M_PI;

    return angles;
}

/**
 * 오일러 각을 회전 행렬로 변환
 * @param angles roll, pitch, yaw를 포함하는 3차원 벡터
 * @return 3x3 회전 행렬
 */
inline Eigen::Matrix3d VecToRot(const Eigen::Vector3d& angles) {
    // Eigen::AngleAxisd를 사용하여 회전 행렬 생성
    Eigen::Matrix3d R = (Eigen::AngleAxisd(angles.z(), Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(angles.y(), Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(angles.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();

    return R;
}


// 두 쿼터니언 사이의 오일러 각 차이를 계산
inline Eigen::Vector3d CalEulerResidualFromQuat(const Eigen::Quaterniond& state_quat,
                                                   const Eigen::Quaterniond& measurement_quat) {
    // 두 쿼터니언을 정규화하여 회전 행렬로 변환
    Eigen::Vector3d state_angles = RotToVec(state_quat.normalized().toRotationMatrix());
    Eigen::Vector3d meas_angles = RotToVec(measurement_quat.normalized().toRotationMatrix());

    // 오일러 각 잔차 계산
    Eigen::Vector3d res_euler = meas_angles - state_angles;

    // 각도를 정규화하여 -π와 π 사이로 유지
    res_euler.x() = NormAngleRad(res_euler.x());
    res_euler.y() = NormAngleRad(res_euler.y());
    res_euler.z() = NormAngleRad(res_euler.z());

    return res_euler;
}

/*
    Lie Algebra
*/
/**
 * 벡터를 반대칭 행렬(skew-symmetric matrix)로 변환
 * @param V 3차원 벡터
 * @return 3x3 반대칭 행렬
 */
inline Eigen::Matrix3d SkewSymmetricMatrix(const Eigen::Vector3d& V) {
    Eigen::Matrix3d M;

    M << 0.0, -V(2), V(1), V(2), 0.0, -V(0), -V(1), V(0), 0.0;
    return M;
}

// SO(3) non-linear space --> so(3) linear space 
/**
 * SO(3) 비선형 공간에서 so(3) 선형 공간으로 변환 (로그 매핑)
 * @param R SO(3) 회전 행렬
 * @return so(3) 3차원 벡터
 */
inline Eigen::Vector3d Log(const Eigen::Matrix3d& R) {
    double cos_theta = (R.trace() - 1) / 2.0;
    cos_theta = std::min(1.0, std::max(-1.0, cos_theta));  // 수치 안정성을 위한 클램핑
    double theta = std::acos(cos_theta);
    
    if (std::abs(theta) < 1e-5) {
        return Eigen::Vector3d::Zero();
    }
    Eigen::Matrix3d log_matrix = (R - R.transpose()) / (2.0 * std::sin(theta));
    return theta * Eigen::Vector3d(log_matrix(2, 1), log_matrix(0, 2), log_matrix(1, 0));
}

/**
 * so(3) 선형 공간에서 SO(3) 비선형 공간으로 변환 (지수 매핑)
 * @param omega so(3) 3차원 벡터
 * @return SO(3) 회전 행렬
 */
inline Eigen::Matrix3d Exp(const Eigen::Vector3d& omega) {
    double theta = omega.norm();
    Eigen::Matrix3d Eye3 = Eigen::Matrix3d::Identity();
    if (theta < 1e-5) {
        return Eye3;
    }
    Eigen::Vector3d axis = omega / theta;
    Eigen::Matrix3d K = SkewSymmetricMatrix(axis);
    return Eye3 + std::sin(theta) * K + (1 - std::cos(theta)) * K * K;
}

/**
 * 각속도와 시간을 이용한 회전 행렬 변화량 계산
 * @param gyro 각속도 벡터
 * @param d_dt_sec 시간 간격
 * @return 회전 행렬 변화량
 */
inline Eigen::Matrix3d ExpGyroToRotMatrix(const Eigen::Vector3d& gyro, double d_dt_sec) {
    Eigen::Vector3d omega = gyro * d_dt_sec; // Angular velocity scaled by time step
    return Exp(omega); // Use the ExpMap function to get the rotation matrix
}


/**
 * 각속도와 시간을 이용한 쿼터니언 변화량 계산
 * @param gyro 각속도 벡터
 * @param d_dt_sec 시간 간격
 * @return 쿼터니언 변화량
 */
inline Eigen::Quaterniond ExpGyroToQuat(const Eigen::Vector3d& gyro, double d_dt_sec) {
    Eigen::Vector3d omega = gyro * d_dt_sec; // 각속도 벡터에 시간 간격 곱하기
    Eigen::Matrix3d rotation_matrix = Exp(omega); // 기존 Exp 함수 사용
    return Eigen::Quaterniond(rotation_matrix); // 회전 행렬을 쿼터니언으로 변환
}

/*
    Given:
    δ_rot = Exp(gyro * Δt)
    ∂δ_rot / ∂ω = ∂Exp(ω) / ∂ω = ∂/∂ω (I + sin(θ) * K + (1 - cos(θ)) * K^2)
    ω = gyro * Δt
    θ = ||ω||

    The partial derivative of δ_rot with respect to gyro:
    
    ∂δ_rot / ∂gyro ≈ Δt * (I + (1 - cos(θ)) / θ^2 * K + (θ - sin(θ)) / θ^3 * K^2)

    where:
    - I is the identity matrix
    - K is the skew-symmetric matrix of ω/θ
*/
/**
 * 회전에 대한 각속도의 편미분 계산
 * @param gyro 각속도 벡터
 * @param d_dt_sec 시간 간격
 * @return 3x3 자코비안 행렬
 */
inline Eigen::Matrix3d PartialDerivativeRotWrtGyro(const Eigen::Vector3d& gyro, double d_dt_sec) {

    Eigen::Vector3d omega = gyro * d_dt_sec; // 각속도 벡터
    double theta = omega.norm(); // 각속도 총량

    if (theta < 1e-5) {
        return Eigen::Matrix3d::Zero(); // Near-zero rotation, derivative is approximately zero
    }

    Eigen::Vector3d axis = omega / theta; // 회전 축 = 각속도 벡터 / 각속도 총량
    Eigen::Matrix3d K = SkewSymmetricMatrix(axis); // 반대칭 행렬: SO(3) 에서 회전벡터 표현
    Eigen::Matrix3d partial_derivative = d_dt_sec * 
                                        (Eigen::Matrix3d::Identity() 
                                        + (1 - std::cos(theta)) / (theta * theta) * K 
                                        + (theta - std::sin(theta)) / (theta * theta * theta) * K * K);

    return partial_derivative;
}

/**
 * 전역 좌표계 속도를 로컬 좌표계 속도로 변환
 * @param global_vx,vy,vz 전역 좌표계 속도
 * @param roll_rad,pitch_rad,yaw_rad 현재 자세 각도
 * @param local_vx,vy,vz 로컬 좌표계 속도 (출력)
 */
inline void ConvertGlobalToLocalVelocity(const double global_vx, const double global_vy, const double global_vz,
                                         const double roll_rad, const double pitch_rad, const double yaw_rad,
                                         double& local_vx, double& local_vy, double& local_vz) {
    // Calculate rotation matrix elements
    const double cos_yaw = cos(yaw_rad);
    const double sin_yaw = sin(yaw_rad);
    const double cos_pitch = cos(pitch_rad);
    const double sin_pitch = sin(pitch_rad);
    const double cos_roll = cos(roll_rad);
    const double sin_roll = sin(roll_rad);

    // ZYX rotation matrix (transposed for global to local conversion)
    // First row determines local_vx
    local_vx = global_vx * (cos_yaw * cos_pitch) + global_vy * (sin_yaw * cos_pitch) + global_vz * (-sin_pitch);

    // Second row determines local_vy
    local_vy = global_vx * (cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll) +
               global_vy * (sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll) + global_vz * (cos_pitch * sin_roll);

    // Third row determines local_vz
    local_vz = global_vx * (cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll) +
               global_vy * (sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll) + global_vz * (cos_pitch * cos_roll);
}

/**
 * 로컬 좌표계 각속도를 전역 좌표계 각속도로 변환
 * @param local_roll_rate,pitch_rate,yaw_rate 로컬 각속도
 * @param roll_rad,pitch_rad,yaw_rad 현재 자세 각도
 * @param global_roll_rate,pitch_rate,yaw_rate 전역 각속도 (출력)
 */
inline void ConvertLocalToGlobalAngularRate(const double local_roll_rate, const double local_pitch_rate,
                                          const double local_yaw_rate, const double roll_rad,
                                          const double pitch_rad, const double yaw_rad,
                                          double& global_roll_rate, double& global_pitch_rate,
                                          double& global_yaw_rate)
{
    // 삼각함수 값 계산
    const double cos_roll = cos(roll_rad);
    const double sin_roll = sin(roll_rad);
    const double cos_pitch = cos(pitch_rad);
    const double sin_pitch = sin(pitch_rad);

    // 오일러 각속도와 차체 각속도의 관계식
    // [global_roll_rate]   [1    0         -sin_pitch    ] [local_roll_rate ]
    // [global_pitch_rate] = [0  cos_roll   sin_roll*cos_pitch] [local_pitch_rate]
    // [global_yaw_rate  ]   [0 -sin_roll   cos_roll*cos_pitch] [local_yaw_rate  ]

    global_roll_rate = local_roll_rate + local_pitch_rate * 0 + local_yaw_rate * (-sin_pitch);

    global_pitch_rate = local_roll_rate * 0 + local_pitch_rate * cos_roll + local_yaw_rate * (sin_roll * cos_pitch);

    global_yaw_rate = local_roll_rate * 0 + local_pitch_rate * (-sin_roll) + local_yaw_rate * (cos_roll * cos_pitch);
}

/**
 * 전역 좌표계 각속도를 로컬 좌표계 각속도로 변환
 * @param global_roll_rate,pitch_rate,yaw_rate 전역 각속도
 * @param roll_rad,pitch_rad,yaw_rad 현재 자세 각도
 * @param local_roll_rate,pitch_rate,yaw_rate 로컬 각속도 (출력)
 */
inline void ConvertGlobalToLocalAngularRate(const double global_roll_rate, const double global_pitch_rate,
                                          const double global_yaw_rate, const double roll_rad,
                                          const double pitch_rad, const double yaw_rad,
                                          double& local_roll_rate, double& local_pitch_rate,
                                          double& local_yaw_rate)
{
    // 삼각함수 값 계산
    const double cos_roll = cos(roll_rad);
    const double sin_roll = sin(roll_rad);
    const double cos_pitch = cos(pitch_rad);
    const double sin_pitch = sin(pitch_rad);

    // 글로벌 각속도를 로컬 각속도로 변환하기 위한 역행렬 계산
    // [local_roll_rate ]   [1    0          -sin_pitch   ]^-1 [global_roll_rate ]
    // [local_pitch_rate] = [0  cos_roll    sin_roll*cos_pitch]^-1 [global_pitch_rate]
    // [local_yaw_rate  ]   [0 -sin_roll    cos_roll*cos_pitch]^-1 [global_yaw_rate  ]

    // 변환 행렬의 행렬식을 이용하여 각속도 변환
    const double det = cos_pitch * cos_roll;
    if (fabs(det) < 1e-6) {
        std::cerr << "Singularity encountered during angular rate conversion!" << std::endl;
        return;
    }

    // 로컬 각속도 계산
    local_roll_rate =
            global_roll_rate + global_pitch_rate * (sin_roll / cos_pitch) + global_yaw_rate * (-cos_roll / cos_pitch);

    local_pitch_rate = global_pitch_rate * cos_roll + global_yaw_rate * sin_roll;

    local_yaw_rate = global_pitch_rate * (-sin_roll / cos_pitch) + global_yaw_rate * (cos_roll / cos_pitch);
}

#endif
