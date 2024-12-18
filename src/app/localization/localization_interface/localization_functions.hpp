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

// COUT macros
#define START_TIMER auto start_time_##__LINE__ = std::chrono::high_resolution_clock::now()
#define STOP_TIMER(name) std::cout << name << " Time: " \
                                  << GetDurationMs(start_time_##__LINE__) << " ms" << RESET << std::endl

#define START_TIMER_NAMED(timer_name) auto start_time_##timer_name = std::chrono::high_resolution_clock::now()
#define STOP_TIMER_NAMED(timer_name, name) std::cout << name << " Time: " \
                                          << GetDurationMs(start_time_##timer_name) << " ms" << RESET << std::endl


// cout color
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
 * Time measurement function (convert nanoseconds to milliseconds)
 * @param start_time start time
 * @return elapsed time (milliseconds)
 */
inline double GetDurationMs(const std::chrono::high_resolution_clock::time_point& start_time) {
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::high_resolution_clock::now() - start_time).count();
    return duration / 1e6;
}

/**
 * Convert IMU message angular velocity to ROS format
 * @param thisImuMsg IMU message
 * @param angular_x,y,z angular velocity of each axis (output)
 */
template <typename T>
inline void ImuAngular2RosAngular(sensor_msgs::Imu* thisImuMsg, T* angular_x, T* angular_y, T* angular_z) {
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

/**
 * Convert IMU data to vehicle coordinate system (only rotation)
 * @param imu_in input IMU message
 * @param rotation_calibration ego to IMU rotation matrix
 * @return ego-based IMU structure
 */
inline ImuStruct ImuStructConverter(const sensor_msgs::Imu& imu_in, const Eigen::Matrix3d& rotation_calibration) {
    // rotation_calibration 는 ego --> imu

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
 * Convert IMU data to vehicle coordinate system (considering both rotation and translation)
 * @param imu_in input IMU message
 * @param rotation_calibration rotation matrix from ego to IMU
 * @param translation_calibration translation vector from ego to IMU
 * @return ego-based IMU structure (including centrifugal effects)
 */
inline ImuStruct ImuStructConverter(const sensor_msgs::Imu& imu_in, 
                                  const Eigen::Matrix3d& rotation_calibration,
                                  const Eigen::Vector3d& translation_calibration) {
    // rotation_calibration: ego --> imu
    // translation_calibration: ego --> imu (ego coordinate system)
    // output imu is ego-based

    ImuStruct imu_out;
    imu_out.timestamp = imu_in.header.stamp.toSec();

    // imu --> ego rotation matrix 
    Eigen::Matrix3d R_imu_to_ego = rotation_calibration;

    // gyro
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);

    gyr = R_imu_to_ego * gyr;
    imu_out.gyro = gyr;

    // acceleration
    Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);

    // 1. basic rotation
    acc = R_imu_to_ego * acc;

    // 2. Add centrifugal effect (ω × (ω × r))
    Eigen::Vector3d centrifugal = gyr.cross(gyr.cross(-translation_calibration));

    // Final acceleration = rotated acceleration + centrifugal force
    imu_out.acc = acc + centrifugal;

    return imu_out;
}

/**
 * Convert IMU data to ROS message
 * @param imu_in input IMU message
 * @param rotation_calibration ego to IMU rotation matrix
 * @return converted ROS IMU message
 */
inline sensor_msgs::Imu ImuConverterToSensorMsg(const sensor_msgs::Imu& imu_in,
                                              const Eigen::Matrix3d& rotation_calibration) {
    // rotation_calibration 는 ego --> imu

    sensor_msgs::Imu imu_out = imu_in; 

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
 * Calculate interpolated transformation between two transformation matrices
 * @param affine_trans_between start and end transformation
 * @param dt_scan current time and start time difference
 * @param dt_trans end time and start time difference
 * @return interpolated transformation matrix
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
 * Normalize angle to 0~360 degrees
 * @param d_angle_deg input angle (degrees)
 * @return normalized angle (degrees)
 */
inline double NormAngleDeg(double d_angle_deg) {
    double d_angle_norm_deg = d_angle_deg;

    // Set the input angle into the 0~pi
    while (d_angle_norm_deg > 360.) d_angle_norm_deg -= 360.;
    while (d_angle_norm_deg < 0.) d_angle_norm_deg += 360.;

    return d_angle_norm_deg;
}

/**
 * Normalize angle to -π~π radians
 * @param d_angle_rad input angle (radians)
 * @return normalized angle (radians)
 */
inline double NormAngleRad(double d_angle_rad) {
    double d_angle_norm_rad = d_angle_rad;

    // Set the input angle into the 0~pi
    while (d_angle_norm_rad > M_PI) d_angle_norm_rad -= M_PI * 2.;
    while (d_angle_norm_rad < -M_PI) d_angle_norm_rad += M_PI * 2.;

    return d_angle_norm_rad;
}

/**
 * Calculate angle difference (-180~180 degrees)
 * @param d_ref_deg reference angle (degrees)
 * @param d_rel_deg relative angle (degrees)
 * @return angle difference (degrees)
 */
inline double AngleDiffDeg(double d_ref_deg, double d_rel_deg) {
    double d_angle_diff_deg = d_rel_deg - d_ref_deg;

    // calculate angle difference
    while (d_angle_diff_deg > 180.) d_angle_diff_deg = d_angle_diff_deg - 360.;
    while (d_angle_diff_deg < -180.) d_angle_diff_deg = d_angle_diff_deg + 360.;

    return d_angle_diff_deg;
}

/**
 * Calculate angle difference (-π~π radians)
 * @param d_ref_rad reference angle (radians)
 * @param d_rel_rad relative angle (radians)
 * @return angle difference (radians)
 */
inline double AngleDiffRad(double d_ref_rad, double d_rel_rad) {
    double d_angle_diff_rad = d_rel_rad - d_ref_rad;

    // calculate angle difference
    while (d_angle_diff_rad > M_PI) d_angle_diff_rad = d_angle_diff_rad - 2. * M_PI;
    while (d_angle_diff_rad < -M_PI) d_angle_diff_rad = d_angle_diff_rad + 2. * M_PI;

    return d_angle_diff_rad;
}


// Rotation Matrix to Euler angles (to avoid gimbal lock)
/**
 * Convert rotation matrix to Euler angles (to avoid gimbal lock)
 * @param R 3x3 rotation matrix
 * @return 3D vector containing roll, pitch, yaw
 */
inline Eigen::Vector3d RotToVec(const Eigen::Matrix3d& R) {
    Eigen::Vector3d angles;

    // Special case handling (to detect gimbal lock)
    if (std::abs(R(2, 0)) > 0.998) { // gimbal lock occurs
        angles(2) = std::atan2(-R(1, 2), R(1, 1));
        angles(1) = M_PI / 2 * (R(2, 0) >= 0 ? 1 : -1);
        angles(0) = 0;
    }
    else {
        angles(1) = std::asin(-R(2, 0));
        angles(0) = std::atan2(R(2, 1) / std::cos(angles(1)), R(2, 2) / std::cos(angles(1)));
        angles(2) = std::atan2(R(1, 0) / std::cos(angles(1)), R(0, 0) / std::cos(angles(1)));
    }

    // Normalize angles to be within -π and π
    angles(0) = std::fmod(angles(0) + M_PI, 2 * M_PI) - M_PI;
    angles(1) = std::fmod(angles(1) + M_PI, 2 * M_PI) - M_PI;
    angles(2) = std::fmod(angles(2) + M_PI, 2 * M_PI) - M_PI;

    return angles;
}

/**
 * Convert Euler angles to rotation matrix
 * @param angles 3D vector containing roll, pitch, yaw
 * @return 3x3 rotation matrix
 */
inline Eigen::Matrix3d VecToRot(const Eigen::Vector3d& angles) {
    Eigen::Matrix3d R = (Eigen::AngleAxisd(angles.z(), Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(angles.y(), Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(angles.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
    return R;
}


// Calculate angle difference between two quaternions
/**
 * Calculate angle difference between two quaternions
 * @param state_quat state quaternion
 * @param measurement_quat measurement quaternion
 * @return angle difference (radians)
 */
inline Eigen::Vector3d CalEulerResidualFromQuat(const Eigen::Quaterniond& state_quat,
                                                   const Eigen::Quaterniond& measurement_quat) {
    // Normalize quaternions to rotation matrices
    Eigen::Vector3d state_angles = RotToVec(state_quat.normalized().toRotationMatrix());
    Eigen::Vector3d meas_angles = RotToVec(measurement_quat.normalized().toRotationMatrix());

    // Calculate Euler angle residual
    Eigen::Vector3d res_euler = meas_angles - state_angles;

    // Normalize angles to be within -π and π
    res_euler.x() = NormAngleRad(res_euler.x());
    res_euler.y() = NormAngleRad(res_euler.y());
    res_euler.z() = NormAngleRad(res_euler.z());

    return res_euler;
}

/*
    Lie Algebra
*/
/**
 * Convert vector to skew-symmetric matrix
 * @param V 3D vector
 * @return 3x3 skew-symmetric matrix
 */
inline Eigen::Matrix3d SkewSymmetricMatrix(const Eigen::Vector3d& V) {
    Eigen::Matrix3d M;

    M << 0.0, -V(2), V(1), V(2), 0.0, -V(0), -V(1), V(0), 0.0;
    return M;
}

// SO(3) non-linear space --> so(3) linear space 
/**
 * SO(3) non-linear space --> so(3) linear space (log mapping)
 * @param R SO(3) rotation matrix
 * @return so(3) 3D vector
 */
inline Eigen::Vector3d Log(const Eigen::Matrix3d& R) {
    double cos_theta = (R.trace() - 1) / 2.0;
    cos_theta = std::min(1.0, std::max(-1.0, cos_theta));  // Clamping for numerical stability
    double theta = std::acos(cos_theta);
    
    if (std::abs(theta) < 1e-5) {
        return Eigen::Vector3d::Zero();
    }
    Eigen::Matrix3d log_matrix = (R - R.transpose()) / (2.0 * std::sin(theta));
    return theta * Eigen::Vector3d(log_matrix(2, 1), log_matrix(0, 2), log_matrix(1, 0));
}

/**
 * so(3) linear space --> SO(3) non-linear space (exp mapping)
 * @param omega so(3) 3D vector
 * @return SO(3) rotation matrix
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
 * Calculate rotation matrix change using angular velocity and time
 * @param gyro angular velocity vector
 * @param d_dt_sec time step
 * @return rotation matrix change
 */
inline Eigen::Matrix3d ExpGyroToRotMatrix(const Eigen::Vector3d& gyro, double d_dt_sec) {
    Eigen::Vector3d omega = gyro * d_dt_sec; // Angular velocity scaled by time step
    return Exp(omega); // Use the ExpMap function to get the rotation matrix
}


/**
 * Calculate quaternion change using angular velocity and time
 * @param gyro angular velocity vector
 * @param d_dt_sec time step
 * @return quaternion change
 */
inline Eigen::Quaterniond ExpGyroToQuat(const Eigen::Vector3d& gyro, double d_dt_sec) {
    Eigen::Vector3d omega = gyro * d_dt_sec; // Angular velocity vector scaled by time step
    Eigen::Matrix3d rotation_matrix = Exp(omega); // Use the Exp function
    return Eigen::Quaterniond(rotation_matrix); // Convert rotation matrix to quaternion
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
 * Calculate partial derivative of rotation with respect to angular velocity
 * @param gyro angular velocity vector
 * @param d_dt_sec time step
 * @return 3x3 Jacobian matrix
 */
inline Eigen::Matrix3d PartialDerivativeRotWrtGyro(const Eigen::Vector3d& gyro, double d_dt_sec) {

    Eigen::Vector3d omega = gyro * d_dt_sec; // angular velocity vector scaled by time step
    double theta = omega.norm(); // total angular velocity

    if (theta < 1e-5) {
        return Eigen::Matrix3d::Zero(); // Near-zero rotation, derivative is approximately zero
    }

    Eigen::Vector3d axis = omega / theta; // rotation axis = angular velocity vector / total angular velocity
    Eigen::Matrix3d K = SkewSymmetricMatrix(axis); // skew-symmetric matrix: rotation vector representation in SO(3)
    Eigen::Matrix3d partial_derivative = d_dt_sec * 
                                        (Eigen::Matrix3d::Identity() 
                                        + (1 - std::cos(theta)) / (theta * theta) * K 
                                        + (theta - std::sin(theta)) / (theta * theta * theta) * K * K);

    return partial_derivative;
}

/**
 * Convert global velocity to local velocity
 * @param global_vx,vy,vz global velocity
 * @param roll_rad,pitch_rad,yaw_rad current pose angles
 * @param local_vx,vy,vz local velocity (output)
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
 * Convert local angular velocity to global angular velocity
 * @param local_roll_rate,pitch_rate,yaw_rate local angular velocity
 * @param roll_rad,pitch_rad,yaw_rad current pose angles
 * @param global_roll_rate,pitch_rate,yaw_rate global angular velocity (output)
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

    // Relationship between Euler angular velocity and vehicle angular velocity
    // [global_roll_rate]   [1    0         -sin_pitch    ] [local_roll_rate ]
    // [global_pitch_rate] = [0  cos_roll   sin_roll*cos_pitch] [local_pitch_rate]
    // [global_yaw_rate  ]   [0 -sin_roll   cos_roll*cos_pitch] [local_yaw_rate  ]

    global_roll_rate = local_roll_rate + local_pitch_rate * 0 + local_yaw_rate * (-sin_pitch);

    global_pitch_rate = local_roll_rate * 0 + local_pitch_rate * cos_roll + local_yaw_rate * (sin_roll * cos_pitch);

    global_yaw_rate = local_roll_rate * 0 + local_pitch_rate * (-sin_roll) + local_yaw_rate * (cos_roll * cos_pitch);
}

/**
 * Convert global angular velocity to local angular velocity
 * @param global_roll_rate,pitch_rate,yaw_rate global angular velocity
 * @param roll_rad,pitch_rad,yaw_rad current pose angles
 * @param local_roll_rate,pitch_rate,yaw_rate local angular velocity (output)
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

    // Inverse matrix calculation for converting global angular velocity to local angular velocity
    // [local_roll_rate ]   [1    0          -sin_pitch   ]^-1 [global_roll_rate ]
    // [local_pitch_rate] = [0  cos_roll    sin_roll*cos_pitch]^-1 [global_pitch_rate]
    // [local_yaw_rate  ]   [0 -sin_roll    cos_roll*cos_pitch]^-1 [global_yaw_rate  ]

    const double det = cos_pitch * cos_roll;
    if (fabs(det) < 1e-6) {
        std::cerr << "Singularity encountered during angular rate conversion!" << std::endl;
        return;
    }

    // local angular velocity calculation
    local_roll_rate =
            global_roll_rate + global_pitch_rate * (sin_roll / cos_pitch) + global_yaw_rate * (-cos_roll / cos_pitch);

    local_pitch_rate = global_pitch_rate * cos_roll + global_yaw_rate * sin_roll;

    local_yaw_rate = global_pitch_rate * (-sin_roll / cos_pitch) + global_yaw_rate * (cos_roll / cos_pitch);
}

#endif
