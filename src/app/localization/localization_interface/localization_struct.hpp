/**
 * @file localization_struct.hpp
 * @author Jaeyoung Jo <wodud3743@gmail.com>
 * @brief Various structs for EKF based localization
 * @version 0.1
 * @date 2024-10-28
 *
 * @copyright Copyright (c) 2024
 */

#ifndef __EKF_COMMON_STRUCT__
#define __EKF_COMMON_STRUCT__
#pragma once

// STD header
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Libraries
#include <Eigen/Dense>

enum class GnssSource { NOVATEL, NAVSATFIX, BESTPOS, PCM, PCM_INIT };

typedef struct {
    double timestamp;

    double latitude;
    double longitude;
    double height;

    double x_m;
    double y_m;
    double z_m; // new 1014

    double vx; // local coord
    double vy; // local coord
    double vz; // local coord

    double roll_vel;
    double pitch_vel;
    double yaw_vel;

    double ax; // local coord
    double ay; // local coord
    double az; // local coord

    double roll_rad;
    double pitch_rad;
    double yaw_rad;

    double latitude_std;  // world
    double longitude_std; // world
    double height_std;    // new 1014

    double x_cov_m; // ego
    double y_cov_m; // ego
    double z_cov_m; // ego new 1014

    double vx_cov_ms;
    double vy_cov_ms; // new 1014
    double vz_cov_ms; // new 1014

    double roll_cov_rad;  // new 1014
    double pitch_cov_rad; // new 1014
    double yaw_cov_rad;

} EgoState;

typedef struct {
    double timestamp;

    GnssSource gnss_source;

    double latitude;
    double longitude;
    double height;

    double x_m;
    double y_m;
    double z_m; // new 1014

    double vx;
    double vy;
    double vz;

    double roll_rad;
    double pitch_rad;
    double yaw_rad;

    double latitude_std;
    double longitude_std;
    double height_std;

    double x_cov_m; // local_x
    double y_cov_m; // local_y
    double z_cov_m; // local_y new 1014

    double vx_cov_ms;
    double vy_cov_ms; // new 1014
    double vz_cov_ms; // new 1014

    double roll_cov_rad;  // new 1014
    double pitch_cov_rad; // new 1014
    double yaw_cov_rad;
} GnssStruct;

typedef struct {
    double timestamp = 0.0;
    double vel_dir = 0.0; // velocity direction from East, Counterclockwise is positive
    double hor_vel = 0.0; // horizontal velocity
    double ver_vel = 0.0; // vertical velocity
} BestVelStruct;

typedef struct {
    double timestamp = 0.0;
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();  // local velocity, only x is valid
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero(); // local gyro, only z is valid
} CanStruct;

typedef struct {
    double timestamp = 0.0;
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
} ImuStruct;

// EKF state
typedef struct {
    double timestamp = 0.0;
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();           // global
    Eigen::Quaterniond rot = Eigen::Quaterniond::Identity(); // global
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();           // global
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();          // local
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();           // global
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();            // bias gyro
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();            // bias acc
    Eigen::Vector3d grav = Eigen::Vector3d::Zero();          // global gravity
    Eigen::Quaterniond imu_rot = Eigen::Quaterniond::Identity();
} EkfState;

typedef struct {
    double timestamp = 0.0;
    GnssSource gnss_source = GnssSource::NOVATEL;            // Define UNKNOWN as a default in GnssSource enum
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();           // global
    Eigen::Quaterniond rot = Eigen::Quaterniond::Identity(); // global
    Eigen::Matrix3d pos_cov = Eigen::Matrix3d::Identity();   // position covariance (3x3 matrix)
    Eigen::Matrix3d rot_cov = Eigen::Matrix3d::Identity();   // rotation covariance (3x3 matrix)
} EkfGnssMeasurement;

#endif