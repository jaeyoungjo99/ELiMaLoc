/**
 * @file ekf_algorithm.hpp
 * @author Jaeyoung Jo <wodud3743@gmail.com>
 * @brief Extended Kalman Filter for various state estimation
 * @version 0.2
 * @date 2024-12-18
 *
 * @copyright Copyright (c) 2024
 */

#ifndef __EKF_ALGORITHM__
#define __EKF_ALGORITHM__
#pragma once

#include <Eigen/Dense>

#include "localization_functions.hpp"
#include "localization_struct.hpp"

#include "ekf_localization_config.hpp"

// STD header
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

// ROS header
#include <geometry_msgs/TwistStamped.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/filesystem.hpp>

#define S_X 0
#define S_Y 1
#define S_Z 2
#define S_ROLL 3
#define S_PITCH 4
#define S_YAW 5
#define S_VX 6
#define S_VY 7
#define S_VZ 8
#define S_ROLL_RATE 9
#define S_PITCH_RATE 10
#define S_YAW_RATE 11
#define S_AX 12
#define S_AY 13
#define S_AZ 14
#define S_B_ROLL_RATE 15
#define S_B_PITCH_RATE 16
#define S_B_YAW_RATE 17
#define S_B_AX 18
#define S_B_AY 19
#define S_B_AZ 20
#define S_G_X 21
#define S_G_Y 22
#define S_G_Z 23
#define S_IMU_ROLL 24
#define S_IMU_PITCH 25
#define S_IMU_YAW 26

#define STATE_ORDER 27

#define GNSS_MEAS_ORDER 6 // x y z roll pitch yaw
#define CAN_MEAS_ORDER 2  // v_local_x, yaw_rate
#define INIT_STATE_COV 100.0

#define DEBUG_PRINT_DT 1.0 // Printing cout dt

using namespace ros;
using namespace tf;
using namespace std;

class EkfAlgorithm {
public:
    EkfAlgorithm(EkfLocalizationConfig cfg);
    ~EkfAlgorithm();

    void Init();

    void UpdateDynamicConfig(EkfLocalizationConfig cfg);

    // EKF Functions
    bool RunPrediction(double cur_timestamp);

    bool RunPredictionImu(double cur_timestamp, ImuStruct imu_input);
    bool RunGnssUpdate(EkfGnssMeasurement gnss_input);
    bool RunCanUpdate(CanStruct can_input);

    bool IsStateInitialized() const { return b_state_initialized_; }
    bool IsYawInitialized() const { return b_yaw_initialized_; }
    bool IsRotationStabilized() const { return b_rotation_stabilized_; }
    bool IsStateStabilized() const { return b_state_stabilized_; }

    EgoState GetCurrentState();

    Eigen::Vector3d GetImuCalibration();

private:
    // Utils

    // ZUPT : Zero Velocity Potential Update
    void ZuptImu(ImuStruct imu_input);
    void ZuptCan(CanStruct can_input);

    // 상보 필터링 함수
    void ComplementaryKalmanFilter(ImuStruct imu_input);

    // Vehicle - IMU Calibration 함수
    void CalibrateVehicleToImu(ImuStruct imu_input);

    template <int MEAS_SIZE, int K_COLS>
    void UpdateEkfState(const Eigen::Matrix<double, STATE_ORDER, K_COLS>& K,    // 칼만 게인
                        const Eigen::Matrix<double, MEAS_SIZE, 1>& Y,           // 잔차
                        Eigen::Matrix<double, STATE_ORDER, STATE_ORDER>& P,     // 공분산 행렬
                        const Eigen::Matrix<double, MEAS_SIZE, STATE_ORDER>& H, // 관측 행렬
                        EkfState& X                                             // EKF 상태
    ) {
        // State update
        Eigen::Matrix<double, STATE_ORDER, 1> state_update = K * Y;
        X.pos += state_update.head<3>(); // Position update
        X.vel += state_update.block<3, 1>(S_VX, 0);
        X.gyro += state_update.block<3, 1>(S_ROLL_RATE, 0);
        X.acc += state_update.block<3, 1>(S_AX, 0);
        X.bg += state_update.block<3, 1>(S_B_ROLL_RATE, 0);
        X.ba += state_update.block<3, 1>(S_B_AX, 0);
        X.grav += state_update.block<3, 1>(S_G_X, 0);

        // Quaternion to rotation update
        Eigen::Vector3d rot_delta = state_update.segment<3>(3);
        Eigen::Quaterniond quat_delta(Eigen::AngleAxisd(rot_delta.norm(), rot_delta.normalized()));
        X.rot = (X.rot * quat_delta).normalized();

        // Quaternion to imu rotation
        Eigen::Vector3d imu_rot_delta = state_update.segment<3>(24);
        Eigen::Quaterniond imu_quat_delta(Eigen::AngleAxisd(imu_rot_delta.norm(), imu_rot_delta.normalized()));
        X.imu_rot = (X.imu_rot * imu_quat_delta).normalized();

        // Covariance update
        P = P - K * H * P;
    }

private: // state check
    void CheckStateInitialized() {
        if (sqrt(P_(S_ROLL, S_ROLL)) < 5.0 * M_PI / 180.0 && sqrt(P_(S_PITCH, S_PITCH)) < 5.0 * M_PI / 180.0 &&
            sqrt(P_(S_YAW, S_YAW)) < 5.0 * M_PI / 180.0 && sqrt(P_(S_X, S_X)) < 1.0 && sqrt(P_(S_Y, S_Y)) < 1.0) {
            if (b_state_initialized_ == false) {
                std::cout << GREEN << REVERSE << "STATE Initialized!" << RESET << std::endl;
                b_state_initialized_ = true;
            }
        }
        else {
            if (b_state_initialized_ == true) {
                std::cout << YELLOW << "STATE Not Initialized!" << RESET << std::endl;
                b_state_initialized_ = false;
            }
        }
    }

    void CheckYawInitialized() {
        if (sqrt(P_(S_YAW, S_YAW)) < 5.0 * M_PI / 180.0) {
            if (b_yaw_initialized_ == false) {
                std::cout << GREEN << REVERSE << "YAW Initialized!" << RESET << std::endl;
                b_yaw_initialized_ = true;
            }
        }
        else {
            if (b_yaw_initialized_ == true) {
                std::cout << YELLOW << "YAW Not Initialized!" << RESET << std::endl;
                b_yaw_initialized_ = false;
            }
        }
    }

    void CheckRotationStabilized() {
        if (sqrt(P_(S_ROLL, S_ROLL)) < 0.2 * M_PI / 180.0 && sqrt(P_(S_PITCH, S_PITCH)) < 0.2 * M_PI / 180.0 &&
            sqrt(P_(S_YAW, S_YAW)) < 0.2 * M_PI / 180.0) {
            if (b_rotation_stabilized_ == false) {
                std::cout << GREEN << REVERSE << "ROTATION Stabilized!" << RESET << std::endl;
                b_rotation_stabilized_ = true;
            }
        }
        else {
            if (b_rotation_stabilized_ == true) {
                std::cout << YELLOW << "ROTATION Unstabilized!" << RESET << std::endl;
                b_rotation_stabilized_ = false;
            }
        }
    }

    void CheckStateStabilized() {
        if (sqrt(P_(S_ROLL, S_ROLL)) < 0.2 * M_PI / 180.0 && sqrt(P_(S_PITCH, S_PITCH)) < 0.2 * M_PI / 180.0 &&
            sqrt(P_(S_YAW, S_YAW)) < 0.2 * M_PI / 180.0 && sqrt(P_(S_X, S_X)) < 0.5 && sqrt(P_(S_Y, S_Y)) < 0.5) {
            if (b_state_stabilized_ == false) {
                std::cout << GREEN << REVERSE << "STATE Stabilized!" << RESET << std::endl;
                b_state_stabilized_ = true;
            }
        }
        else {
            if (b_state_stabilized_ == true) {
                std::cout << YELLOW << "STATE Unstabilized!" << RESET << std::endl;
                b_state_stabilized_ = false;
            }
        }
    }

    void PrintState() {
        std::cout << RESET << "\n----------------------------------------" << std::endl;

        // GNSS Warning
        if (prev_timestamp_ - prev_gnss_.timestamp > 1.0) {
            std::cout << YELLOW << "GNSS Not Updated!" << RESET << std::endl;
        }

        // Print data setting ex: GPS: NavSatFix, CAN: O, PCM: X
        const char* gps_types[] = {"NavSatFix", "Odometry"};
        std::cout << "GPS: " << (cfg_.b_use_gps ? gps_types[(int)cfg_.i_gps_type] : "X") << ", ";
        std::cout << "CAN: " << (cfg_.b_use_can ? "O" : "X") << ", ";
        std::cout << "PCM: " << (cfg_.b_use_pcm_matching ? "O" : "X") << std::endl;

        // State init print
        if (b_state_initialized_ == false) {
            std::cout << YELLOW << "State Not Initialized!, " << RESET;
        }
        else {
            std::cout << GREEN << "State Init, " << RESET;
        }

        if (b_state_stabilized_ == false) {
            std::cout << YELLOW << "State Unstabilized!" << RESET << std::endl;
        }
        else {
            std::cout << GREEN << "State Stabilized" << RESET << std::endl;
        }

        std::cout << std::fixed << std::setprecision(3);
        std::cout << "State Std\n"
                  << "X: " << sqrt(P_(S_X, S_X)) << " Y: " << sqrt(P_(S_Y, S_Y)) << " Z: " << sqrt(P_(S_Z, S_Z))
                  << " m \n"
                  << "Roll: " << sqrt(P_(S_ROLL, S_ROLL)) * 180.0 / M_PI
                  << " Pitch: " << sqrt(P_(S_PITCH, S_PITCH)) * 180.0 / M_PI
                  << " Yaw: " << sqrt(P_(S_YAW, S_YAW)) * 180.0 / M_PI << " deg \n"
                  << std::endl;

        if (cfg_.b_imu_estimate_calibration) {
            std::cout << "IMU Calibration:\n"
                      << "Rot: " << RotToVec(S_.imu_rot.toRotationMatrix()).transpose() * 180.0 / M_PI << " deg\n"
                      << "Std: " << sqrt(P_(S_IMU_ROLL, S_IMU_ROLL)) * 180.0 / M_PI << " "
                      << sqrt(P_(S_IMU_PITCH, S_IMU_PITCH)) * 180.0 / M_PI << " "
                      << sqrt(P_(S_IMU_YAW, S_IMU_YAW)) * 180.0 / M_PI << " deg" << std::endl;
        }

        std::cout << std::fixed << std::setprecision(6);

        std::cout << "----------------------------------------" << std::endl;
    }

private: // config
    EkfLocalizationConfig cfg_;

private: // variables
    // mutex
    std::mutex mutex_state_;

    bool b_reset_for_init_prediction_ = true;

    bool b_state_initialized_ = false;
    bool b_yaw_initialized_ = false;
    bool b_rotation_stabilized_ = false;
    bool b_state_stabilized_ = false;

    bool b_pcm_init_on_going_ = false;
    bool b_vehicle_imu_calib_started_ = false;

    double d_can_yaw_rate_bias_rad_ = 0.0;

    EkfState S_;                                        // state
    Eigen::Matrix<double, STATE_ORDER, STATE_ORDER> P_; // covariance

    int i_pcm_update_count_ = 0;

    CanStruct prev_can_;
    EkfGnssMeasurement prev_gnss_;
    EgoState prev_ego_state_;
    double prev_timestamp_;
};

#endif