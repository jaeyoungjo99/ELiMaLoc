/**
 * @file ekf_algorithm.cpp
 * @author Jaeyoung Jo <wodud3743@gmail.com>
 * @brief Extended Kalman Filter for various state estimation
 * @version 0.1
 * @date 2024-10-28
 *
 * @copyright Copyright (c) 2024
 */

#include "ekf_algorithm.hpp"

EkfAlgorithm::EkfAlgorithm(EkfLocalizationConfig cfg) : b_reset_for_init_prediction_(true) {
    cfg_ = cfg;
    memset(&prev_ego_state_, 0, sizeof(EkfState));
    memset(&prev_gnss_, 0, sizeof(EkfGnssMeasurement));
    memset(&prev_can_, 0, sizeof(CanStruct));
    prev_timestamp_ = 0.0;
}
EkfAlgorithm::~EkfAlgorithm() {}

void EkfAlgorithm::Init() {
    std::lock_guard<std::mutex> lock(mutex_state_);

    std::cout << REVERSE << "EKF Algorithm Init Start" << RESET << std::endl;

    // 상태 초기화 (State initialization using configuration)
    S_.pos = Eigen::Vector3d(cfg_.d_ekf_init_x_m, cfg_.d_ekf_init_y_m, cfg_.d_ekf_init_z_m);
    S_.rot = Eigen::AngleAxisd(cfg_.d_ekf_init_yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(cfg_.d_ekf_init_pitch_deg * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(cfg_.d_ekf_init_roll_deg * M_PI / 180.0, Eigen::Vector3d::UnitX());
    S_.vel.setZero();
    S_.gyro.setZero();
    S_.acc.setZero();
    S_.bg.setZero();
    S_.ba.setZero();
    S_.grav = Eigen::Vector3d(0.0, 0.0, cfg_.d_imu_gravity);

    // 공분산 초기화 (Covariance initialization using configuration)
    P_ = Eigen::MatrixXd::Identity(STATE_ORDER, STATE_ORDER) * INIT_STATE_COV; // 대각선을 INIT_STATE_COV로 초기화

    // 바이어스 항목 초기화 값 설정
    P_(S_B_ROLL_RATE, S_B_ROLL_RATE) = cfg_.d_ekf_imu_bias_cov_gyro;
    P_(S_B_PITCH_RATE, S_B_PITCH_RATE) = cfg_.d_ekf_imu_bias_cov_gyro;
    P_(S_B_YAW_RATE, S_B_YAW_RATE) = cfg_.d_ekf_imu_bias_cov_gyro;
    P_(S_B_AX, S_B_AX) = cfg_.d_ekf_imu_bias_cov_acc;
    P_(S_B_AY, S_B_AY) = cfg_.d_ekf_imu_bias_cov_acc;
    P_(S_B_AZ, S_B_AZ) = cfg_.d_ekf_imu_bias_cov_acc;

    // 중력 항목 초기화 값 설정
    P_(S_G_X, S_G_X) = 1;
    P_(S_G_Y, S_G_Y) = 1;
    P_(S_G_Z, S_G_Z) = 1;

    P_(S_IMU_ROLL, S_IMU_ROLL) = cfg_.d_ekf_imu_bias_cov_gyro;
    P_(S_IMU_PITCH, S_IMU_PITCH) = cfg_.d_ekf_imu_bias_cov_gyro;
    P_(S_IMU_YAW, S_IMU_YAW) = cfg_.d_ekf_imu_bias_cov_gyro;

    b_reset_for_init_prediction_ = true;
    b_yaw_initialized_ = false;
    b_state_initialized_ = false;
    b_rotation_stabilized_ = false;
    b_state_stabilized_ = false;
    b_pcm_init_on_going_ = false;
    b_vehicle_imu_calib_started_ = false;

    std::cout << REVERSE << "EKF Algorithm Init Done" << RESET << std::endl;
}

void EkfAlgorithm::UpdateDynamicConfig(EkfLocalizationConfig cfg) {
    cfg_.b_debug_print = cfg.b_debug_print;
    cfg_.b_debug_imu_print = cfg.b_debug_imu_print;

    cfg_.b_use_zupt = cfg.b_use_zupt;
    cfg_.b_use_complementary_filter = cfg.b_use_complementary_filter;

    cfg_.b_use_gps = cfg.b_use_gps;
    cfg_.i_gps_type = cfg.i_gps_type;
    cfg_.b_use_can = cfg.b_use_can;
    cfg_.b_use_pcm_matching = cfg.b_use_pcm_matching;
}

bool EkfAlgorithm::RunPrediction(double cur_timestamp) {
    if (b_reset_for_init_prediction_ == true) {
        prev_timestamp_ = cur_timestamp;

        b_reset_for_init_prediction_ = false;
        return false;
    }

    if (b_pcm_init_on_going_ == true) {
        std::cout << YELLOW << "[RunPrediction] PCM Intializing on going.. Do not predict." << RESET << std::endl;

        prev_timestamp_ = cur_timestamp;
        return false;
    }

    if (fabs(cur_timestamp - prev_timestamp_) < 1e-6) {
        if (cfg_.b_debug_print) std::cout << "[RunPrediction] Not New Prediction data" << std::endl;
        return false; // Not new data
    }

    // prediction dt 계산
    double d_dt = cur_timestamp - prev_timestamp_;

    // dt print
    if (cfg_.b_debug_print) {
        std::cout << "[RunPrediction] Pred dt ms: " << d_dt * 1000.0 << std::endl;
    }

    EkfState ekf_state_prev = S_;

    // x, y, z
    S_.pos += ekf_state_prev.vel * d_dt + 0.5 * ekf_state_prev.acc * d_dt * d_dt;

    // roll, pitch, yaw
    // delta_angles를 쿼터니언 회전으로 직접 변환하여 적용
    Eigen::Quaterniond delta_rot = ExpGyroToQuat(ekf_state_prev.gyro, d_dt);
    S_.rot = (ekf_state_prev.rot * delta_rot).normalized(); // 회전 적용 및 정규화

    // vx, vy, vz
    S_.vel += ekf_state_prev.acc * d_dt;

    // Angular rates prediction
    S_.gyro = ekf_state_prev.gyro;
    S_.acc = ekf_state_prev.acc;
    S_.bg = ekf_state_prev.bg;
    S_.ba = ekf_state_prev.ba;
    S_.grav = ekf_state_prev.grav;

    // -----------------------------------------------------------------------
    // Q matrix initialization
    Eigen::Matrix<double, STATE_ORDER, STATE_ORDER> Q = Eigen::Matrix<double, STATE_ORDER, STATE_ORDER>::Zero();

    // Q 에 분산 할당 (표준편차의 제곱)
    Q.block<3, 3>(S_X, S_X) = Eigen::Matrix3d::Identity() * std::pow(cfg_.d_state_std_pos_m, 2) * d_dt * d_dt;
    Q.block<3, 3>(S_ROLL, S_ROLL) =
            Eigen::Matrix3d::Identity() * std::pow(cfg_.d_state_std_rot_deg * M_PI / 180.0, 2) * d_dt * d_dt;
    Q.block<3, 3>(S_VX, S_VX) = Eigen::Matrix3d::Identity() * std::pow(cfg_.d_state_std_vel_mps, 2) * d_dt * d_dt;
    Q.block<3, 3>(S_ROLL_RATE, S_ROLL_RATE) =
            Eigen::Matrix3d::Identity() * std::pow(cfg_.d_state_std_gyro_dps, 2) * d_dt * d_dt;
    Q.block<3, 3>(S_AX, S_AX) = Eigen::Matrix3d::Identity() * std::pow(cfg_.d_state_std_acc_mps, 2) * d_dt * d_dt;

    // -----------------------------------------------------------------------
    // Covariance (P) propagation using CV model

    // Compute the Jacobian matrix (F) of CV model
    Eigen::Matrix<double, STATE_ORDER, STATE_ORDER> F = Eigen::Matrix<double, STATE_ORDER, STATE_ORDER>::Identity();

    // Position derivatives
    F.block<3, 3>(S_X, S_VX) = Eigen::Matrix3d::Identity() * d_dt; // dx/dvx, dy/dvy, dz/dvz

    // Attitude derivatives
    F.block<3, 3>(S_ROLL, S_ROLL_RATE) =
            Eigen::Matrix3d::Identity() * d_dt; // droll/droll_rate, dpitch/dpitch_rate, dyaw/dyaw_rate

    // Acceleration and position integration
    F.block<3, 3>(S_X, S_AX) = Eigen::Matrix3d::Identity() * 0.5 * d_dt * d_dt; // dx/dax, dy/day, dz/daz
    F.block<3, 3>(S_VX, S_AX) = Eigen::Matrix3d::Identity() * d_dt;             // dvx/dax, dvy/day, dvz/daz

    // Covariance matrix
    P_ = F * P_ * F.transpose() + Q;

    prev_timestamp_ = cur_timestamp;

    return true;
}

bool EkfAlgorithm::RunPredictionImu(double cur_timestamp, ImuStruct imu_input) {
    std::unique_lock<std::mutex> lock(mutex_state_, std::try_to_lock);
    if (lock.owns_lock() == false) {
        // 이 lock이 점유하지 못했다면(다른 process가 state에 접근중이라면) prediction을 수행하지 않음
        // Measurement Update 우선 원칙
        return false;
    }

    // State 상태 출력
    static double last_print_time = 0.0;
    if (cur_timestamp - last_print_time > DEBUG_PRINT_DT) {
        PrintState();
        last_print_time = cur_timestamp;
    }

    if (b_reset_for_init_prediction_ == true) {
        prev_timestamp_ = cur_timestamp;

        b_reset_for_init_prediction_ = false;
        return false;
    }

    if (b_pcm_init_on_going_ == true) {
        std::cout << BLUE << "[RunPredictionImu] PCM Intializing on going.. Do not predict." << RESET << std::endl;

        prev_timestamp_ = cur_timestamp;
        return false;
    }

    CheckRotationStabilized();

    if (IsStateInitialized() == false) {
        prev_timestamp_ = cur_timestamp;

        // Complementary Filter는 Yaw가 초기화 되어야, Roll Pitch 추정 가능
        if (IsYawInitialized() == true && (cfg_.i_gps_type == GpsType::BESTPOS || cfg_.b_use_complementary_filter)) {
            ComplementaryKalmanFilter(imu_input);
        }

        return false;
    }

    if (fabs(cur_timestamp - prev_timestamp_) < 1e-6) {
        if (cfg_.b_debug_print) std::cout << "[RunPredictionImu] Not New Prediction data" << std::endl;
        return false; // Not new data
    }

    // prediction dt 계산
    double d_dt = cur_timestamp - prev_timestamp_;

    // dt print
    if (cfg_.b_debug_print) {
        if (d_dt > 0.02) {
            std::cout << YELLOW << "[RunPredictionImu] Pred dt ms: " << d_dt * 1000.0 << RESET << std::endl;
        }
        else {
            std::cout << "[RunPredictionImu] Pred dt ms: " << d_dt * 1000.0 << std::endl;
        }
    }

    EkfState ekf_state_prev = S_;

    // State rotation quat to matrix
    Eigen::Matrix3d G_R_I = S_.rot.toRotationMatrix();

    // Gyroscope reading corrected for bias
    Eigen::Vector3d corrected_gyro = imu_input.gyro - ekf_state_prev.bg;
    Eigen::Quaterniond delta_rot = ExpGyroToQuat(corrected_gyro, d_dt);
    S_.rot = (ekf_state_prev.rot * delta_rot).normalized();

    // Comensate IMU Bias and Gravity
    Eigen::Vector3d corrected_accel = imu_input.acc - ekf_state_prev.ba;
    Eigen::Vector3d accel_global = G_R_I * corrected_accel - ekf_state_prev.grav;

    // Predict Pose, Velocity
    S_.pos += ekf_state_prev.vel * d_dt + 0.5 * accel_global * d_dt * d_dt;
    S_.vel += accel_global * d_dt;

    // Use prior state of gyro and acc
    S_.gyro = corrected_gyro;
    S_.acc = accel_global;

    // Use prior state of bias and gravity
    S_.bg = ekf_state_prev.bg;
    S_.ba = ekf_state_prev.ba;
    S_.grav = ekf_state_prev.grav;

    // Process model matrix (Q) generation
    Eigen::Matrix<double, STATE_ORDER, STATE_ORDER> Q = Eigen::Matrix<double, STATE_ORDER, STATE_ORDER>::Zero();

    // Allocate std in Q matrix
    Q.block<3, 3>(S_X, S_X) = Eigen::Matrix3d::Identity() * std::pow(cfg_.d_state_std_pos_m, 2) * d_dt * d_dt;
    Q.block<3, 3>(S_ROLL, S_ROLL) =
            Eigen::Matrix3d::Identity() * std::pow(cfg_.d_state_std_rot_deg * M_PI / 180.0, 2) * d_dt * d_dt;
    Q.block<3, 3>(S_VX, S_VX) = Eigen::Matrix3d::Identity() * std::pow(cfg_.d_state_std_vel_mps, 2) * d_dt * d_dt;
    Q.block<3, 3>(S_ROLL_RATE, S_ROLL_RATE) =
            Eigen::Matrix3d::Identity() * std::pow(cfg_.d_imu_std_gyro_dps * M_PI / 180.0, 2) * d_dt * d_dt;
    Q.block<3, 3>(S_AX, S_AX) = Eigen::Matrix3d::Identity() * std::pow(cfg_.d_imu_std_acc_mps, 2) * d_dt * d_dt;
    Q.block<3, 3>(S_B_ROLL_RATE, S_B_ROLL_RATE) =
            Eigen::Matrix3d::Identity() * std::pow(cfg_.d_ekf_imu_bias_cov_gyro, 2) * d_dt * d_dt;
    Q.block<3, 3>(S_B_AX, S_B_AX) =
            Eigen::Matrix3d::Identity() * std::pow(cfg_.d_ekf_imu_bias_cov_acc, 2) * d_dt * d_dt;
    Q.block<3, 3>(S_G_X, S_G_X) = Eigen::Matrix3d::Identity() * std::pow(cfg_.d_imu_std_acc_mps, 2) * d_dt * d_dt;
    Q.block<3, 3>(S_IMU_ROLL, S_IMU_ROLL) =
            Eigen::Matrix3d::Identity() * std::pow(cfg_.d_state_std_rot_deg * M_PI / 180.0, 2) * d_dt * d_dt;

    // Covariance (P) propagation using CV model
    Eigen::Matrix<double, STATE_ORDER, STATE_ORDER> F = Eigen::Matrix<double, STATE_ORDER, STATE_ORDER>::Identity();

    // Position partial differentiation
    F.block<3, 3>(S_X, S_VX) = Eigen::Matrix3d::Identity() * d_dt; // ∂pos / ∂vel
    // F.block<3, 3>(S_X, S_AX) = 0.5 * G_R_I * d_dt * d_dt;     // ∂pos / ∂acc (global transform 포함) // FIXME: 의미 없는 편미분
    F.block<3, 3>(S_X, S_B_AX) = -0.5 * G_R_I * d_dt * d_dt; // ∂pos / ∂ba (global transform 포함)

    // Rotation partial differentiation
    // F.block<3, 3>(S_ROLL, S_ROLL_RATE) =   0.5 * d_dt * Eigen::Matrix3d::Identity(); // d rot / d gyro  // FIXME: 의미 없는 편미분
    F.block<3, 3>(S_ROLL, S_B_ROLL_RATE) = -PartialDerivativeRotWrtGyro(corrected_gyro, d_dt); // d rot / d bg

    // Velocity partial differentiation
    // F.block<3, 3>(S_VX, S_AX) = G_R_I * d_dt;             // ∂vel / ∂acc (global transform 포함) // FIXME: 의미 없는 편미분
    F.block<3, 3>(S_VX, S_B_AX) = -G_R_I * d_dt; // ∂vel / ∂ba (global transform 포함)
    F.block<3, 3>(S_ROLL_RATE, S_B_ROLL_RATE) = -Eigen::Matrix3d::Identity(); // ∂gyro / ∂bg
    F.block<3, 3>(S_AX, S_B_AX) = -G_R_I; // ∂acc / ∂ba (global transform 포함)

    if (cfg_.b_imu_estimate_gravity) {
        // Only Z axis
        F(S_Z, S_G_Z) = -0.5 * d_dt * d_dt; // ∂z / ∂gz
        F(S_VZ, S_G_Z) = -d_dt;             // ∂vz / ∂gz
        F(S_AZ, S_G_Z) = -1.0;              // ∂az / ∂gz
    }

    // Covariance matrix
    P_ = F * P_ * F.transpose() + Q;

    prev_timestamp_ = cur_timestamp;

    // IMU State Debugging
    if (cfg_.b_debug_imu_print) {
        std::cout << "Bias Rate X: " << S_.bg(0) << " Y: " << S_.bg(1) << " Z: " << S_.bg(2) << std::endl;
        std::cout << "Bias ACC  X: " << S_.ba(0) << " Y: " << S_.ba(1) << " Z: " << S_.ba(2) << std::endl;
        std::cout << "Gravity   X: " << S_.grav(0) << " Y: " << S_.grav(1) << " Z: " << S_.grav(2) << std::endl;
    }

    if (cfg_.b_use_zupt) ZuptImu(imu_input);
    if (cfg_.i_gps_type == GpsType::BESTPOS || cfg_.b_use_complementary_filter) ComplementaryKalmanFilter(imu_input);
    if (cfg_.b_imu_estimate_calibration) CalibrateVehicleToImu(imu_input);

    return true;
}

bool EkfAlgorithm::RunGnssUpdate(EkfGnssMeasurement gnss_input) {
    std::lock_guard<std::mutex> lock(mutex_state_);

    double gnss_dt = gnss_input.timestamp - prev_gnss_.timestamp;

    // PCM Init Pose 로 State 강제 초기화
    if (gnss_input.gnss_source == GnssSource::PCM_INIT) {
        // 상태 초기화
        S_.pos = gnss_input.pos;
        S_.rot = gnss_input.rot;
        S_.vel.setZero();
        S_.gyro.setZero();
        S_.acc.setZero();
        S_.bg.setZero();
        S_.ba.setZero();
        S_.grav = Eigen::Vector3d(0.0, 0.0, cfg_.d_imu_gravity);

        P_ = Eigen::MatrixXd::Identity(STATE_ORDER, STATE_ORDER) * INIT_STATE_COV; // 대각선을 INIT_STATE_COV로 초기화

        std::cout << GREEN << REVERSE << "[RunGnssUpdate] GPS Status Initialized by Init Pose!" << RESET << std::endl;

        // PCM Initialized를 통해 상태를 강제로 변경 했으므로 ekf state가 초기화 된 것으로 간주
        b_state_initialized_ = true;
        b_yaw_initialized_ = true;

        // 향후 일정 시간동안, pcm update만을 통해 상태 업데이트 (Prediction 미수행)
        b_pcm_init_on_going_ = true;

        return true;
    }

    CheckYawInitialized();
    CheckStateInitialized();
    CheckRotationStabilized();
    CheckStateStabilized();

    // // EKF 초기화 이전에는 PCM 업데이트 수행하지 않음 (Map matching 발산 위험)
    // if (IsStateInitialized() == false && gnss_input.gnss_source == GnssSource::PCM) {
    //     std::cout << YELLOW << "[RunGnssUpdate] GPS Status not initialized for PCM" << RESET << std::endl;
    //     return false;
    // }

    // PCM 초기화 이후 일정 시간동안, PCM Update만 수행하여 EKF 초기화 시도
    if (b_pcm_init_on_going_ == true && gnss_input.gnss_source == GnssSource::PCM) {
        if (i_pcm_update_count_ > 10) {
            b_pcm_init_on_going_ = false;
            std::cout << BLUE << REVERSE << "[RunGnssUpdate] PCM Status Initialized!" << RESET << std::endl;
        }
        i_pcm_update_count_++;
    }

    // GNSS 소스 유형 출력
    std::string gnss_type;
    gnss_type = gnss_input.gnss_source == GnssSource::NOVATEL
                        ? "NOVATEL"
                        : gnss_input.gnss_source == GnssSource::NAVSATFIX
                                  ? "NAVSATFIX"
                                  : gnss_input.gnss_source == GnssSource::PCM ? "PCM" : "";

    if (cfg_.b_debug_print) {
        std::cout << GREEN << "[RunGnssUpdate] " << gnss_type << " dt: " << gnss_dt * 1000.0 << " ms" << RESET
                  << std::endl;
    }

    // Main Algorithm

    // Observation matrix: H
    Eigen::Matrix<double, GNSS_MEAS_ORDER, STATE_ORDER> H;
    H.setZero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); // Position
    H.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity(); // Rotation

    // Measurement model: Z_state (state position and rotation)
    Eigen::Matrix<double, GNSS_MEAS_ORDER, 1> Z_state;
    Z_state.head<3>() = S_.pos; // Position

    // Measurement vector: Z (from gnss_input)
    Eigen::Matrix<double, GNSS_MEAS_ORDER, 1> Z;
    Z.head<3>() = gnss_input.pos;

    // Measurement covariance matrix: R
    Eigen::Matrix<double, GNSS_MEAS_ORDER, GNSS_MEAS_ORDER> R =
            Eigen::Matrix<double, GNSS_MEAS_ORDER, GNSS_MEAS_ORDER>::Zero();
    R.block<3, 3>(0, 0) = gnss_input.pos_cov;
    R.block<3, 3>(3, 3) = gnss_input.rot_cov;

    // Add extra uncertainty for GNSS Source
    if (gnss_input.gnss_source == GnssSource::NOVATEL || gnss_input.gnss_source == GnssSource::BESTPOS ||
        gnss_input.gnss_source == GnssSource::NAVSATFIX) {
        R(0, 0) += cfg_.d_ekf_gnss_min_cov_x_m;
        R(1, 1) += cfg_.d_ekf_gnss_min_cov_y_m;
        R(2, 2) += cfg_.d_ekf_gnss_min_cov_z_m;
        R(3, 3) += cfg_.d_ekf_gnss_min_cov_roll_deg * M_PI / 180.0;
        R(4, 4) += cfg_.d_ekf_gnss_min_cov_pitch_deg * M_PI / 180.0;
        R(5, 5) += cfg_.d_ekf_gnss_min_cov_yaw_deg * M_PI / 180.0;
    }

    // Observation covariance matrix: S (GNSS_MEAS_ORDER, GNSS_MEAS_ORDER)
    Eigen::Matrix<double, GNSS_MEAS_ORDER, GNSS_MEAS_ORDER> S = H * P_ * H.transpose() + R;

    // Kalman Gain: K (STATE_ORDER * GNSS_MEAS_ORDER)
    Eigen::Matrix<double, STATE_ORDER, GNSS_MEAS_ORDER> K = P_ * H.transpose() * S.inverse();

    // Residual: Y (for position and rotation in quaternion)
    Eigen::Vector3d res_angle_euler = CalEulerResidualFromQuat(S_.rot, gnss_input.rot);

    Eigen::Matrix<double, GNSS_MEAS_ORDER, 1> Y;
    Y.head<3>() = Z.head<3>() - Z_state.head<3>(); // Position residual
    Y.tail<3>() = res_angle_euler;                 // Rotation residual as angle-axis vector

    if (gnss_input.gnss_source == GnssSource::NAVSATFIX || gnss_input.gnss_source == GnssSource::BESTPOS) {
        // 안테나 기준 pose를 제공하는 BESTPOS, NAVSATFIX 특성상, yaw가 잡히지 않으면 calibration 오류가 발생
        if (IsYawInitialized() == false) {
            R(0, 0) += 3.0; // TODO: 실제 Vehicle to Antenna 오차 반영
            R(1, 1) += 3.0; // TODO: 실제 Vehicle to Antenna 오차 반영
        }

        Eigen::Matrix<double, 3, STATE_ORDER> H3 = H.block<3, STATE_ORDER>(0, 0);
        Eigen::Matrix<double, 3, 3> S3 = H3 * P_ * H3.transpose() + R.block<3, 3>(0, 0);
        Eigen::Matrix<double, STATE_ORDER, 3> K3 = P_ * H3.transpose() * S3.inverse();
        Eigen::Matrix<double, 3, 1> Y3 = Y.head<3>();

        UpdateEkfState(K3, Y3, P_, H3, S_);
    }
    else {
        UpdateEkfState(K, Y, P_, H, S_);
    }

    prev_gnss_ = gnss_input;
    return true;
}

bool EkfAlgorithm::RunCanUpdate(CanStruct can_input) {
    std::lock_guard<std::mutex> lock(mutex_state_);

    double can_dt = can_input.timestamp - prev_can_.timestamp;

    // if (IsStateInitialized() == false) {
    //     // Orientation 초기화 안됐으면 CAN 사용 불가
    //     std::cout << YELLOW << "[RunCanUpdate] GPS-IMU Not init yet." << RESET << std::endl;
    //     return false;
    // }

    // 이전 CAN 과의 시간 간격 유지
    if (fabs(can_dt) < 0.01) {
        return false;
    }

    if (cfg_.b_debug_print) {
        std::cout << CYAN << "[RunCanUpdate] dt: " << can_dt * 1000.0 << " ms" << RESET << std::endl;
    }

    // Remove sensor bias from can input
    CanStruct unbiased_can = can_input;
    unbiased_can.gyro.z() -= d_can_yaw_rate_bias_rad_;
    unbiased_can.vel.x() *= cfg_.d_can_vel_scale_factor;

    // CAN 속도를 전역 좌표계로 변환
    Eigen::Vector3d can_vel_global = S_.rot * unbiased_can.vel;

    // Observation matrix: H (only updates local x-velocity and yaw rate)
    Eigen::Matrix<double, 4, STATE_ORDER> H;
    H.setZero();
    H(0, S_VX) = 1.0;
    H(1, S_VY) = 1.0;
    H(2, S_VZ) = 1.0;
    H(3, S_YAW_RATE) = 1.0;

    // Measurement model: Z_state (state velocity in local x and yaw rate)
    Eigen::Matrix<double, 4, 1> Z_state;
    Z_state(0) = S_.vel.x();
    Z_state(1) = S_.vel.y();
    Z_state(2) = S_.vel.z();
    Z_state(3) = S_.gyro.z(); // current yaw rate

    // Measurement vector: Z (from can_input)
    Eigen::Matrix<double, 4, 1> Z;
    Z(0) = can_vel_global.x();    // measured x-velocity in global frame
    Z(1) = can_vel_global.y();    // measured x-velocity in global frame
    Z(2) = can_vel_global.z();    // measured x-velocity in global frame
    Z(3) = unbiased_can.gyro.z(); // measured yaw rate in local frame

    // Measurement covariance matrix: R
    Eigen::Matrix<double, 4, 4> R = Eigen::Matrix<double, 4, 4>::Zero();

    Eigen::Matrix<double, 3, 3> R_local = Eigen::Matrix<double, 3, 3>::Zero();
    R_local(0, 0) = pow(cfg_.d_ekf_can_meas_uncertainty_vel_mps, 2);     // velocity measurement noise local x
    R_local(1, 1) = pow(cfg_.d_ekf_can_meas_uncertainty_vel_mps * 2, 2); // velocity measurement noise local y
    R_local(2, 2) = pow(cfg_.d_ekf_can_meas_uncertainty_vel_mps * 2, 2); // velocity measurement noise local z

    R.block<3, 3>(0, 0) = S_.rot.toRotationMatrix() * R_local * S_.rot.toRotationMatrix().transpose();
    R(3, 3) = pow(cfg_.d_ekf_can_meas_uncertainty_yaw_rate_deg * M_PI / 180.0, 2); // yaw rate measurement noise

    // Observation covariance matrix: S
    Eigen::Matrix<double, 4, 4> S = H * P_ * H.transpose() + R;

    // Kalman Gain: K
    Eigen::Matrix<double, STATE_ORDER, 4> K = P_ * H.transpose() * S.inverse();

    // Residual: Y (for velocity in local x direction and yaw rate)
    Eigen::Matrix<double, 4, 1> Y = Z - Z_state;

    // Update state with Kalman gain
    UpdateEkfState(K, Y, P_, H, S_);

    prev_can_ = unbiased_can; // Update previous CAN data timestamp

    // CAN ZUPT 수행
    ZuptCan(can_input);

    return true;
}

bool EkfAlgorithm::RunBestVelUpdate(BestVelStruct best_vel_input) {
    std::lock_guard<std::mutex> lock(mutex_state_);

    // Observation matrix: H (vx, vy, vz, yaw)
    Eigen::Matrix<double, 4, STATE_ORDER> H;
    H.setZero();

    // 속도와 yaw 관련 H 행렬 설정
    H(0, S_VX) = 1.0;
    H(1, S_VY) = 1.0;
    H(2, S_VZ) = 1.0;
    H(3, S_YAW) = 1.0;

    // Measurement vector: Z
    Eigen::Vector4d Z;
    Eigen::Vector4d Z_state;

    // Convert velocity to local coordinates
    Eigen::Vector3d vel_local = S_.rot.toRotationMatrix().transpose() * S_.vel;
    // 후진일 경우 gps velocity 방향 반대로 설정
    if (vel_local.x() < 0.0) best_vel_input.vel_dir += M_PI;

    // 속도 관련 측정값 설정
    Z(0) = best_vel_input.hor_vel * cos(best_vel_input.vel_dir);
    Z(1) = best_vel_input.hor_vel * sin(best_vel_input.vel_dir);
    Z(2) = best_vel_input.ver_vel;
    Z(3) = best_vel_input.vel_dir;

    // 상태 벡터 설정
    Z_state(0) = S_.vel.x();
    Z_state(1) = S_.vel.y();
    Z_state(2) = S_.vel.z();
    Z_state(3) = RotToVec(S_.rot.toRotationMatrix()).z();

    // Measurement covariance matrix: R
    Eigen::Matrix4d R = Eigen::Matrix4d::Zero();

    // 속도 관련 공분산 설정
    R(0, 0) = pow(cfg_.d_ekf_bestvel_meas_uncertainty_vel_mps, 2);
    R(1, 1) = pow(cfg_.d_ekf_bestvel_meas_uncertainty_vel_mps, 2);
    R(2, 2) = pow(cfg_.d_ekf_bestvel_meas_uncertainty_vel_mps, 2);

    // yaw 공분산을 속도에 따라 지수적으로 설정
    const double d_base_yaw_uncertainty = 1.0 * M_PI / 180.0; // 기본 yaw 불확실성 (1도)
    const double d_ref_vel = 3.0;                             // 기준 속도 (m/s)

    // 속도가 5m/s일 때 d_base_yaw_uncertainty가 되고,
    // 0에 가까울 때는 매우 커지며,
    // 3m/s 이상에서는 더 작아지도록 설정
    const double d_yaw_uncertainty =
            std::min(d_base_yaw_uncertainty * exp(d_ref_vel / std::max(best_vel_input.hor_vel, 0.5)), DBL_MAX);

    // 속도가 너무 작을땐 영향 못주도록
    if (best_vel_input.hor_vel < 1.0) H(3, S_YAW) = 0.0;

    R(3, 3) = pow(std::min(d_yaw_uncertainty, M_PI), 2); // 최대 불확실성을 π rad로 제한 (inf 방지)

    // std::cout << "BESTVEL YAW STD: " << d_yaw_uncertainty * 180.0 / M_PI << " P: " << sqrt(P_(S_YAW, S_YAW))*180.0 /
    // M_PI << std::endl;

    // Observation covariance matrix: S
    Eigen::Matrix4d S = H * P_ * H.transpose() + R;

    // Kalman Gain: K
    Eigen::Matrix<double, STATE_ORDER, 4> K = P_ * H.transpose() * S.inverse();

    // Residual: Y
    Eigen::Vector4d Y = Z - Z_state;

    Y(3) = NormAngleRad(Y(3));

    // Update state with Kalman gain
    UpdateEkfState(K, Y, P_, H, S_);

    return true;
}

void EkfAlgorithm::ZuptImu(ImuStruct imu_input) {
    // ZUPT 학습률 설정

    double alpha = 0.01; // Acc bias 업데이트 속도
    double gamma = 0.01; // Gyro bias 업데이트 속도

    double vel_thre = 0.1;
    double gyro_thre = 0.1;
    double acc_thre = 0.1;

    Eigen::Vector3d vel_local = S_.rot.inverse() * S_.vel;
    // vx vy가 작고, 각속도, 가속도 작을때만 수행. vz 는 gps 오차때문에 자주 발생하기에 무시
    if (std::fabs(vel_local.x()) > vel_thre) return;

    // 0.0 이면 1.0, 0.1 이면 0
    double vel_coeff = (vel_thre - vel_local.head<1>().norm()) / vel_thre * 0.1;

    // 1. 속도 오차(Kinematic): Local vx 가 0 일땐, 나머지도 0
    Eigen::Vector3d vel_error = -S_.vel;
    S_.vel += vel_coeff * vel_error; // acc_error를 로컬 기준으로 ba 보정

    if (cfg_.b_debug_imu_print) {
        std::cout << MAGENTA;
        std::cout << "[Zupt] Updated vel : " << S_.vel.transpose() << std::endl;
    }

    if (S_.gyro.norm() > gyro_thre || S_.acc.head<2>().norm() > acc_thre) return;

    double gyro_coeff = (gyro_thre - S_.gyro.norm()) / gyro_thre * 0.1;
    double acc_coeff = (acc_thre - S_.acc.head<2>().norm()) / acc_thre * 0.1;

    // 2. 각속도 오차: 정지 상태에서 이론적으로 각속도는 0이어야 함
    Eigen::Vector3d gyro_error = imu_input.gyro - S_.bg; // Measured gyro - bg
    S_.bg += gamma * gyro_error;                         // bg를 gyro error 방향으로 보정

    // 3. 가속도 오차: grav를 로컬 좌표계로 변환하여 ba와 비교
    Eigen::Vector3d grav_local = S_.rot.inverse() * S_.grav; // grav를 로컬 좌표계로 변환
    Eigen::Vector3d acc_error_loc = imu_input.acc - (grav_local + S_.ba);
    Eigen::Vector3d acc_error_global = S_.rot * (imu_input.acc - S_.ba) - S_.grav;

    S_.ba += alpha * acc_error_loc;              // acc_error를 로컬 기준으로 ba 보정
    S_.grav.z() += alpha * acc_error_global.z(); // acc_error를 전역으로 변환해 grav 보정

    // 업데이트 후 디버깅 출력 (선택 사항)
    if (cfg_.b_debug_imu_print) {
        std::cout << MAGENTA;
        std::cout << "[Zupt] Updated bg: " << S_.bg.transpose() << std::endl;
        std::cout << "[Zupt] Updated ba: " << S_.ba.transpose() << std::endl;
        std::cout << "[Zupt] Updated grav: " << S_.grav.transpose() << RESET << std::endl;
    }

    return;
}

void EkfAlgorithm::ZuptCan(CanStruct can_input_) {
    double d_vel_threshold = 0.05;
    if (can_input_.vel.norm() > d_vel_threshold) {
        return;
    }

    // ZUPT 학습률 설정
    double d_alpha = 0.05;

    d_can_yaw_rate_bias_rad_ = d_alpha * can_input_.gyro.z() + (1.0 - d_alpha) * d_can_yaw_rate_bias_rad_;

    // state 속도 지수가중 업데이트
    S_.vel = (1.0 - d_alpha) * S_.vel;

    if (cfg_.b_debug_imu_print) {
        std::cout << CYAN;
        std::cout << "[ZuptCan] Updated CAN BIAS: " << d_can_yaw_rate_bias_rad_ << RESET << std::endl;
    }

    return;
}

/*
    Complementary Filter:
    목적: 중력 방향을 추정하여 회전 오차를 보정
    1. 센서 입력에서 바이어스 추정값 제거
    2. 원심력 보상
    3. 중력 방향 추정
    4. 회전 오차 보정
*/
void EkfAlgorithm::ComplementaryKalmanFilter(ImuStruct imu_input) {
    // 가속도 크기 검사를 통한 동적 상태 감지
    const Eigen::Vector3d vec_acc_meas = imu_input.acc - S_.ba; // 바이어스 보정된 가속도

    // 원심력 보상
    // 1. 차량 로컬 프레임에서의 속도 계산
    Eigen::Vector3d vel_local = S_.rot.inverse() * S_.vel;

    // 2. 원심력 계산 (a_c = v * ω)
    // 차량 기준 좌표계에서 원심력은 y축 방향으로 작용
    double centripetal_acc = vel_local.x() * S_.gyro.z(); // v * yaw_rate

    // 3. 원심력 보상 벡터 생성 (차량 로컬 프레임)
    Eigen::Vector3d vec_acc_centrip(0, centripetal_acc, 0);

    // 1. 로컬 가속도 추정 (속도 변화율 기반)
    static double prev_vel_local_x = vel_local.x();
    static double prev_time = imu_input.timestamp;

    double dt = imu_input.timestamp - prev_time;
    if (dt < 1e-6) return; // 시간차가 너무 작으면 리턴

    // 속도 변화율로 가속도 계산 TODO: EKF 초반에 속도가 튀면서 해당값 튐 --> Pitch 오차 발생
    double est_acc_x = (vel_local.x() - prev_vel_local_x) / dt;
    // double est_acc_x = S_.acc.x();
    Eigen::Vector3d est_acc_local = Eigen::Vector3d(est_acc_x, 0, 0);

    // 상태 업데이트
    prev_vel_local_x = vel_local.x();
    prev_time = imu_input.timestamp;

    // 4. 원심력 보상한 순수 중력 성분
    Eigen::Vector3d compensated_acc = vec_acc_meas - vec_acc_centrip;

    // rotation이 잡히면, 종가감속 보상
    if (IsRotationStabilized()) {
        compensated_acc -= est_acc_local;
    }

    // 보상된 가속도의 크기 검사
    const double d_acc_sensor_magnitude = vec_acc_meas.norm();
    const double d_gravity_magnitude = S_.grav.norm();
    const double d_acc_diff = d_acc_sensor_magnitude - d_gravity_magnitude;

    // 1. 측정값 계산 (z) - 원심력이 보상된 가속도 사용
    Eigen::Vector3d gravity_direction = compensated_acc.normalized();
    Eigen::Vector2d z;                                             // roll, pitch 측정값
    z << std::atan2(gravity_direction.y(), gravity_direction.z()), // roll
            -std::asin(gravity_direction.x());                     // pitch

    // 2. 현재 상태의 roll, pitch 추출 (h(x))
    Eigen::Vector3d current_rpy = RotToVec(S_.rot.toRotationMatrix());
    Eigen::Vector2d h_x;     // 현재 상태의 roll, pitch
    h_x << current_rpy.x(),  // roll
            current_rpy.y(); // pitch

    // 3. Innovation (residual Y) 계산
    Eigen::Vector2d innovation = z - h_x;

    // Roll, pitch는 [-π, π] 범위로 정규화
    innovation(0) = NormAngleRad(innovation(0));
    innovation(1) = NormAngleRad(innovation(1));

    // 4. Measurement Jacobian (H) 계산
    Eigen::Matrix<double, 2, STATE_ORDER> H = Eigen::Matrix<double, 2, STATE_ORDER>::Zero();
    H(0, S_ROLL) = 1.0;  // ∂roll_meas/∂roll
    H(1, S_PITCH) = 1.0; // ∂pitch_meas/∂pitch

    // 5. Measurement noise covariance (R) 설정
    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();

    // double d_base_uncertainty = std::pow(20.0 * cfg_.d_imu_std_acc_mps, 2);
    double d_base_uncertainty = 1.0 * M_PI / 180.0;

    // 상태 초기화 전까진 update 강도 약하게 ( 발산 우려 )
    if (IsStateInitialized() == false) d_base_uncertainty = 10.0 * M_PI / 180.0;

    // 동적 상황에 따른 불확실성 증가
    const double centripetal_uncertainty = std::abs(centripetal_acc) / 9.81 * 10.0; // g 단위로 정규화
    const double longitudinal_uncertainty = std::abs(est_acc_x) / 9.81 * 10.0;      // g 단위로 정규화
    const double acc_diff_uncertainty = std::abs(d_acc_diff) / 9.81 * 10.0;
    const double d_lat_noise_scale = 1.0 + acc_diff_uncertainty + centripetal_uncertainty;
    const double d_longi_noise_scale = 1.0 + acc_diff_uncertainty + longitudinal_uncertainty;

    // std::cout<<"RollScale: "<<d_lat_noise_scale <<" PitchScale: "<<d_longi_noise_scale<<std::endl;

    // 최종 measurement noise covariance
    R(0, 0) = std::max(std::pow(d_base_uncertainty * d_lat_noise_scale, 2), std::pow(1.0 * M_PI / 180.0, 2));   // roll
    R(1, 1) = std::max(std::pow(d_base_uncertainty * d_longi_noise_scale, 2), std::pow(1.0 * M_PI / 180.0, 2)); // pitch

    // 6. Kalman gain 계산
    Eigen::Matrix2d S = H * P_ * H.transpose() + R;                             // Innovation covariance
    Eigen::Matrix<double, STATE_ORDER, 2> K = P_ * H.transpose() * S.inverse(); // Kalman gain

    // 7. EKF 상태 업데이트
    UpdateEkfState<2, 2>(K, innovation, P_, H, S_);

    if (cfg_.b_debug_imu_print) {
        std::cout << MAGENTA << "Roll: " << innovation(0) * 180.0 / M_PI << " deg, "
                  << "Pitch: " << innovation(1) * 180.0 / M_PI << " deg, "
                  << "Est Centrip acc: " << centripetal_acc << " m/s^2, "
                  << "Est X acc: " << est_acc_x << " m/s^2, "
                  << "Scale: " << d_lat_noise_scale << RESET << std::endl;
    }
}

void EkfAlgorithm::CalibrateVehicleToImu(ImuStruct imu_input) {
    // 차량이 충분히 움직이고 있는지 확인
    const double d_min_velocity = 3.0; // m/s
    Eigen::Vector3d vec_velocity = S_.vel;

    // 속도가 너무 낮으면 보정하지 않음
    if (vec_velocity.norm() < d_min_velocity) {
        return;
    }

    // Rotation 불확실성이 너무 높으면 보정하지 않음
    if (IsRotationStabilized() == false) {
        return;
    }

    if (b_vehicle_imu_calib_started_ == false) {
        b_vehicle_imu_calib_started_ = true;
        std::cout << MAGENTA << REVERSE << "[CalibrateVehicleToImu] Vehicle - IMU Calibration Started!" << RESET
                  << std::endl;
    }

    // IMU 좌표계에서의 속도 방향
    // 만약 imu가 yaw 양수로 돌아 있으면, vel y음수 발생
    Eigen::Vector3d vec_imu_vel_local = (S_.rot * S_.imu_rot.inverse()).inverse() * vec_velocity;
    Eigen::Vector3d vec_imu_vel_dir = vec_imu_vel_local.normalized();

    // 두 벡터 사이의 각도 차이를 RPY로 계산
    // 노말라이즈된 속도 벡터를 RPY로 변환
    double d_yaw = std::atan2(vec_imu_vel_dir.y(), vec_imu_vel_dir.x());
    double d_pitch = -std::asin(vec_imu_vel_dir.z()); // 수정된 부분
    double d_roll = 0.0;                              // roll은 속도 방향으로는 추정이 어려움

    // Measurement - state
    Eigen::Vector3d innovation(-d_roll, -d_pitch, -d_yaw);

    Eigen::Matrix<double, 3, STATE_ORDER> H = Eigen::Matrix<double, 3, STATE_ORDER>::Zero();

    H(0, S_IMU_ROLL) = 1.0;
    H(1, S_IMU_PITCH) = 1.0;
    H(2, S_IMU_YAW) = 1.0;

    // 측정 불확실성을 동적으로 계산
    double d_base_uncertainty = 30.0 * M_PI / 180.0; // 기본 불확실성 10도

    // 속도에 따른 스케일 계산 (속도가 높을수록 불확실성 감소)
    double d_vel_scale = std::exp(5.0 / vec_velocity.norm());

    // 각속도에 따른 스케일 계산 (각속도가 높을수록 불확실성 증가)
    Eigen::Vector3d vec_angular_rates(S_.gyro.x(), S_.gyro.y(), S_.gyro.z());
    double d_angular_scale = 1.0 + vec_angular_rates.norm() / (10.0 * M_PI / 180.0); // 10deg/s 기준

    // 최종 불확실성 계산 (최소값 제한)
    double d_final_uncertainty = std::max(d_base_uncertainty * d_vel_scale * d_angular_scale,
                                          1.0 * M_PI / 180.0 // 최소 1.0도의 불확실성 유지
    );

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * d_final_uncertainty * d_final_uncertainty;

    R(0, 0) = std::pow(1.0 * M_PI / 180.0, 2);
    R(1, 1) = std::pow(1.0 * M_PI / 180.0, 2);
    R(2, 2) = std::pow(1.0 * M_PI / 180.0, 2);

    Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double, STATE_ORDER, 3> K = P_ * H.transpose() * S.inverse();
    UpdateEkfState<3, 3>(K, innovation, P_, H, S_);

    Eigen::Vector3d imu_euler_angles = RotToVec(S_.imu_rot.toRotationMatrix());

    if (cfg_.b_debug_imu_print) {
        std::cout << MAGENTA << "innovation: " << innovation.transpose() * 180.0 / M_PI
                  << " STD: " << d_final_uncertainty * 180.0 / M_PI << std::endl;
        std::cout << MAGENTA << "imu_rot: " << imu_euler_angles.transpose() * 180.0 / M_PI << std::endl;
    }
}

EgoState EkfAlgorithm::GetCurrentState() {
    std::lock_guard<std::mutex> lock(mutex_state_);

    EgoState o_ego_state;

    // 마지막으로 사용된 prediction step의 timestamp 를 출력 stamp로 사용
    o_ego_state.timestamp = prev_timestamp_;

    double delta_time = o_ego_state.timestamp - prev_ego_state_.timestamp;
    if (delta_time < 1e-6) {
        return prev_ego_state_;
    }

    // Position and rotation (in radians)
    o_ego_state.x_m = S_.pos.x();
    o_ego_state.y_m = S_.pos.y();
    o_ego_state.z_m = S_.pos.z();

    Eigen::Vector3d euler_angles = RotToVec(S_.rot.toRotationMatrix());
    o_ego_state.roll_rad = euler_angles.x();
    o_ego_state.pitch_rad = euler_angles.y();
    o_ego_state.yaw_rad = euler_angles.z();

    // Angular rates
    o_ego_state.roll_vel = S_.gyro.x();
    o_ego_state.pitch_vel = S_.gyro.y();
    o_ego_state.yaw_vel = S_.gyro.z();

    // Convert velocity to local coordinates
    ConvertGlobalToLocalVelocity(S_.vel.x(), S_.vel.y(), S_.vel.z(), o_ego_state.roll_rad, o_ego_state.pitch_rad,
                                 o_ego_state.yaw_rad, o_ego_state.vx, o_ego_state.vy, o_ego_state.vz);

    // Convert acceleration to local coordinates
    ConvertGlobalToLocalVelocity(S_.acc.x(), S_.acc.y(), S_.acc.z(), o_ego_state.roll_rad, o_ego_state.pitch_rad,
                                 o_ego_state.yaw_rad, o_ego_state.ax, o_ego_state.ay, o_ego_state.az);

    // Convert position covariance (P) to local coordinates
    ConvertGlobalToLocalVelocity(P_(S_X, S_X), P_(S_Y, S_Y), P_(S_Z, S_Z), o_ego_state.roll_rad, o_ego_state.pitch_rad,
                                 o_ego_state.yaw_rad, o_ego_state.x_cov_m, o_ego_state.y_cov_m, o_ego_state.z_cov_m);

    o_ego_state.x_cov_m = fabs(o_ego_state.x_cov_m);
    o_ego_state.y_cov_m = fabs(o_ego_state.y_cov_m);
    o_ego_state.z_cov_m = fabs(o_ego_state.z_cov_m);

    // Covariances in global coordinates
    o_ego_state.latitude_std = sqrt(P_(S_X, S_X));
    o_ego_state.longitude_std = sqrt(P_(S_Y, S_Y));
    o_ego_state.height_std = sqrt(P_(S_Z, S_Z));
    o_ego_state.roll_cov_rad = P_(S_ROLL, S_ROLL);
    o_ego_state.pitch_cov_rad = P_(S_PITCH, S_PITCH);
    o_ego_state.yaw_cov_rad = P_(S_YAW, S_YAW);

    prev_ego_state_ = o_ego_state;

    return o_ego_state;
}

Eigen::Vector3d EkfAlgorithm::GetImuCalibration() {
    std::lock_guard<std::mutex> lock(mutex_state_);
    return RotToVec(S_.imu_rot.toRotationMatrix());
}
