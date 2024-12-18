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

    // Calculate prediction dt
    double d_dt = cur_timestamp - prev_timestamp_;

    // Print dt
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

    // Compensate IMU Bias and Gravity
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
    // F.block<3, 3>(S_X, S_AX) = 0.5 * G_R_I * d_dt * d_dt;     // ∂pos / ∂acc (including global transform) // FIXME: meaningless partial differentiation
    F.block<3, 3>(S_X, S_B_AX) = -0.5 * G_R_I * d_dt * d_dt; // ∂pos / ∂ba (including global transform)

    // Rotation partial differentiation
    // F.block<3, 3>(S_ROLL, S_ROLL_RATE) =   0.5 * d_dt * Eigen::Matrix3d::Identity(); // d rot / d gyro  // FIXME: meaningless partial differentiation
    F.block<3, 3>(S_ROLL, S_B_ROLL_RATE) = -PartialDerivativeRotWrtGyro(corrected_gyro, d_dt); // d rot / d bg

    // Velocity partial differentiation
    // F.block<3, 3>(S_VX, S_AX) = G_R_I * d_dt;             // ∂vel / ∂acc (including global transform) // FIXME: meaningless partial differentiation
    F.block<3, 3>(S_VX, S_B_AX) = -G_R_I * d_dt; // ∂vel / ∂ba (including global transform)
    F.block<3, 3>(S_ROLL_RATE, S_B_ROLL_RATE) = -Eigen::Matrix3d::Identity(); // ∂gyro / ∂bg
    F.block<3, 3>(S_AX, S_B_AX) = -G_R_I; // ∂acc / ∂ba (including global transform)

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

    // Force state initialization with PCM Init Pose
    if (gnss_input.gnss_source == GnssSource::PCM_INIT) {
        // State initialization
        S_.pos = gnss_input.pos;
        S_.rot = gnss_input.rot;
        S_.vel.setZero();
        S_.gyro.setZero();
        S_.acc.setZero();
        S_.bg.setZero();
        S_.ba.setZero();
        S_.grav = Eigen::Vector3d(0.0, 0.0, cfg_.d_imu_gravity);

        P_ = Eigen::MatrixXd::Identity(STATE_ORDER, STATE_ORDER) * INIT_STATE_COV; // Initialize diagonal with INIT_STATE_COV

        std::cout << GREEN << REVERSE << "[RunGnssUpdate] GPS Status Initialized by Init Pose!" << RESET << std::endl;

        // Since the state was forcibly changed through PCM Initialized, consider the ekf state initialized
        b_state_initialized_ = true;
        b_yaw_initialized_ = true;

        // For a certain period of time, update state only through pcm update (no prediction)
        b_pcm_init_on_going_ = true;

        return true;
    }

    CheckYawInitialized();
    CheckStateInitialized();
    CheckRotationStabilized();
    CheckStateStabilized();

    // // Do not perform PCM update before EKF initialization (risk of map matching divergence)
    // if (IsStateInitialized() == false && gnss_input.gnss_source == GnssSource::PCM) {
    //     std::cout << YELLOW << "[RunGnssUpdate] GPS Status not initialized for PCM" << RESET << std::endl;
    //     return false;
    // }

    // For a certain period of time after PCM initialization, attempt EKF initialization only through PCM Update
    if (b_pcm_init_on_going_ == true && gnss_input.gnss_source == GnssSource::PCM) {
        if (i_pcm_update_count_ > 10) {
            b_pcm_init_on_going_ = false;
            std::cout << BLUE << REVERSE << "[RunGnssUpdate] PCM Status Initialized!" << RESET << std::endl;
        }
        i_pcm_update_count_++;
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
        // Due to the nature of BESTPOS, NAVSATFIX providing pose based on antenna, calibration error occurs if yaw is not captured
        if (IsYawInitialized() == false) {
            R(0, 0) += 3.0; // TODO: Reflect actual Vehicle to Antenna error
            R(1, 1) += 3.0; // TODO: Reflect actual Vehicle to Antenna error
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

    // Convert CAN speed to global coordinate system
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

    // Perform CAN ZUPT
    ZuptCan(can_input);

    return true;
}

void EkfAlgorithm::ZuptImu(ImuStruct imu_input) {
    // Set ZUPT learning rate

    double alpha = 0.01; // Acc bias update speed
    double gamma = 0.01; // Gyro bias update speed

    double vel_thre = 0.1;
    double gyro_thre = 0.1;
    double acc_thre = 0.1;

    Eigen::Vector3d vel_local = S_.rot.inverse() * S_.vel;
    // Perform only when vx vy are small, and angular velocity, acceleration are small. Ignore vz as it often occurs due to GPS error
    if (std::fabs(vel_local.x()) > vel_thre) return;

    // If 0.0, then 1.0, if 0.1, then 0
    double vel_coeff = (vel_thre - vel_local.head<1>().norm()) / vel_thre * 0.1;

    // 1. Velocity error (Kinematic): When local vx is 0, the rest are also 0
    Eigen::Vector3d vel_error = -S_.vel;
    S_.vel += vel_coeff * vel_error; // Correct ba based on acc_error in local

    if (cfg_.b_debug_imu_print) {
        std::cout << MAGENTA;
        std::cout << "[Zupt] Updated vel : " << S_.vel.transpose() << std::endl;
    }

    if (S_.gyro.norm() > gyro_thre || S_.acc.head<2>().norm() > acc_thre) return;

    double gyro_coeff = (gyro_thre - S_.gyro.norm()) / gyro_thre * 0.1;
    double acc_coeff = (acc_thre - S_.acc.head<2>().norm()) / acc_thre * 0.1;

    // 2. Angular velocity error: Theoretically, angular velocity should be 0 when stationary
    Eigen::Vector3d gyro_error = imu_input.gyro - S_.bg; // Measured gyro - bg
    S_.bg += gamma * gyro_error;                         // Correct bg in the direction of gyro error

    // 3. Acceleration error: Convert grav to local coordinate system and compare with ba
    Eigen::Vector3d grav_local = S_.rot.inverse() * S_.grav; // Convert grav to local coordinate system
    Eigen::Vector3d acc_error_loc = imu_input.acc - (grav_local + S_.ba);
    Eigen::Vector3d acc_error_global = S_.rot * (imu_input.acc - S_.ba) - S_.grav;

    S_.ba += alpha * acc_error_loc;              // Correct ba based on acc_error in local
    S_.grav.z() += alpha * acc_error_global.z(); // Correct grav by converting acc_error to global

    // Debug output after update (optional)
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

    // Set ZUPT learning rate
    double d_alpha = 0.05;

    d_can_yaw_rate_bias_rad_ = d_alpha * can_input_.gyro.z() + (1.0 - d_alpha) * d_can_yaw_rate_bias_rad_;

    // Exponentially weighted update of state velocity
    S_.vel = (1.0 - d_alpha) * S_.vel;

    if (cfg_.b_debug_imu_print) {
        std::cout << CYAN;
        std::cout << "[ZuptCan] Updated CAN BIAS: " << d_can_yaw_rate_bias_rad_ << RESET << std::endl;
    }

    return;
}

/*
    Complementary Filter:
    Purpose: Correct rotation error by estimating gravity direction
    1. Remove bias estimate from sensor input
    2. Compensate centrifugal force
    3. Estimate gravity direction
    4. Correct rotation error
*/
void EkfAlgorithm::ComplementaryKalmanFilter(ImuStruct imu_input) {
    // Detect dynamic state through acceleration magnitude check
    const Eigen::Vector3d vec_acc_meas = imu_input.acc - S_.ba; // Bias-corrected acceleration

    // Compensate centrifugal force
    // 1. Calculate speed in vehicle local frame
    Eigen::Vector3d vel_local = S_.rot.inverse() * S_.vel;

    // 2. Calculate centrifugal force (a_c = v * ω)
    // Centrifugal force acts in the y-axis direction in the vehicle coordinate system
    double centripetal_acc = vel_local.x() * S_.gyro.z(); // v * yaw_rate

    // 3. Create centrifugal compensation vector (vehicle local frame)
    Eigen::Vector3d vec_acc_centrip(0, centripetal_acc, 0);

    // 1. Estimate local acceleration (based on speed change rate)
    static double prev_vel_local_x = vel_local.x();
    static double prev_time = imu_input.timestamp;

    double dt = imu_input.timestamp - prev_time;
    if (dt < 1e-6) return; // Return if time difference is too small

    // Calculate acceleration based on speed change rate TODO: At the beginning of EKF, speed jumps and this value jumps --> Pitch error occurs
    double est_acc_x = (vel_local.x() - prev_vel_local_x) / dt;
    // double est_acc_x = S_.acc.x();
    Eigen::Vector3d est_acc_local = Eigen::Vector3d(est_acc_x, 0, 0);

    // Update state
    prev_vel_local_x = vel_local.x();
    prev_time = imu_input.timestamp;

    // 4. Pure gravity component compensated for centrifugal force
    Eigen::Vector3d compensated_acc = vec_acc_meas - vec_acc_centrip;

    // If rotation is captured, compensate for longitudinal acceleration
    if (IsRotationStabilized()) {
        compensated_acc -= est_acc_local;
    }

    // Check the magnitude of the compensated acceleration
    const double d_acc_sensor_magnitude = vec_acc_meas.norm();
    const double d_gravity_magnitude = S_.grav.norm();
    const double d_acc_diff = d_acc_sensor_magnitude - d_gravity_magnitude;

    // 1. Calculate measurement value (z) - use compensated acceleration for centrifugal force
    Eigen::Vector3d gravity_direction = compensated_acc.normalized();
    Eigen::Vector2d z;                                             // roll, pitch measurement value
    z << std::atan2(gravity_direction.y(), gravity_direction.z()), // roll
            -std::asin(gravity_direction.x());                     // pitch

    // 2. Extract roll, pitch of current state (h(x))
    Eigen::Vector3d current_rpy = RotToVec(S_.rot.toRotationMatrix());
    Eigen::Vector2d h_x;     // roll, pitch of current state
    h_x << current_rpy.x(),  // roll
            current_rpy.y(); // pitch

    // 3. Calculate Innovation (residual Y)
    Eigen::Vector2d innovation = z - h_x;

    // Normalize roll, pitch to the range [-π, π]
    innovation(0) = NormAngleRad(innovation(0));
    innovation(1) = NormAngleRad(innovation(1));

    // 4. Calculate Measurement Jacobian (H)
    Eigen::Matrix<double, 2, STATE_ORDER> H = Eigen::Matrix<double, 2, STATE_ORDER>::Zero();
    H(0, S_ROLL) = 1.0;  // ∂roll_meas/∂roll
    H(1, S_PITCH) = 1.0; // ∂pitch_meas/∂pitch

    // 5. Set Measurement noise covariance (R)
    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();

    // double d_base_uncertainty = std::pow(20.0 * cfg_.d_imu_std_acc_mps, 2);
    double d_base_uncertainty = 1.0 * M_PI / 180.0;

    // Until the state is initialized, keep the update strength weak (risk of divergence)
    if (IsStateInitialized() == false) d_base_uncertainty = 10.0 * M_PI / 180.0;

    // Increase uncertainty according to dynamic situation
    const double centripetal_uncertainty = std::abs(centripetal_acc) / 9.81 * 10.0; // Normalize in g units
    const double longitudinal_uncertainty = std::abs(est_acc_x) / 9.81 * 10.0;      // Normalize in g units
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
