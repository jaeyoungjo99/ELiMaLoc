/**
 * @file ekf_localization.cpp
 * @author Jaeyoung Jo <wodud3743@gmail.com>
 * @brief Extended Kalman Filter for various state estimation
 * @version 0.1
 * @date 2024-10-28
 *
 * @copyright Copyright (c) 2024
 */

#include "ekf_localization.hpp"

EkfLocalization::EkfLocalization(std::string node_name, double period) 
    : task_name_(node_name), task_period_(period), task_rate_(1.0 / period) {
    ROS_WARN_STREAM("[" << node_name << "] Initialize node (Period: " << 1 / period << " Hz)");
    update_time_ = ros::Time::now();
    execution_time_ = ros::Duration(0.0);

    // memset(&i_vehicle_can_, 0, sizeof(interface::VehicleCAN));
    ptr_ekf_algorithm_ = std::shared_ptr<EkfAlgorithm>();
    Init();
}
EkfLocalization::~EkfLocalization() {
    Eigen::Vector3d imu_calib = ptr_ekf_algorithm_->GetImuCalibration();
    std::cout << REVERSE << GREEN << "IMU Calibration deg: " << imu_calib.transpose() * 180.0 / M_PI << RESET
              << std::endl;
}

void EkfLocalization::Init() {
    ROS_INFO("Init function");
    NodeHandle nh;

    // Ini init
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/localization.ini");
    std::string calib_ini_dir("/config/calibration.ini");
    util_ini_parser_.Init((dir + ini_dir).c_str());
    util_calib_ini_parser_.Init((dir + calib_ini_dir).c_str());

    nh.getParam("/ekf_localization/ref_latitude", cfg_.ref_latitude);
    nh.getParam("/ekf_localization/ref_longitude", cfg_.ref_longitude);
    nh.getParam("/ekf_localization/ref_height", cfg_.ref_altitude);
    std::cout.precision(10);
    std::cout << YELLOW << "Reference Latitude " << cfg_.ref_latitude << ", Longitude: " << cfg_.ref_longitude << RESET
              << std::endl;

    ProcessINI();

    // Subscriber init
    if (cfg_.i_gps_type == GpsType::ODOMETRY) {
        // sub_novatel_inspvax_ = nh.subscribe("novatel/oem7/inspvax", 1, &EkfLocalization::CallbackINSPVAX, this);
    }
    else if (cfg_.i_gps_type == GpsType::NAVSATFIX) {
        sub_navsatfix_ = nh.subscribe(cfg_.s_navsatfix_topic_name, 1, &EkfLocalization::CallbackNavsatFix, this);
    }

    sub_can_ = nh.subscribe(cfg_.s_can_topic_name, 1, &EkfLocalization::CallbackCAN, this);
    sub_imu_ = nh.subscribe(cfg_.s_imu_topic_name, 1, &EkfLocalization::CallbackImu, this);
    sub_pcm_odom_ = nh.subscribe("/app/loc/pcm_odom", 1, &EkfLocalization::CallbackPcmOdom, this);
    sub_pcm_init_odom_ = nh.subscribe("/app/loc/pcm_init_odom", 1, &EkfLocalization::CallbackPcmInitOdom, this);

    // Publisher init
    // pub_vehicle_state_ = nh.advertise<autohyu_msgs::VehicleState>("app/loc/vehicle_state", 10);
    // pub_reference_ = nh.advertise<autohyu_msgs::Reference>("app/loc/reference_point", 10);

    pub_ekf_pose_odom_ = nh.advertise<nav_msgs::Odometry>("/app/loc/ekf_pose_odom", 10);
    pub_ekf_ego_marker_ = nh.advertise<visualization_msgs::Marker>("/app/loc/ekf_ego_marker", 1);
    pub_gps_ego_marker_ = nh.advertise<visualization_msgs::Marker>("/app/loc/gps_ego_marker", 1);
    pub_rviz_ekf_text_ = nh.advertise<jsk_rviz_plugins::OverlayText>("/app/loc/ekf_text", 1);

    pub_x_plot_ = nh.advertise<std_msgs::Float32>("/app/loc/ekf/x_plot", 1);
    pub_y_plot_ = nh.advertise<std_msgs::Float32>("/app/loc/ekf/y_plot", 1);
    pub_z_plot_ = nh.advertise<std_msgs::Float32>("/app/loc/ekf/z_plot", 1);
    pub_vx_plot_ = nh.advertise<std_msgs::Float32>("/app/loc/ekf/vx_plot", 1);
    pub_vy_plot_ = nh.advertise<std_msgs::Float32>("/app/loc/ekf/vy_plot", 1);
    pub_vz_plot_ = nh.advertise<std_msgs::Float32>("/app/loc/ekf/vz_plot", 1);
    pub_ax_plot_ = nh.advertise<std_msgs::Float32>("/app/loc/ekf/ax_plot", 1);
    pub_ay_plot_ = nh.advertise<std_msgs::Float32>("/app/loc/ekf/ay_plot", 1);
    pub_az_plot_ = nh.advertise<std_msgs::Float32>("/app/loc/ekf/az_plot", 1);
    pub_roll_deg_plot_ = nh.advertise<std_msgs::Float32>("/app/loc/ekf/roll_deg_plot", 1);
    pub_pitch_deg_plot_ = nh.advertise<std_msgs::Float32>("/app/loc/ekf/pitch_deg_plot", 1);
    pub_yaw_deg_plot_ = nh.advertise<std_msgs::Float32>("/app/loc/ekf/yaw_deg_plot", 1);

    pub_satellite_nav_fix_ = nh.advertise<sensor_msgs::NavSatFix>("/app/loc/satellite_nav_fix", 1);

    pub_gps_pose_odom_ = nh.advertise<nav_msgs::Odometry>("/app/loc/gps_pose_odom", 10);

    ptr_ekf_algorithm_.reset(new EkfAlgorithm(cfg_));
    ptr_ekf_algorithm_->Init();

    ROS_INFO("Init function Done");
}

void EkfLocalization::CallbackNavsatFix(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    EkfGnssMeasurement gnss_meas;

    // Timestamp and source
    gnss_meas.timestamp = msg->header.stamp.toSec();
    gnss_meas.gnss_source = GnssSource::NAVSATFIX;

    // GPS coordinates (latitude, longitude, altitude)
    gnss_meas.pos = ProjectGpsPoint(msg->latitude, msg->longitude, msg->altitude);

    // GNSS measurement covariance for position
    gnss_meas.pos_cov.setZero();
    gnss_meas.pos_cov(0, 0) = std::pow(msg->position_covariance[0], 2); // latitude std
    gnss_meas.pos_cov(1, 1) = std::pow(msg->position_covariance[4], 2); // longitude std
    gnss_meas.pos_cov(2, 2) = std::pow(msg->position_covariance[8], 2); // altitude std

    // 회전 데이터 (NavSatFix에는 기본적으로 포함되지 않음), 단위 quaternion으로 초기화
    gnss_meas.rot = Eigen::Quaterniond::Identity();

    // 회전 불확실성 설정 (기본적으로 0으로 설정)
    gnss_meas.rot_cov.setZero();

    // GPS 시각화
    UpdateGpsEgoMarker(gnss_meas);

    if (cfg_.b_use_gps == false) return;

    // GNSS 불확실성 검증
    if (gnss_meas.pos_cov(0, 0) > cfg_.gnss_uncertainy_max_m || gnss_meas.pos_cov(1, 1) > cfg_.gnss_uncertainy_max_m)
        return;

    // EKF 알고리즘을 이용한 GNSS 업데이트
    if (ptr_ekf_algorithm_->RunGnssUpdate(gnss_meas) == true) {
        UpdateGpsOdom(gnss_meas);
    }
}

void EkfLocalization::CallbackCAN(const geometry_msgs::TwistStampedConstPtr& msg) {
    if (cfg_.b_use_can == false) return;
    i_can_ = *msg;
    CanStruct can_struct{0.0};

    can_struct.timestamp = i_can_.header.stamp.toSec();
    can_struct.vel.x() = i_can_.twist.linear.x;
    can_struct.gyro.z() = i_can_.twist.angular.z;

    ptr_ekf_algorithm_->RunCanUpdate(can_struct);
}

void EkfLocalization::CallbackImu(const sensor_msgs::Imu::ConstPtr& msg) {
    if (cfg_.b_use_imu == false) return;
    ProcessINI();
    ImuStruct imu_struct = ImuStructConverter(*msg, cfg_.mat_ego_to_imu_rot, cfg_.vec_ego_to_imu_trans);
    ptr_ekf_algorithm_->RunPredictionImu(imu_struct.timestamp, imu_struct);
    PublishInThread(); // Imu Prediction 주기마다 출력
}

void EkfLocalization::CallbackPcmOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    if (cfg_.b_use_pcm_matching == false) return;
    nav_msgs::Odometry pcm_odom = *msg;

    EkfGnssMeasurement gnss_meas, time_compensated_gnss;

    gnss_meas.timestamp = pcm_odom.header.stamp.toSec();
    gnss_meas.gnss_source = GnssSource::PCM;

    // Position in global coordinates
    gnss_meas.pos = Eigen::Vector3d(pcm_odom.pose.pose.position.x, pcm_odom.pose.pose.position.y,
                                    pcm_odom.pose.pose.position.z);

    gnss_meas.rot = Eigen::Quaterniond(pcm_odom.pose.pose.orientation.w, pcm_odom.pose.pose.orientation.x,
                                       pcm_odom.pose.pose.orientation.y, pcm_odom.pose.pose.orientation.z);

    gnss_meas.pos_cov.setZero();
    gnss_meas.rot_cov.setZero();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            gnss_meas.pos_cov(row, col) = pcm_odom.pose.covariance[row * 6 + col];
        }
    }
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            gnss_meas.rot_cov(row, col) = pcm_odom.pose.covariance[(row + 3) * 6 + (col + 3)];
        }
    }

    if (GnssTimeCompensation(gnss_meas, time_compensated_gnss)) {
        ptr_ekf_algorithm_->RunGnssUpdate(time_compensated_gnss);
    }
}

void EkfLocalization::CallbackPcmInitOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    if (cfg_.b_use_pcm_matching == false) return;
    nav_msgs::Odometry pcm_init_odom = *msg;

    EkfGnssMeasurement gnss_meas, time_compensated_gnss;

    gnss_meas.timestamp = pcm_init_odom.header.stamp.toSec();
    gnss_meas.gnss_source = GnssSource::PCM_INIT;

    // Position in global coordinates
    gnss_meas.pos = Eigen::Vector3d(pcm_init_odom.pose.pose.position.x, pcm_init_odom.pose.pose.position.y,
                                    pcm_init_odom.pose.pose.position.z);

    gnss_meas.rot = Eigen::Quaterniond(pcm_init_odom.pose.pose.orientation.w, pcm_init_odom.pose.pose.orientation.x,
                                       pcm_init_odom.pose.pose.orientation.y, pcm_init_odom.pose.pose.orientation.z);

    gnss_meas.pos_cov.setZero();
    gnss_meas.rot_cov.setZero();

    gnss_meas.pos_cov = Eigen::Matrix3d::Identity() * 1e-9;
    gnss_meas.rot_cov = Eigen::Matrix3d::Identity() * 1e-9;

    ptr_ekf_algorithm_->RunGnssUpdate(gnss_meas);
}

void EkfLocalization::Run() {
    // imu 를 prediction 으로 쓰지 ��으면, ros time을 기준시로 사용하여 Prediction 수행
    // imu 사용시, Imu call back 때 Prediction 수행
    if (cfg_.b_use_imu == true) return;

    double cur_timestamp = ros::Time::now().toSec();

    ProcessINI();
    ptr_ekf_algorithm_->RunPrediction(cur_timestamp);
    PublishInThread();
}

void EkfLocalization::ProcessINI() {
    if (util_calib_ini_parser_.IsFileUpdated()) {
        std::cout << "[ProcessINI] Calib Ini Updated!" << std::endl;

        util_calib_ini_parser_.ParseConfig("Vehicle Origin", "vehicle_origin", cfg_.i_vehicle_origin);

        std::string str_origin = "";
        if (cfg_.i_vehicle_origin == 0) {
            str_origin = "Rear";
        }
        else {
            str_origin = "CG";
        }
        util_calib_ini_parser_.ParseConfig(str_origin + " To Imu", "transform_xyz_m", cfg_.vec_d_ego_to_imu_trans);
        util_calib_ini_parser_.ParseConfig(str_origin + " To Imu", "rotation_rpy_deg", cfg_.vec_d_ego_to_imu_rot);
        util_calib_ini_parser_.ParseConfig(str_origin + " To Gps", "transform_xyz_m", cfg_.vec_d_ego_to_gps_trans);
        util_calib_ini_parser_.ParseConfig(str_origin + " To Gps", "rotation_rpy_deg", cfg_.vec_d_ego_to_gps_rot);

        // Transform ego_to_imu calibration parameters
        if (cfg_.vec_d_ego_to_imu_trans.size() == 3 && cfg_.vec_d_ego_to_imu_rot.size() == 3 &&
            cfg_.vec_d_ego_to_gps_trans.size() == 3 && cfg_.vec_d_ego_to_gps_rot.size() == 3) {
            cfg_.vec_ego_to_imu_trans = Eigen::Vector3d(cfg_.vec_d_ego_to_imu_trans[0], cfg_.vec_d_ego_to_imu_trans[1],
                                                        cfg_.vec_d_ego_to_imu_trans[2]);
            Eigen::Vector3d euler_ego_to_imu = Eigen::Vector3d(cfg_.vec_d_ego_to_imu_rot[0] * M_PI / 180.0,
                                                               cfg_.vec_d_ego_to_imu_rot[1] * M_PI / 180.0,
                                                               cfg_.vec_d_ego_to_imu_rot[2] * M_PI / 180.0);
            cfg_.mat_ego_to_imu_rot = VecToRot(euler_ego_to_imu);

            cfg_.vec_ego_to_gps_trans = Eigen::Vector3d(cfg_.vec_d_ego_to_gps_trans[0], cfg_.vec_d_ego_to_gps_trans[1],
                                                        cfg_.vec_d_ego_to_gps_trans[2]);
            Eigen::Vector3d euler_ego_to_gps = Eigen::Vector3d(cfg_.vec_d_ego_to_gps_rot[0] * M_PI / 180.0,
                                                               cfg_.vec_d_ego_to_gps_rot[1] * M_PI / 180.0,
                                                               cfg_.vec_d_ego_to_gps_rot[2] * M_PI / 180.0);
            cfg_.mat_ego_to_gps_rot = VecToRot(euler_ego_to_gps);
        }
        else {
            ROS_ERROR("[EKF Localization] Invalid Calibration!");
            ros::shutdown();
        }
    }
    if (util_ini_parser_.IsFileUpdated()) {
        std::cout << "[ProcessINI] Ini Updated!" << std::endl;

        // Node Configuration
        util_ini_parser_.ParseConfig("common_variable", "projection_mode", cfg_.projection_mode);
        // util_ini_parser_.ParseConfig("common_variable", "ref_latitude", cfg_.ref_latitude);
        // util_ini_parser_.ParseConfig("common_variable", "ref_longitude", cfg_.ref_longitude);
        // util_ini_parser_.ParseConfig("common_variable", "ref_altitude", cfg_.ref_altitude);

        // Topic Configuration
        util_ini_parser_.ParseConfig("common_variable", "can_topic_name", cfg_.s_can_topic_name);
        util_ini_parser_.ParseConfig("common_variable", "imu_topic_name", cfg_.s_imu_topic_name);
        util_ini_parser_.ParseConfig("common_variable", "navsatfix_topic_name", cfg_.s_navsatfix_topic_name);

        // Debug Configuration
        util_ini_parser_.ParseConfig("ekf_localization", "debug_print", cfg_.b_debug_print);
        util_ini_parser_.ParseConfig("ekf_localization", "debug_imu_print", cfg_.b_debug_imu_print);

        int i_temp_gps_type;
        util_ini_parser_.ParseConfig("ekf_localization", "gps_type", i_temp_gps_type);
        cfg_.i_gps_type = GpsType(i_temp_gps_type);
        util_ini_parser_.ParseConfig("ekf_localization", "use_gps", cfg_.b_use_gps);
        util_ini_parser_.ParseConfig("ekf_localization", "use_can", cfg_.b_use_can);
        util_ini_parser_.ParseConfig("ekf_localization", "use_imu", cfg_.b_use_imu);
        util_ini_parser_.ParseConfig("ekf_localization", "use_pcm_matching", cfg_.b_use_pcm_matching);

        // GNSS Configuration
        util_ini_parser_.ParseConfig("ekf_localization", "gnss_uncertainy_max_m", cfg_.gnss_uncertainy_max_m);

        util_ini_parser_.ParseConfig("ekf_localization", "imu_gravity", cfg_.d_imu_gravity);
        util_ini_parser_.ParseConfig("ekf_localization", "imu_estimate_gravity", cfg_.b_imu_estimate_gravity);
        util_ini_parser_.ParseConfig("ekf_localization", "imu_estimate_calibration", cfg_.b_imu_estimate_calibration);
        util_ini_parser_.ParseConfig("ekf_localization", "use_zupt", cfg_.b_use_zupt);
        util_ini_parser_.ParseConfig("ekf_localization", "use_complementary_filter", cfg_.b_use_complementary_filter);
        util_ini_parser_.ParseConfig("ekf_localization", "can_vel_scale_factor", cfg_.d_can_vel_scale_factor);

        util_ini_parser_.ParseConfig("ekf_localization", "ekf_init_x_m", cfg_.d_ekf_init_x_m);
        util_ini_parser_.ParseConfig("ekf_localization", "ekf_init_y_m", cfg_.d_ekf_init_y_m);
        util_ini_parser_.ParseConfig("ekf_localization", "ekf_init_z_m", cfg_.d_ekf_init_z_m);
        util_ini_parser_.ParseConfig("ekf_localization", "ekf_init_roll_deg", cfg_.d_ekf_init_roll_deg);
        util_ini_parser_.ParseConfig("ekf_localization", "ekf_init_pitch_deg", cfg_.d_ekf_init_pitch_deg);
        util_ini_parser_.ParseConfig("ekf_localization", "ekf_init_yaw_deg", cfg_.d_ekf_init_yaw_deg);

        util_ini_parser_.ParseConfig("ekf_localization", "ekf_state_uncertainty_pos_m", cfg_.d_state_std_pos_m);
        util_ini_parser_.ParseConfig("ekf_localization", "ekf_state_uncertainty_rot_deg", cfg_.d_state_std_rot_deg);
        util_ini_parser_.ParseConfig("ekf_localization", "ekf_state_uncertainty_vel_mps", cfg_.d_state_std_vel_mps);
        util_ini_parser_.ParseConfig("ekf_localization", "ekf_state_uncertainty_gyro_dps", cfg_.d_state_std_gyro_dps);
        util_ini_parser_.ParseConfig("ekf_localization", "ekf_state_uncertainty_acc_mps", cfg_.d_state_std_acc_mps);

        util_ini_parser_.ParseConfig("ekf_localization", "ekf_imu_uncertainty_gyro_dps", cfg_.d_imu_std_gyro_dps);
        util_ini_parser_.ParseConfig("ekf_localization", "ekf_imu_uncertainty_acc_mps", cfg_.d_imu_std_acc_mps);

        util_ini_parser_.ParseConfig("ekf_localization", "ekf_imu_bias_cov_gyro", cfg_.d_ekf_imu_bias_cov_gyro);
        util_ini_parser_.ParseConfig("ekf_localization", "ekf_imu_bias_cov_acc", cfg_.d_ekf_imu_bias_cov_acc);

        util_ini_parser_.ParseConfig("ekf_localization", "ekf_gnss_min_cov_x_m", cfg_.d_ekf_gnss_min_cov_x_m);
        util_ini_parser_.ParseConfig("ekf_localization", "ekf_gnss_min_cov_y_m", cfg_.d_ekf_gnss_min_cov_y_m);
        util_ini_parser_.ParseConfig("ekf_localization", "ekf_gnss_min_cov_z_m", cfg_.d_ekf_gnss_min_cov_z_m);
        util_ini_parser_.ParseConfig("ekf_localization", "ekf_gnss_min_cov_roll_deg", cfg_.d_ekf_gnss_min_cov_roll_deg);
        util_ini_parser_.ParseConfig("ekf_localization", "ekf_gnss_min_cov_pitch_deg",
                                     cfg_.d_ekf_gnss_min_cov_pitch_deg);
        util_ini_parser_.ParseConfig("ekf_localization", "ekf_gnss_min_cov_yaw_deg", cfg_.d_ekf_gnss_min_cov_yaw_deg);

        util_ini_parser_.ParseConfig("ekf_localization", "ekf_can_meas_uncertainty_vel_mps",
                                     cfg_.d_ekf_can_meas_uncertainty_vel_mps);
        util_ini_parser_.ParseConfig("ekf_localization", "ekf_can_meas_uncertainty_yaw_rate_deg",
                                     cfg_.d_ekf_can_meas_uncertainty_yaw_rate_deg);
        util_ini_parser_.ParseConfig("ekf_localization", "ekf_bestvel_meas_uncertainty_vel_mps",
                                     cfg_.d_ekf_bestvel_meas_uncertainty_vel_mps);

        if (ptr_ekf_algorithm_) {
            ptr_ekf_algorithm_->UpdateDynamicConfig(cfg_);
        }
    }
}

// gnss와 가장 가까운 EKF State를 통해 Interpolation
bool EkfLocalization::GnssTimeCompensation(const EkfGnssMeasurement& i_gnss, EkfGnssMeasurement& o_gnss) {
    o_gnss = i_gnss;

    EgoState closest_ekf_state, current_ekf_state;
    {
        std::lock_guard<std::mutex> lock(mutex_ekf_state_deque_);

        // EKF 상태 큐가 비었으면 보정 불가능
        if (deq_ekf_state_.empty()) return false;

        current_ekf_state = deq_ekf_state_.back();

        // GNSS 시간보다 최신의 EKF 상태만 있는 경우, 보정 불가
        if (deq_ekf_state_.front().timestamp > i_gnss.timestamp) return false;

        // GNSS 시간보다 늦은 EKF 상태를 찾음
        for (const auto& ekf_state : deq_ekf_state_) {
            if (ekf_state.timestamp > i_gnss.timestamp) {
                closest_ekf_state = ekf_state;
                break;
            }
            closest_ekf_state = ekf_state; // 최신 EKF 상태 업데이트
        }
    }

    // GNSS 시간과 가장 가까운 EKF 상태와의 시간 ��이 계산
    double d_gnss_to_ekf_time_sec = current_ekf_state.timestamp - i_gnss.timestamp;

    // EKF 시간과 GNSS 시간이 같거나 EKF 시간이 더 과거인 경우, 보정 필요 없음
    if (d_gnss_to_ekf_time_sec <= 0.0) return true;

    // 위치 및 회전 보정 변수 초기화
    double dx{0.0}, dy{0.0}, dz{0.0};
    double d_roll{0.0}, d_pitch{0.0}, d_yaw{0.0};

    // closest_ekf_state와 current_ekf_state가 다를 때 보정 수행
    if (fabs(current_ekf_state.timestamp - closest_ekf_state.timestamp) > 1e-5) {
        double ratio = d_gnss_to_ekf_time_sec / (current_ekf_state.timestamp - closest_ekf_state.timestamp);

        // EKF 상태 간 상대 변위 계산
        dx = (current_ekf_state.x_m - closest_ekf_state.x_m) * ratio;
        dy = (current_ekf_state.y_m - closest_ekf_state.y_m) * ratio;
        dz = (current_ekf_state.z_m - closest_ekf_state.z_m) * ratio;

        // 회전 각도 차이 계산 (roll, pitch, yaw)
        d_roll = AngleDiffRad(closest_ekf_state.roll_rad, current_ekf_state.roll_rad) * ratio;
        d_pitch = AngleDiffRad(closest_ekf_state.pitch_rad, current_ekf_state.pitch_rad) * ratio;
        d_yaw = AngleDiffRad(closest_ekf_state.yaw_rad, current_ekf_state.yaw_rad) * ratio;
    }

    // 상대 변위와 회전 변위를 GNSS에 반영
    o_gnss.timestamp = current_ekf_state.timestamp;
    o_gnss.pos.x() = i_gnss.pos.x() + dx;
    o_gnss.pos.y() = i_gnss.pos.y() + dy;
    o_gnss.pos.z() = i_gnss.pos.z() + dz;

    // Quaternion으로 각도 차이를 반영하여 회전 보정
    Eigen::Quaterniond delta_quaternion = Eigen::AngleAxisd(d_yaw, Eigen::Vector3d::UnitZ()) *
                                          Eigen::AngleAxisd(d_pitch, Eigen::Vector3d::UnitY()) *
                                          Eigen::AngleAxisd(d_roll, Eigen::Vector3d::UnitX());
    o_gnss.rot = i_gnss.rot * delta_quaternion;
    o_gnss.rot.normalize();

    // 디버그 출력
    if (cfg_.b_debug_print) {
        std::cout.precision(3);
        std::cout << "[GnssTimeCompensation] Time comp: " << d_gnss_to_ekf_time_sec << " Pos: " << dx << " " << dy
                  << " " << dz << " Rot: " << d_roll * 180.0 / M_PI << " " << d_pitch * 180.0 / M_PI << " "
                  << d_yaw * 180.0 / M_PI << std::endl;
    }

    return true;
}

// ----- Publish Result Functions ----- //
void EkfLocalization::PublishInThread() {
    EgoState ego_ekf_state = ptr_ekf_algorithm_->GetCurrentState();

    {
        std::lock_guard<std::mutex> lock(mutex_ekf_state_deque_);
        if (deq_ekf_state_.size() < 1 || deq_ekf_state_.back().timestamp + 1e-5 < ego_ekf_state.timestamp)
            deq_ekf_state_.push_back(ego_ekf_state);

        if (deq_ekf_state_.back().timestamp > ego_ekf_state.timestamp) deq_ekf_state_.clear();

        while (deq_ekf_state_.size() > 1000) {
            deq_ekf_state_.pop_front();
        }
    }

    // GeographicLib를 사용하여 GPSPoint 계산
    GeographicLib::LocalCartesian local_cartesian(cfg_.ref_latitude, cfg_.ref_longitude, cfg_.ref_altitude);
    double lat, lon, ele;
    local_cartesian.Reverse(ego_ekf_state.x_m, ego_ekf_state.y_m, ego_ekf_state.z_m, lat, lon, ele);

    ego_ekf_state.latitude = lat;
    ego_ekf_state.longitude = lon;
    ego_ekf_state.height = ele;

    UpdateEgoMarker(ego_ekf_state);
    UpdateTF(ego_ekf_state);
    UpdateEkfOdom(ego_ekf_state);
    UpdateEkfText(ego_ekf_state);
}

void EkfLocalization::UpdateEgoMarker(EgoState ego_ekf_state) {
    visualization_msgs::Marker o_ekf_ego_marker_msgs;

    o_ekf_ego_marker_msgs.type = visualization_msgs::Marker::CUBE;
    o_ekf_ego_marker_msgs.header.frame_id = "world";
    o_ekf_ego_marker_msgs.header.stamp = ros::Time(ego_ekf_state.timestamp);

    o_ekf_ego_marker_msgs.scale.x = 4.57;
    o_ekf_ego_marker_msgs.scale.y = 1.8;
    o_ekf_ego_marker_msgs.scale.z = 1.44;

    o_ekf_ego_marker_msgs.color.a = 0.5;
    o_ekf_ego_marker_msgs.color.r = 0.6;
    o_ekf_ego_marker_msgs.color.g = 0.8;
    o_ekf_ego_marker_msgs.color.b = 1.0;

    // Calculate orientation quaternion based on yaw, pitch, roll in EgoState
    Eigen::Quaterniond quat = Eigen::AngleAxisd(ego_ekf_state.yaw_rad, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(ego_ekf_state.pitch_rad, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(ego_ekf_state.roll_rad, Eigen::Vector3d::UnitX());

    // Adjust position to be 1.0m behind the center (along the x-axis in the local frame)
    double x_offset = 0.0;
    if (cfg_.i_vehicle_origin == 0) x_offset = 1.51;
    Eigen::Vector3d offset(x_offset, 0.0, o_ekf_ego_marker_msgs.scale.z / 2.0);
    Eigen::Vector3d adjusted_position =
            Eigen::Vector3d(ego_ekf_state.x_m, ego_ekf_state.y_m, ego_ekf_state.z_m) + quat * offset;

    // Assign the adjusted position to the marker's pose
    o_ekf_ego_marker_msgs.pose.position.x = adjusted_position.x();
    o_ekf_ego_marker_msgs.pose.position.y = adjusted_position.y();
    o_ekf_ego_marker_msgs.pose.position.z = adjusted_position.z();

    // Assign quaternion to marker's orientation
    o_ekf_ego_marker_msgs.pose.orientation.x = quat.x();
    o_ekf_ego_marker_msgs.pose.orientation.y = quat.y();
    o_ekf_ego_marker_msgs.pose.orientation.z = quat.z();
    o_ekf_ego_marker_msgs.pose.orientation.w = quat.w();

    pub_ekf_ego_marker_.publish(o_ekf_ego_marker_msgs);
}

void EkfLocalization::UpdateGpsEgoMarker(EkfGnssMeasurement ego_gps_state) {
    visualization_msgs::Marker o_gps_ego_marker_msgs;

    o_gps_ego_marker_msgs.type = visualization_msgs::Marker::CUBE;
    o_gps_ego_marker_msgs.header.frame_id = "world";
    o_gps_ego_marker_msgs.header.stamp = ros::Time(ego_gps_state.timestamp);

    o_gps_ego_marker_msgs.scale.x = 4.57;
    o_gps_ego_marker_msgs.scale.y = 1.8;
    o_gps_ego_marker_msgs.scale.z = 1.44;

    o_gps_ego_marker_msgs.color.a = 0.5;
    o_gps_ego_marker_msgs.color.r = 1.0;
    o_gps_ego_marker_msgs.color.g = 0.0;
    o_gps_ego_marker_msgs.color.b = 0.0;

    // Adjust position to be 1.0m behind the center (along the x-axis in the local frame)
    double x_offset = 0.0;
    if (cfg_.i_vehicle_origin == 0) x_offset = 1.51;
    Eigen::Vector3d offset(x_offset, 0.0, o_gps_ego_marker_msgs.scale.z / 2.0);
    Eigen::Vector3d adjusted_position =
            Eigen::Vector3d(ego_gps_state.pos.x(), ego_gps_state.pos.y(), ego_gps_state.pos.z()) +
            ego_gps_state.rot * offset;

    // Position from GPS measurement
    o_gps_ego_marker_msgs.pose.position.x = adjusted_position.x();
    o_gps_ego_marker_msgs.pose.position.y = adjusted_position.y();
    o_gps_ego_marker_msgs.pose.position.z = adjusted_position.z();

    // Orientation from GPS measurement (Quaternion)
    o_gps_ego_marker_msgs.pose.orientation.x = ego_gps_state.rot.x();
    o_gps_ego_marker_msgs.pose.orientation.y = ego_gps_state.rot.y();
    o_gps_ego_marker_msgs.pose.orientation.z = ego_gps_state.rot.z();
    o_gps_ego_marker_msgs.pose.orientation.w = ego_gps_state.rot.w();

    pub_gps_ego_marker_.publish(o_gps_ego_marker_msgs);
}

void EkfLocalization::UpdateTF(EgoState ego_ekf_state) {
    tf::Transform transform;

    Eigen::Quaterniond quat = Eigen::AngleAxisd(ego_ekf_state.yaw_rad, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(ego_ekf_state.pitch_rad, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(ego_ekf_state.roll_rad, Eigen::Vector3d::UnitX());

    transform.setOrigin(tf::Vector3(ego_ekf_state.x_m, ego_ekf_state.y_m, ego_ekf_state.z_m)); // x, y, z 축 변환값
    transform.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w())); // 회전값의 스칼라 부분

    // 현재 시간으로 TimeStamped 객체 생성
    tf::StampedTransform stampedTransform(transform, ros::Time(ego_ekf_state.timestamp), "world", "ego_frame");

    // TF 송신
    tf_broadcaster_.sendTransform(stampedTransform);
}

void EkfLocalization::UpdateEkfOdom(EgoState ego_ekf_state) {
    nav_msgs::Odometry ekf_pose_odom;

    ekf_pose_odom.header.frame_id = "world";
    ekf_pose_odom.header.stamp = ros::Time(ego_ekf_state.timestamp);

    Eigen::Quaterniond quat = Eigen::AngleAxisd(ego_ekf_state.yaw_rad, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(ego_ekf_state.pitch_rad, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(ego_ekf_state.roll_rad, Eigen::Vector3d::UnitX());

    ekf_pose_odom.pose.pose.position.x = ego_ekf_state.x_m;
    ekf_pose_odom.pose.pose.position.y = ego_ekf_state.y_m;
    ekf_pose_odom.pose.pose.position.z = ego_ekf_state.z_m;

    ekf_pose_odom.pose.pose.orientation.x = quat.x();
    ekf_pose_odom.pose.pose.orientation.y = quat.y();
    ekf_pose_odom.pose.pose.orientation.z = quat.z();
    ekf_pose_odom.pose.pose.orientation.w = quat.w();

    ekf_pose_odom.pose.covariance[0] = ego_ekf_state.x_cov_m;
    ekf_pose_odom.pose.covariance[7] = ego_ekf_state.y_cov_m;
    ekf_pose_odom.pose.covariance[14] = ego_ekf_state.z_cov_m;
    ekf_pose_odom.pose.covariance[21] = ego_ekf_state.roll_cov_rad;
    ekf_pose_odom.pose.covariance[28] = ego_ekf_state.pitch_cov_rad;
    ekf_pose_odom.pose.covariance[35] = ego_ekf_state.yaw_cov_rad;

    ekf_pose_odom.twist.twist.linear.x = ego_ekf_state.vx;
    ekf_pose_odom.twist.twist.linear.y = ego_ekf_state.vy;
    ekf_pose_odom.twist.twist.linear.z = ego_ekf_state.vz;

    ekf_pose_odom.twist.twist.angular.x = ego_ekf_state.roll_vel;
    ekf_pose_odom.twist.twist.angular.y = ego_ekf_state.pitch_vel;
    ekf_pose_odom.twist.twist.angular.z = ego_ekf_state.yaw_vel;

    o_ekf_pose_odom_ = ekf_pose_odom;

    pub_ekf_pose_odom_.publish(o_ekf_pose_odom_);

    // // // --- navsatfix ---
    // sensor_msgs::NavSatFix fix_msgs;
    // fix_msgs.header.stamp = ros::Time().fromSec(ego_ekf_state.timestamp);
    // fix_msgs.header.frame_id = "ego_frame";
    // fix_msgs.latitude = ego_ekf_state.latitude;
    // fix_msgs.longitude = ego_ekf_state.longitude;
    // fix_msgs.altitude = ego_ekf_state.height;
    // pub_satellite_nav_fix_.publish(fix_msgs);
}

void EkfLocalization::UpdateGpsOdom(EkfGnssMeasurement gnss) {
    nav_msgs::Odometry gps_pose_odom;

    // 헤더 정보 설정
    gps_pose_odom.header.frame_id = "world";
    gps_pose_odom.header.stamp = ros::Time(gnss.timestamp);

    // 위치 설정
    gps_pose_odom.pose.pose.position.x = gnss.pos.x();
    gps_pose_odom.pose.pose.position.y = gnss.pos.y();
    gps_pose_odom.pose.pose.position.z = gnss.pos.z();

    // 쿼터니언 회전 설정
    gps_pose_odom.pose.pose.orientation.x = gnss.rot.x();
    gps_pose_odom.pose.pose.orientation.y = gnss.rot.y();
    gps_pose_odom.pose.pose.orientation.z = gnss.rot.z();
    gps_pose_odom.pose.pose.orientation.w = gnss.rot.w();

    // 위치 공분산 할당
    gps_pose_odom.pose.covariance[0] = gnss.pos_cov(0, 0);  // x covariance
    gps_pose_odom.pose.covariance[7] = gnss.pos_cov(1, 1);  // y covariance
    gps_pose_odom.pose.covariance[14] = gnss.pos_cov(2, 2); // z covariance

    // 회전 공분산 할당
    gps_pose_odom.pose.covariance[21] = gnss.rot_cov(0, 0); // roll covariance
    gps_pose_odom.pose.covariance[28] = gnss.rot_cov(1, 1); // pitch covariance
    gps_pose_odom.pose.covariance[35] = gnss.rot_cov(2, 2); // yaw covariance

    // Odometry 메시지 발행
    o_gps_pose_odom_ = gps_pose_odom;
    pub_gps_pose_odom_.publish(o_gps_pose_odom_);
}

void EkfLocalization::UpdateEkfText(const EgoState ego_ekf_state) {
    jsk_rviz_plugins::OverlayText rviz_pos_type;
    rviz_pos_type.left = 300;
    rviz_pos_type.top = 0;
    rviz_pos_type.width = 1000;
    rviz_pos_type.height = 300;
    rviz_pos_type.text_size = 16;

    rviz_pos_type.fg_color.r = 0.5f;
    rviz_pos_type.fg_color.g = 0.5f;
    rviz_pos_type.fg_color.b = 1.0f;
    rviz_pos_type.fg_color.a = 1.0f;

    std::ostringstream lat_std_stream, lon_std_stream, x_cov_stream, y_cov_stream;
    lat_std_stream << std::fixed << std::setprecision(3) << ego_ekf_state.latitude_std;
    lon_std_stream << std::fixed << std::setprecision(3) << ego_ekf_state.longitude_std;
    x_cov_stream << std::fixed << std::setprecision(3) << ego_ekf_state.x_cov_m;
    y_cov_stream << std::fixed << std::setprecision(3) << ego_ekf_state.y_cov_m;

    rviz_pos_type.text = rviz_pos_type.text + "Lat_std:\t" + lat_std_stream.str() + "(m)";
    rviz_pos_type.text = rviz_pos_type.text + "\n" + "Lon std:\t" + lon_std_stream.str() + "(m)";
    rviz_pos_type.text = rviz_pos_type.text + "\n" + "X   std:\t" + x_cov_stream.str() + "(m)";
    rviz_pos_type.text = rviz_pos_type.text + "\n" + "Y   std:\t" + y_cov_stream.str() + "(m)";

    o_rviz_ekf_text_ = rviz_pos_type;
    pub_rviz_ekf_text_.publish(o_rviz_ekf_text_);

    // -------------------------------------------------------------------------------------
    // Plotter2D를 사용한 x, y, vx, vy, ax, ay 시각화
    std_msgs::Float32 x_plot, y_plot, z_plot, vx_plot, vy_plot, vz_plot, ax_plot, ay_plot, az_plot;
    std_msgs::Float32 roll_deg_plot, pitch_deg_plot, yaw_deg_plot;

    x_plot.data = ego_ekf_state.x_m;
    y_plot.data = ego_ekf_state.y_m;
    z_plot.data = ego_ekf_state.z_m;
    vx_plot.data = ego_ekf_state.vx;
    vy_plot.data = ego_ekf_state.vy;
    vz_plot.data = ego_ekf_state.vz;
    ax_plot.data = ego_ekf_state.ax;
    ay_plot.data = ego_ekf_state.ay;
    az_plot.data = ego_ekf_state.az;
    roll_deg_plot.data = ego_ekf_state.roll_rad * 180.0 / M_PI;
    pitch_deg_plot.data = ego_ekf_state.pitch_rad * 180.0 / M_PI;
    yaw_deg_plot.data = ego_ekf_state.yaw_rad * 180.0 / M_PI;

    pub_x_plot_.publish(x_plot);
    pub_y_plot_.publish(y_plot);
    pub_z_plot_.publish(z_plot);
    pub_vx_plot_.publish(vx_plot);
    pub_vy_plot_.publish(vy_plot);
    pub_vz_plot_.publish(vz_plot);
    pub_ax_plot_.publish(ax_plot);
    pub_ay_plot_.publish(ay_plot);
    pub_az_plot_.publish(az_plot);
    pub_roll_deg_plot_.publish(roll_deg_plot);
    pub_pitch_deg_plot_.publish(pitch_deg_plot);
    pub_yaw_deg_plot_.publish(yaw_deg_plot);
}


// interface::VehicleState EkfLocalization::EkfToVehicleState(const EgoState& ekf_state) {
//     interface::VehicleState vehicle_state;

//     // Header 초기화 필요시 설정
//     vehicle_state.header.seq = 0;
//     vehicle_state.header.stamp = ekf_state.timestamp;
//     vehicle_state.header.frame_id = "ego_frame";

//     vehicle_state.reference.projection = "local_cartesian";
//     vehicle_state.reference.wgs84.latitude = cfg_.ref_latitude;
//     vehicle_state.reference.wgs84.longitude = cfg_.ref_longitude;
//     vehicle_state.reference.wgs84.altitude = cfg_.ref_altitude;

//     vehicle_state.pos_type = interface::PositionOrVelocityType::INS_RTKFIXED; // TODO:

//     // GNSS 정보 설정
//     vehicle_state.gnss.latitude = ekf_state.latitude;
//     vehicle_state.gnss.longitude = ekf_state.longitude;
//     vehicle_state.gnss.altitude = ekf_state.height;

//     vehicle_state.gnss_stdev.latitude = ekf_state.latitude_std;
//     vehicle_state.gnss_stdev.longitude = ekf_state.longitude_std;
//     vehicle_state.gnss_stdev.altitude = ekf_state.height_std;

//     // 위치와 속도 설정 (ENU frame)
//     vehicle_state.x = ekf_state.x_m;
//     vehicle_state.y = ekf_state.y_m;
//     vehicle_state.z = ekf_state.z_m;

//     vehicle_state.vx = ekf_state.vx;
//     vehicle_state.vy = ekf_state.vy;
//     vehicle_state.vz = ekf_state.vz;

//     // 가속도 설정 (local coord)
//     vehicle_state.ax = ekf_state.ax;
//     vehicle_state.ay = ekf_state.ay;
//     vehicle_state.az = ekf_state.az;

//     // 회전 각도 설정 (world to vehicle)
//     vehicle_state.roll = ekf_state.roll_rad;
//     vehicle_state.pitch = ekf_state.pitch_rad;
//     vehicle_state.yaw = ekf_state.yaw_rad;

//     // 각속도 설정
//     vehicle_state.roll_vel = ekf_state.roll_vel;
//     vehicle_state.pitch_vel = ekf_state.pitch_vel;
//     vehicle_state.yaw_vel = ekf_state.yaw_vel;

//     // 표준편차 설정
//     vehicle_state.roll_stdev = sqrt(ekf_state.roll_cov_rad);
//     vehicle_state.pitch_stdev = sqrt(ekf_state.pitch_cov_rad);
//     vehicle_state.yaw_stdev = sqrt(ekf_state.yaw_cov_rad);

//     return vehicle_state;
// }

Eigen::Vector3d EkfLocalization::ProjectGpsPoint(const double& lat, const double& lon, const double& height) {
    GeographicLib::LocalCartesian local_cartesian(cfg_.ref_latitude, cfg_.ref_longitude, cfg_.ref_altitude);
    double x, y, z;
    local_cartesian.Forward(lat, lon, height, x, y, z);
    return Eigen::Vector3d(x, y, z);
}

void EkfLocalization::Exec(int num_thread) {
    boost::thread main_thread(boost::bind(&EkfLocalization::MainLoop, this));

    ros::AsyncSpinner spinner(num_thread);
    spinner.start();
    ros::waitForShutdown();

    main_thread.join();
}

void EkfLocalization::MainLoop() {
    ros::Rate loop_rate(task_rate_);
    ros::Time last_log_time = ros::Time::now();
    while (ros::ok()) {
        update_time_ = ros::Time::now();

        // Run algorithm
        Run();

        // Calculate execution time
        execution_time_ = ros::Time::now() - update_time_;

        if ((ros::Time::now() - last_log_time).toSec() >= 1.0) {
            if (execution_time_.toSec() > task_period_) {
                ROS_ERROR_STREAM("[" << task_name_ << "] Rate: " << task_period_ * 1000.0 <<
                                 "ms, Exec Time:" << (execution_time_).toSec() * 1000.0 << "ms");
            } else {
                ROS_INFO_STREAM("[" << task_name_ << "] Rate: " << task_period_ * 1000.0 <<
                                "ms, Exec Time:" << (execution_time_).toSec() * 1000.0 << "ms");
            }
            last_log_time = ros::Time::now();
        }

        // Publish topics
        PublishInThread();

        loop_rate.sleep();
    }
}

int main(int argc, char** argv) {
    std::string node_name = "ekf_localization";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double period;
    if (!nh.getParam("task_period/period_ekf_localization", period)) {
        period = 0.01;
    }

    EkfLocalization main_task(node_name, period);
    main_task.Exec(7);

    return 0;
}