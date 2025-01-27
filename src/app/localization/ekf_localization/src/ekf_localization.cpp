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

    Init();
}
EkfLocalization::~EkfLocalization() {}

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
    else if (cfg_.i_gps_type == GpsType::BESTPOS) {
        // sub_bestpos_ = nh.subscribe(cfg_.s_bestpos_topic_name, 1, &EkfLocalization::CallbackBestpos, this);
    }
    else if (cfg_.i_gps_type == GpsType::NAVSATFIX) {
        sub_navsatfix_ = nh.subscribe(cfg_.s_navsatfix_topic_name, 1, &EkfLocalization::CallbackNavsatFix, this);
    }

    sub_can_ = nh.subscribe(cfg_.s_can_topic_name, 1, &EkfLocalization::CallbackCAN, this);
    sub_imu_ = nh.subscribe(cfg_.s_imu_topic_name, 1, &EkfLocalization::CallbackImu, this);
    sub_pcm_odom_ = nh.subscribe("/app/loc/pcm_odom", 1, &EkfLocalization::CallbackPcmOdom, this);
    sub_pcm_init_odom_ = nh.subscribe("/app/loc/pcm_init_odom", 1, &EkfLocalization::CallbackPcmInitOdom, this);

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

    pose_estimation_params_.imu_gyro_std = cfg_.d_imu_std_gyro_dps;
    pose_estimation_params_.imu_acc_std = cfg_.d_imu_std_acc_mps;
    pose_estimation_params_.imu_bias_gyro_std = cfg_.d_ekf_imu_bias_cov_gyro;
    pose_estimation_params_.imu_bias_acc_std = cfg_.d_ekf_imu_bias_cov_acc;
    pose_estimation_params_.imu_gravity = cfg_.d_imu_gravity;
    pose_estimation_params_.state_std_pos_m = cfg_.d_state_std_pos_m;
    pose_estimation_params_.state_std_rot_rad = cfg_.d_state_std_rot_deg * M_PI / 180.0;
    pose_estimation_params_.state_std_vel_mps = cfg_.d_state_std_vel_mps;
    pose_estimation_params_.estimate_imu_bias = true;   
    pose_estimation_params_.estimate_gravity = cfg_.b_imu_estimate_gravity;

    pose_estimation_.Reset(pose_estimation_params_);


    ROS_INFO("Init function Done");
}

void EkfLocalization::CallbackNavsatFix(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    InertialPoseLib::GnssStruct gnss_meas;

    // Timestamp and source
    gnss_meas.timestamp = msg->header.stamp.toSec();

    // GPS coordinates (latitude, longitude, altitude)
    gnss_meas.pos = ProjectGpsPoint(msg->latitude, msg->longitude, msg->altitude);

    // GNSS measurement covariance for position
    gnss_meas.pos_cov.setZero();
    gnss_meas.pos_cov(0, 0) = std::pow(msg->position_covariance[0], 2); // latitude std
    gnss_meas.pos_cov(1, 1) = std::pow(msg->position_covariance[4], 2); // longitude std
    gnss_meas.pos_cov(2, 2) = std::pow(msg->position_covariance[8], 2); // altitude std

    // Rotation data is not included in NavSatFix, initialize with identity quaternion
    gnss_meas.rot = Eigen::Quaterniond::Identity();
    gnss_meas.rot_cov.setZero();

    // GPS 시각화
    UpdateGpsEgoMarker(gnss_meas);

    if (cfg_.b_use_gps == false) return;

    // GNSS uncertainty verification
    if (gnss_meas.pos_cov(0, 0) > cfg_.gnss_uncertainy_max_m || gnss_meas.pos_cov(1, 1) > cfg_.gnss_uncertainy_max_m)
        return;

    // Update GNSS using EKF algorithm
    pose_estimation_.UpdateWithGnss(gnss_meas);
    UpdateGpsOdom(gnss_meas);
}

void EkfLocalization::CallbackCAN(const geometry_msgs::TwistStampedConstPtr& msg) {
    // if (cfg_.b_use_can == false) return;
    // i_can_ = *msg;
    // CanStruct can_struct{0.0};

    // can_struct.timestamp = i_can_.header.stamp.toSec();
    // can_struct.vel.x() = i_can_.twist.linear.x;
    // can_struct.gyro.z() = i_can_.twist.angular.z;

    // pose_estimation_.PredictVelocity(can_struct);
}

void EkfLocalization::CallbackImu(const sensor_msgs::Imu::ConstPtr& msg) {
    // if (cfg_.b_use_imu == false) return;

    sensor_msgs::Imu imu_msg = ImuConverterToSensorMsg(*msg, cfg_.mat_ego_to_imu_rot);

    InertialPoseLib::ImuStruct imu_struct;
    
    imu_struct.timestamp = imu_msg.header.stamp.toSec();
    imu_struct.acc = Eigen::Vector3d(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
    imu_struct.gyro = Eigen::Vector3d(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);

    pose_estimation_.PredictImu(imu_struct);
    PublishInThread(); // Output Imu Prediction every period
}

void EkfLocalization::CallbackPcmOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    if (cfg_.b_use_pcm_matching == false) return;
    nav_msgs::Odometry pcm_odom = *msg;

    InertialPoseLib::GnssStruct gnss_meas, time_compensated_gnss;

    gnss_meas.timestamp = pcm_odom.header.stamp.toSec();
    // gnss_meas.gnss_source = GnssSource::PCM;

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
        pose_estimation_.UpdateWithGnss(time_compensated_gnss);
    }
}

void EkfLocalization::CallbackPcmInitOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    if (cfg_.b_use_pcm_matching == false) return;
    nav_msgs::Odometry pcm_init_odom = *msg;

    InertialPoseLib::GnssStruct gnss_meas, time_compensated_gnss;

    gnss_meas.timestamp = pcm_init_odom.header.stamp.toSec();
    // gnss_meas.gnss_source = GnssSource::PCM_INIT;

    // Position in global coordinates
    gnss_meas.pos = Eigen::Vector3d(pcm_init_odom.pose.pose.position.x, pcm_init_odom.pose.pose.position.y,
                                    pcm_init_odom.pose.pose.position.z);

    gnss_meas.rot = Eigen::Quaterniond(pcm_init_odom.pose.pose.orientation.w, pcm_init_odom.pose.pose.orientation.x,
                                       pcm_init_odom.pose.pose.orientation.y, pcm_init_odom.pose.pose.orientation.z);

    gnss_meas.pos_cov.setZero();
    gnss_meas.rot_cov.setZero();

    gnss_meas.pos_cov = Eigen::Matrix3d::Identity() * 1e-9;
    gnss_meas.rot_cov = Eigen::Matrix3d::Identity() * 1e-9;

    pose_estimation_.UpdateWithGnss(gnss_meas);
}

void EkfLocalization::Run() {
    // // If imu is not used for prediction, use ros time as the reference time for prediction
    // // If imu is used, perform prediction when Imu callback is called
    // if (cfg_.b_use_imu == true) return;

    // double cur_timestamp = ros::Time::now().toSec();

    // ProcessINI();
    // ptr_ekf_algorithm_->RunPrediction(cur_timestamp);
    // PublishInThread();
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

    }
}

// Interpolate the closest EKF State to compensate for the time difference between GNSS and EKF
bool EkfLocalization::GnssTimeCompensation(const InertialPoseLib::GnssStruct& i_gnss, InertialPoseLib::GnssStruct& o_gnss) {
    o_gnss = i_gnss;

    InertialPoseLib::EkfState closest_ekf_state, current_ekf_state;
    {
        std::lock_guard<std::mutex> lock(mutex_ekf_state_deque_);

        // If the EKF state queue is empty, compensation is not possible
        if (deq_ekf_state_.empty()) return false;

        current_ekf_state = deq_ekf_state_.back();

        // If the EKF state queue is empty, compensation is not possible
        if (deq_ekf_state_.front().timestamp > i_gnss.timestamp) return false;

        // Find the EKF state that is older than GNSS time
        for (const auto& ekf_state : deq_ekf_state_) {
            if (ekf_state.timestamp > i_gnss.timestamp) {
                closest_ekf_state = ekf_state;
                break;
            }
            closest_ekf_state = ekf_state; // Update the latest EKF state
        }
    }

    // Calculate the time difference between the GNSS time and the closest EKF state
    double d_gnss_to_ekf_time_sec = current_ekf_state.timestamp - i_gnss.timestamp;

    // If the EKF time is the same or older than the GNSS time, no compensation is needed
    if (d_gnss_to_ekf_time_sec <= 0.0) return true;

    // 위치 및 회전 보정 변수 초기화
    double dx{0.0}, dy{0.0}, dz{0.0};

     double ratio = 0.0;
    // Perform compensation if closest_ekf_state and current_ekf_state are different
    if (fabs(current_ekf_state.timestamp - closest_ekf_state.timestamp) > 1e-5) {
        ratio = d_gnss_to_ekf_time_sec / (current_ekf_state.timestamp - closest_ekf_state.timestamp);

        // Calculate relative displacement between EKF states
        dx = (current_ekf_state.pos.x() - closest_ekf_state.pos.x()) * ratio;
        dy = (current_ekf_state.pos.y() - closest_ekf_state.pos.y()) * ratio;
        dz = (current_ekf_state.pos.z() - closest_ekf_state.pos.z()) * ratio;
    }

    // Reflect relative displacement in GNSS
    o_gnss.timestamp = current_ekf_state.timestamp;
    o_gnss.pos.x() = i_gnss.pos.x() + dx;
    o_gnss.pos.y() = i_gnss.pos.y() + dy;
    o_gnss.pos.z() = i_gnss.pos.z() + dz;

    // 회전 보간을 위해 쿼터니언을 사용
    Eigen::Quaterniond delta_quaternion = closest_ekf_state.rot.slerp(ratio, current_ekf_state.rot);
    o_gnss.rot = i_gnss.rot * delta_quaternion;
    o_gnss.rot.normalize();

    // debug print
    if (cfg_.b_debug_print) {
        std::cout.precision(3);
        std::cout << "[GnssTimeCompensation] Time comp: " << d_gnss_to_ekf_time_sec << " Pos: " << dx << " " << dy
                  << " " << dz << std::endl;
    }

    return true;
}

// ----- Publish Result Functions ----- //
void EkfLocalization::PublishInThread() {
    InertialPoseLib::EkfState ego_ekf_state = pose_estimation_.GetCurrentState();

    {
        std::lock_guard<std::mutex> lock(mutex_ekf_state_deque_);
        if (deq_ekf_state_.size() < 1 || deq_ekf_state_.back().timestamp + 1e-5 < ego_ekf_state.timestamp)
            deq_ekf_state_.push_back(ego_ekf_state);

        if (deq_ekf_state_.back().timestamp > ego_ekf_state.timestamp) deq_ekf_state_.clear();

        while (deq_ekf_state_.size() > 1000) {
            deq_ekf_state_.pop_front();
        }
    }

    GeographicLib::LocalCartesian local_cartesian(cfg_.ref_latitude, cfg_.ref_longitude, cfg_.ref_altitude);
    double lat, lon, ele;
    local_cartesian.Reverse(ego_ekf_state.pos.x(), ego_ekf_state.pos.y(), ego_ekf_state.pos.z(), lat, lon, ele);

    // ego_ekf_state.latitude = lat; // TODO:
    // ego_ekf_state.longitude = lon;
    // ego_ekf_state.height = ele;

    UpdateEgoMarker(ego_ekf_state);
    UpdateTF(ego_ekf_state);
    UpdateEkfOdom(ego_ekf_state);
    UpdateEkfText(ego_ekf_state);
}

void EkfLocalization::UpdateEgoMarker(InertialPoseLib::EkfState ego_ekf_state) {
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

    // Adjust position to be 1.0m behind the center (along the x-axis in the local frame)
    double x_offset = 0.0;
    if (cfg_.i_vehicle_origin == 0) x_offset = 1.51;
    Eigen::Vector3d offset(x_offset, 0.0, o_ekf_ego_marker_msgs.scale.z / 2.0);
    Eigen::Vector3d adjusted_position =
            Eigen::Vector3d(ego_ekf_state.pos.x(), ego_ekf_state.pos.y(), ego_ekf_state.pos.z()) + ego_ekf_state.rot * offset;

    // Assign the adjusted position to the marker's pose
    o_ekf_ego_marker_msgs.pose.position.x = adjusted_position.x();
    o_ekf_ego_marker_msgs.pose.position.y = adjusted_position.y();
    o_ekf_ego_marker_msgs.pose.position.z = adjusted_position.z();

    // Assign quaternion to marker's orientation

    o_ekf_ego_marker_msgs.pose.orientation.x = ego_ekf_state.rot.x();
    o_ekf_ego_marker_msgs.pose.orientation.y = ego_ekf_state.rot.y();
    o_ekf_ego_marker_msgs.pose.orientation.z = ego_ekf_state.rot.z();
    o_ekf_ego_marker_msgs.pose.orientation.w = ego_ekf_state.rot.w();

    pub_ekf_ego_marker_.publish(o_ekf_ego_marker_msgs);
}

void EkfLocalization::UpdateGpsEgoMarker(InertialPoseLib::GnssStruct ego_gps_state) {
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

void EkfLocalization::UpdateTF(InertialPoseLib::EkfState ego_ekf_state) {
    tf::Transform transform;

    transform.setOrigin(tf::Vector3(ego_ekf_state.pos.x(), ego_ekf_state.pos.y(), ego_ekf_state.pos.z()));
    transform.setRotation(tf::Quaternion(ego_ekf_state.rot.x(), ego_ekf_state.rot.y(), ego_ekf_state.rot.z(), ego_ekf_state.rot.w()));

    tf::StampedTransform stampedTransform(transform, ros::Time(ego_ekf_state.timestamp), "world", "ego_frame");
    tf_broadcaster_.sendTransform(stampedTransform);
}

void EkfLocalization::UpdateEkfOdom(InertialPoseLib::EkfState ego_ekf_state) {
    nav_msgs::Odometry ekf_pose_odom;

    ekf_pose_odom.header.frame_id = "world";
    ekf_pose_odom.header.stamp = ros::Time(ego_ekf_state.timestamp);


    ekf_pose_odom.pose.pose.position.x = ego_ekf_state.pos.x();
    ekf_pose_odom.pose.pose.position.y = ego_ekf_state.pos.y();
    ekf_pose_odom.pose.pose.position.z = ego_ekf_state.pos.z();

    ekf_pose_odom.pose.pose.orientation.x = ego_ekf_state.rot.x();
    ekf_pose_odom.pose.pose.orientation.y = ego_ekf_state.rot.y();
    ekf_pose_odom.pose.pose.orientation.z = ego_ekf_state.rot.z();
    ekf_pose_odom.pose.pose.orientation.w = ego_ekf_state.rot.w();

    // ekf_pose_odom.pose.covariance[0] = ego_ekf_state.pos_cov(0, 0); // TODO:
    // ekf_pose_odom.pose.covariance[7] = ego_ekf_state.pos_cov(1, 1);
    // ekf_pose_odom.pose.covariance[14] = ego_ekf_state.pos_cov(2, 2);
    // ekf_pose_odom.pose.covariance[21] = ego_ekf_state.rot_cov(0, 0);
    // ekf_pose_odom.pose.covariance[28] = ego_ekf_state.rot_cov(1, 1);
    // ekf_pose_odom.pose.covariance[35] = ego_ekf_state.rot_cov(2, 2);

    ekf_pose_odom.twist.twist.linear.x = ego_ekf_state.vel.x();
    ekf_pose_odom.twist.twist.linear.y = ego_ekf_state.vel.y();
    ekf_pose_odom.twist.twist.linear.z = ego_ekf_state.vel.z();

    ekf_pose_odom.twist.twist.angular.x = ego_ekf_state.gyro.x();
    ekf_pose_odom.twist.twist.angular.y = ego_ekf_state.gyro.y();
    ekf_pose_odom.twist.twist.angular.z = ego_ekf_state.gyro.z();

    o_ekf_pose_odom_ = ekf_pose_odom;

    pub_ekf_pose_odom_.publish(o_ekf_pose_odom_);

}

void EkfLocalization::UpdateGpsOdom(InertialPoseLib::GnssStruct gnss) {
    nav_msgs::Odometry gps_pose_odom;

    gps_pose_odom.header.frame_id = "world";
    gps_pose_odom.header.stamp = ros::Time(gnss.timestamp);

    gps_pose_odom.pose.pose.position.x = gnss.pos.x();
    gps_pose_odom.pose.pose.position.y = gnss.pos.y();
    gps_pose_odom.pose.pose.position.z = gnss.pos.z();

    gps_pose_odom.pose.pose.orientation.x = gnss.rot.x();
    gps_pose_odom.pose.pose.orientation.y = gnss.rot.y();
    gps_pose_odom.pose.pose.orientation.z = gnss.rot.z();
    gps_pose_odom.pose.pose.orientation.w = gnss.rot.w();

    gps_pose_odom.pose.covariance[0] = gnss.pos_cov(0, 0);  // x covariance
    gps_pose_odom.pose.covariance[7] = gnss.pos_cov(1, 1);  // y covariance
    gps_pose_odom.pose.covariance[14] = gnss.pos_cov(2, 2); // z covariance

    gps_pose_odom.pose.covariance[21] = gnss.rot_cov(0, 0); // roll covariance
    gps_pose_odom.pose.covariance[28] = gnss.rot_cov(1, 1); // pitch covariance
    gps_pose_odom.pose.covariance[35] = gnss.rot_cov(2, 2); // yaw covariance

    o_gps_pose_odom_ = gps_pose_odom;
    pub_gps_pose_odom_.publish(o_gps_pose_odom_);
}

void EkfLocalization::UpdateEkfText(const InertialPoseLib::EkfState ego_ekf_state) {
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
    // lat_std_stream << std::fixed << std::setprecision(3) << ego_ekf_state.latitude_std; // TODO:
    // lon_std_stream << std::fixed << std::setprecision(3) << ego_ekf_state.longitude_std;
    // x_cov_stream << std::fixed << std::setprecision(3) << ego_ekf_state.x_cov_m;
    // y_cov_stream << std::fixed << std::setprecision(3) << ego_ekf_state.y_cov_m;

    rviz_pos_type.text = rviz_pos_type.text + "Lat_std:\t" + lat_std_stream.str() + "(m)";
    rviz_pos_type.text = rviz_pos_type.text + "\n" + "Lon std:\t" + lon_std_stream.str() + "(m)";
    rviz_pos_type.text = rviz_pos_type.text + "\n" + "X   std:\t" + x_cov_stream.str() + "(m)";
    rviz_pos_type.text = rviz_pos_type.text + "\n" + "Y   std:\t" + y_cov_stream.str() + "(m)";

    o_rviz_ekf_text_ = rviz_pos_type;
    pub_rviz_ekf_text_.publish(o_rviz_ekf_text_);

    // -------------------------------------------------------------------------------------
    std_msgs::Float32 x_plot, y_plot, z_plot, vx_plot, vy_plot, vz_plot, ax_plot, ay_plot, az_plot;
    std_msgs::Float32 roll_deg_plot, pitch_deg_plot, yaw_deg_plot;

    x_plot.data = ego_ekf_state.pos.x();
    y_plot.data = ego_ekf_state.pos.y();
    z_plot.data = ego_ekf_state.pos.z();
    vx_plot.data = ego_ekf_state.vel.x();
    vy_plot.data = ego_ekf_state.vel.y();
    vz_plot.data = ego_ekf_state.vel.z();
    ax_plot.data = ego_ekf_state.acc.x();
    ay_plot.data = ego_ekf_state.acc.y();
    az_plot.data = ego_ekf_state.acc.z();

    Eigen::Vector3d angles = RotToVec(ego_ekf_state.rot.toRotationMatrix());

    roll_deg_plot.data = angles(0) * 180.0 / M_PI;
    pitch_deg_plot.data = angles(1) * 180.0 / M_PI;
    yaw_deg_plot.data = angles(2) * 180.0 / M_PI;

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
        // Run();

        // Calculate execution time
        execution_time_ = ros::Time::now() - update_time_;

        // if ((ros::Time::now() - last_log_time).toSec() >= 1.0) {
        //     if (execution_time_.toSec() > task_period_) {
        //         ROS_ERROR_STREAM("[" << task_name_ << "] Rate: " << task_period_ * 1000.0 <<
        //                          "ms, Exec Time:" << (execution_time_).toSec() * 1000.0 << "ms");
        //     } else {
        //         ROS_INFO_STREAM("[" << task_name_ << "] Rate: " << task_period_ * 1000.0 <<
        //                         "ms, Exec Time:" << (execution_time_).toSec() * 1000.0 << "ms");
        //     }
        //     last_log_time = ros::Time::now();
        // }

        // Publish topics
        PublishInThread();

        loop_rate.sleep();
    }
}

int main(int argc, char** argv) {
    std::string node_name = "ekf_localization";
    ros::init(argc, argv, node_name);

    EkfLocalization main_task(node_name, 0.01);
    main_task.Exec(7);

    return 0;
}