/**
 * @file pcm_matching.cpp
 * @author Jaeyoung Jo <wodud3743@gmail.com>
 * @brief Pointcloud map matching algorithm with deskewing
 * @version 0.1
 * @date 2024-10-28
 *
 * @copyright Copyright (c) 2024
 */

#include "pcm_matching.hpp"

PcmMatching::PcmMatching(std::string node_name, double period)
    : task_name_(node_name), task_period_(period), task_rate_(1.0 / period) {
    // Initialize
    ROS_WARN_STREAM("[" << node_name << "] Initialize node (Period: " << 1 / period << " Hz)");

    Init();
}
PcmMatching::~PcmMatching() {}

void PcmMatching::Init() {
    std::cout << REVERSE << "[PcmMatching] Init Started" << RESET << std::endl;
    NodeHandle nh;

    // 1. INI 파일 초기화 및 설정 로드
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/localization.ini");
    std::string calib_ini_dir("/config/calibration.ini");
    util_ini_parser_.Init((dir + ini_dir).c_str());
    util_calib_ini_parser_.Init((dir + calib_ini_dir).c_str());

    nh.getParam("/pcm_matching/map_path", cfg_.pcm_file_path);
    ProcessINI();

    // 2. ROS 토픽 설정
    // Subscribers
    sub_point_cloud_ = nh.subscribe(cfg_.s_lidar_topic_name, 1, &PcmMatching::CallbackPointCloud, this);
    sub_imu_ = nh.subscribe(cfg_.s_imu_topic_name, 1, &PcmMatching::CallbackImu, this);
    sub_ekf_geo_ = nh.subscribe("/app/loc/ekf_pose_odom", 1, &PcmMatching::CallbackEkfState, this);
    sub_initial_pose_ = nh.subscribe("/initialpose", 1, &PcmMatching::CallbackInitialPose, this);

    // Publishers
    pub_pcm_odom_ = nh.advertise<nav_msgs::Odometry>("/app/loc/pcm_odom", 1);
    pub_pcm_init_odom_ = nh.advertise<nav_msgs::Odometry>("/app/loc/pcm_init_odom", 1);
    pub_pcm_init_pc_ = nh.advertise<sensor_msgs::PointCloud2>("/app/loc/pcm_init_pc", 1);
    pub_undistort_pc_ = nh.advertise<sensor_msgs::PointCloud2>("/app/loc/undistort_pc", 1);
    pub_voxel_map_pc_ = nh.advertise<sensor_msgs::PointCloud2>("/app/loc/voxel_map_pc", 1);
    pub_voxel_map_cov_ = nh.advertise<visualization_msgs::MarkerArray>("/app/loc/voxel_map_cov", 1);
    pub_icp_map_pc_ = nh.advertise<sensor_msgs::PointCloud2>("/app/loc/icp_map_pc", 1);
    pub_icp_full_map_pc_ = nh.advertise<sensor_msgs::PointCloud2>("/app/loc/icp_full_map_pc", 1);

    // 3. Point Cloud 포인터 초기화
    undistort_pcptr_.reset(new pcl::PointCloud<PointType>());
    o_undistort_pcptr_.reset(new pcl::PointCloud<PointType>());
    o_voxel_map_pcptr_.reset(new pcl::PointCloud<PointType>());
    o_icp_map_pcptr_.reset(new pcl::PointCloud<PointType>());
    o_icp_full_map_pcptr_.reset(new pcl::PointCloud<PointType>());
    i_lidar_pcptr_.reset(new pcl::PointCloud<PointXYZIT>);

    // 4. TBB 스레드 설정
    int max_num_threads = registration_config_.i_max_thread > 0 ? registration_config_.i_max_thread
                                                                : tbb::this_task_arena::max_concurrency();
    tbb_control_ = std::make_unique<tbb::global_control>(tbb::global_control::max_allowed_parallelism,
                                                         static_cast<size_t>(max_num_threads));
    std::cout << "Use Thread  " << max_num_threads << " out of " << tbb::this_task_arena::max_concurrency() << RESET
              << std::endl;

    // 5. PCD 맵 파일 로드
    std::string pkg_dir(ros::package::getPath("pcm_matching") + "/../../../../");
    cfg_.pcm_file_path = (pkg_dir + cfg_.pcm_file_path);
    pcl::PointCloud<PointType>::Ptr map_pcptr(new pcl::PointCloud<PointType>());

    std::cout << BLINK << "[PcmMatching] Loading map points from " << cfg_.pcm_file_path << RESET << std::endl;
    if (pcl::io::loadPCDFile<PointType>(cfg_.pcm_file_path, *map_pcptr) == -1) {
        std::cout << RED << "[PcmMatching] Couldn't read the PCD file " << RESET << std::endl;
        ros::shutdown();
    }
    std::cout << REVERSE << "Loaded " << map_pcptr->width * map_pcptr->height << " map points" << RESET << std::endl;

    // 6. Registration 및 Local Map 초기화
    registration_.Init(registration_config_);
    std::vector<PointStruct> vec_map_points;
    Pcl2PointStruct(map_pcptr, vec_map_points);

    std::cout << BLINK << "[PcmMatching] Add points in voxel map ..." << RESET << std::endl;
    local_map_.Init(cfg_.d_pcm_voxel_size, cfg_.i_pcm_voxel_max_point);
    local_map_.AddPoints(vec_map_points);
    std::cout << REVERSE << "[PcmMatching] Add points done" << RESET << std::endl;

    // 7. Covariance 계산
    if (registration_config_.icp_method == IcpMethod::VGICP || registration_config_.icp_method == IcpMethod::AVGICP) {
        std::cout << "[PcmMatching] Start Cal Voxel Cov" << RESET << std::endl;
        local_map_.CalVoxelCovAll();
        std::cout << REVERSE << "[PcmMatching] Cal Voxel Cov done" << RESET << std::endl;
    }
    else if (registration_config_.icp_method == IcpMethod::GICP) {
        std::cout << "[PcmMatching] Start Cal Point Cov" << RESET << std::endl;
        local_map_.CalPointCovAll(registration_config_.gicp_cov_search_dist);
        std::cout << REVERSE << "[PcmMatching] Cal Point Cov done" << RESET << std::endl;
    }

    // 8. 맵 시각화
    std::vector<PointStruct> vec_point_map = local_map_.Pointcloud();
    std::vector<CovStruct> vec_cov_map = local_map_.Covariances();
    PointStruct2Pcl(vec_point_map, o_voxel_map_pcptr_, true);
    std::cout << "[PcmMatching] " << vec_point_map.size() << " points in voxel map " << std::endl;

    PublishCloud(pub_voxel_map_pc_, o_voxel_map_pcptr_, ros::Time::now(), "world");
    if (registration_config_.icp_method == IcpMethod::VGICP || registration_config_.icp_method == IcpMethod::AVGICP) {
        VisualizeCovMap(pub_voxel_map_cov_, vec_cov_map, ros::Time::now(), "world");
    }

    std::cout << REVERSE << GREEN << "[PcmMatching] Init function done" << RESET << std::endl;
}

void PcmMatching::Run() {}

void PcmMatching::Publish() {}

void PcmMatching::ProcessINI() {
    if (util_calib_ini_parser_.IsFileUpdated()) {
        int i_vehicle_origin = 0;
        util_calib_ini_parser_.ParseConfig("Vehicle Origin", "vehicle_origin", i_vehicle_origin);
        std::string str_origin = "";
        if (i_vehicle_origin == 0) {
            str_origin = "Rear";
        }
        else {
            str_origin = "CG";
        }
        util_calib_ini_parser_.ParseConfig(str_origin + " To Main LiDAR", "transform_xyz_m",
                                           cfg_.vec_d_ego_to_lidar_trans);
        util_calib_ini_parser_.ParseConfig(str_origin + " To Main LiDAR", "rotation_rpy_deg",
                                           cfg_.vec_d_ego_to_lidar_rot);
        util_calib_ini_parser_.ParseConfig(str_origin + " To Imu", "rotation_rpy_deg", cfg_.vec_d_ego_to_imu_rot);

        if (cfg_.vec_d_ego_to_lidar_trans.size() == 3 && cfg_.vec_d_ego_to_lidar_rot.size() == 3 &&
            cfg_.vec_d_ego_to_imu_rot.size() == 3) {
            registration_config_.ego_to_lidar_trans =
                    Eigen::Map<const Eigen::Vector3d>(cfg_.vec_d_ego_to_lidar_trans.data());

            Eigen::Vector3d euler_ego_to_lidar = Eigen::Vector3d(cfg_.vec_d_ego_to_lidar_rot[0] * M_PI / 180.0,
                                                                 cfg_.vec_d_ego_to_lidar_rot[1] * M_PI / 180.0,
                                                                 cfg_.vec_d_ego_to_lidar_rot[2] * M_PI / 180.0);
            Eigen::Vector3d euler_ego_to_imu = Eigen::Vector3d(cfg_.vec_d_ego_to_imu_rot[0] * M_PI / 180.0,
                                                               cfg_.vec_d_ego_to_imu_rot[1] * M_PI / 180.0,
                                                               cfg_.vec_d_ego_to_imu_rot[2] * M_PI / 180.0);
            registration_config_.ego_to_lidar_rot = VecToRot(euler_ego_to_lidar);
            registration_config_.ego_to_imu_rot = VecToRot(euler_ego_to_imu);
        }
        else {
            std::cout << RED << "[PcmMatching] Invalid Calibration!" << RESET << std::endl;
            ros::shutdown();
        }
        // ego에서 lidar로의 변환 행렬 생성
        cfg_.tf_ego_to_lidar = Eigen::Matrix4d::Identity();
        cfg_.tf_ego_to_lidar.block<3, 3>(0, 0) = registration_config_.ego_to_lidar_rot;
        cfg_.tf_ego_to_lidar.block<3, 1>(0, 3) = registration_config_.ego_to_lidar_trans;
    }

    if (util_ini_parser_.IsFileUpdated()) {
        // Node Configuration

        util_ini_parser_.ParseConfig("common_variable", "lidar_type", cfg_.s_lidar_type);
        util_ini_parser_.ParseConfig("common_variable", "lidar_scan_time_end", cfg_.b_lidar_scan_time_end);
        util_ini_parser_.ParseConfig("common_variable", "lidar_time_delay", cfg_.d_lidar_time_delay);
        util_ini_parser_.ParseConfig("common_variable", "lidar_topic_name", cfg_.s_lidar_topic_name);
        util_ini_parser_.ParseConfig("common_variable", "imu_topic_name", cfg_.s_imu_topic_name);

        util_ini_parser_.ParseConfig("pcm_matching", "debug_print", cfg_.b_debug_print);

        util_ini_parser_.ParseConfig("pcm_matching", "pcm_voxel_size", cfg_.d_pcm_voxel_size);
        util_ini_parser_.ParseConfig("pcm_matching", "pcm_voxel_max_point", cfg_.i_pcm_voxel_max_point);
        util_ini_parser_.ParseConfig("pcm_matching", "run_deskew", cfg_.b_run_deskew);
        util_ini_parser_.ParseConfig("pcm_matching", "input_max_dist", cfg_.d_input_max_dist);
        util_ini_parser_.ParseConfig("pcm_matching", "input_index_sampling", cfg_.i_input_index_sampling);
        util_ini_parser_.ParseConfig("pcm_matching", "input_voxel_ds_m", cfg_.d_input_voxel_ds_m);

        // ICP Parameters
        int i_temp = IcpMethod::P2P;
        util_ini_parser_.ParseConfig("pcm_matching", "icp_method", i_temp);
        registration_config_.icp_method = IcpMethod(i_temp);
        util_ini_parser_.ParseConfig("pcm_matching", "voxel_search_method", registration_config_.voxel_search_method);
        util_ini_parser_.ParseConfig("pcm_matching", "gicp_cov_search_dist", registration_config_.gicp_cov_search_dist);
        util_ini_parser_.ParseConfig("pcm_matching", "max_thread", registration_config_.i_max_thread);
        util_ini_parser_.ParseConfig("pcm_matching", "max_iteration", registration_config_.max_iteration);
        util_ini_parser_.ParseConfig("pcm_matching", "max_search_dist", registration_config_.max_search_dist);
        util_ini_parser_.ParseConfig("pcm_matching", "lm_lambda", registration_config_.lm_lambda);
        util_ini_parser_.ParseConfig("pcm_matching", "icp_termination_threshold_m",
                                     registration_config_.icp_termination_threshold_m);
        // ICP success parameters
        util_ini_parser_.ParseConfig("pcm_matching", "min_overlap_ratio", registration_config_.min_overlap_ratio);
        util_ini_parser_.ParseConfig("pcm_matching", "max_fitness_score", registration_config_.max_fitness_score);
        // Radar Parameters
        util_ini_parser_.ParseConfig("pcm_matching", "use_radar_cov", registration_config_.use_radar_cov);
        util_ini_parser_.ParseConfig("pcm_matching", "doppler_trans_lambda", registration_config_.doppler_trans_lambda);
        util_ini_parser_.ParseConfig("pcm_matching", "range_variance_m", registration_config_.range_variance_m);
        util_ini_parser_.ParseConfig("pcm_matching", "azimuth_variance_deg", registration_config_.azimuth_variance_deg);
        util_ini_parser_.ParseConfig("pcm_matching", "elevation_variance_deg",
                                     registration_config_.elevation_variance_deg);

        registration_config_.b_debug_print = cfg_.b_debug_print;
    }
}

void PcmMatching::CallbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    std::unique_lock<std::mutex> lock(mutex_pcl_, std::try_to_lock);
    if (lock.owns_lock() == false) {
        // 이 lock이 점유하지 못했다면(다른 process가 i_lidar_pcptr_ 접근중이라면) Registration 수행하지 않음
        // Initial Pose 우선 원칙
        return;
    }

    START_TIMER_NAMED(total);
    i_lidar_point_cloud_msg_ = *msg;

    ros::Duration d_time_delay(cfg_.d_lidar_time_delay);
    i_lidar_point_cloud_msg_.header.stamp -= d_time_delay;

    if (cfg_.s_lidar_type == "ouster") {
        *i_lidar_pcptr_ = OusterCloudmsg2cloud(i_lidar_point_cloud_msg_);
    }
    else {
        *i_lidar_pcptr_ = Cloudmsg2cloud(i_lidar_point_cloud_msg_);
    }

    if (i_lidar_pcptr_->points.empty()) {
        std::cout << YELLOW << "[PcmMatching] Input Empty! " << RESET << std::endl;
        return;
    }

    if (cfg_.b_debug_print) {
        std::cout << "[PcmMatching] Callback Point Num: " << i_lidar_pcptr_->points.size() << RESET << std::endl;
    }

    FilterPointsByDistance(i_lidar_pcptr_);
    // - - - - - - - - - - 1. Deskewing - - - - - - - - - - //
    double d_lidar_scan_start_time = i_lidar_point_cloud_msg_.header.stamp.toSec();
    if (DeskewPointCloud(i_lidar_pcptr_, d_lidar_scan_start_time) == false) {
        std::cout << YELLOW << "[PcmMatching] Deskew fail!" << RESET << std::endl;
        return;
    }

    // Publish undistorted point cloud in scan end
    PublishCloud(pub_undistort_pc_, undistort_pcptr_, ros::Time(d_time_scan_end_),
                 i_lidar_point_cloud_msg_.header.frame_id);

    // - - - - - - - - - - 2. Find Synced Pose - - - - - - - - - - //
    Eigen::Affine3f sync_ego_affine;
    if (GetInterpolatedPose(d_time_scan_end_, sync_ego_affine) == false) {
        std::cout << YELLOW << "[PcmMatching] Cannot Find synced pose!" << RESET << std::endl;
        return;
    }

    // source point struct화
    std::vector<PointStruct> vec_src_ori_lidar_points;
    Pcl2PointStruct(undistort_pcptr_, vec_src_ori_lidar_points);

    START_TIMER_NAMED(voxel_downsample);
    std::vector<PointStruct> vec_src_lidar_points =
            local_map_.VoxelDownsample(vec_src_ori_lidar_points, cfg_.d_input_voxel_ds_m);

    if (cfg_.b_debug_print) {
        std::cout << "[PcmMatching] DS Point Num: " << vec_src_lidar_points.size() << RESET << std::endl;
        STOP_TIMER_NAMED(voxel_downsample, "[VoxelDownsample]");
    }

    // - - - - - - - - - - Run registration - - - - - - - - - - //
    Eigen::Matrix4d sync_lidar_pose = sync_ego_affine.matrix().cast<double>() * cfg_.tf_ego_to_lidar;

    /*
        Run Main Registration Algorithm
        Input:
            source points(local), voxel map, init pose, last pose(for radar), dt(for radar)
            Translation sigma, Velocity sigma(for radar), config, ...
        Output:
            Registration result pose, success, fitness score(lower the better)
    */

    START_TIMER_NAMED(registration);
    bool b_icp_success = false;
    double d_fitness_score = 0.0;
    Eigen::Matrix4d icp_lidar_pose =
            registration_.RunRegister(vec_src_lidar_points, local_map_, sync_lidar_pose, registration_config_,
                                      b_icp_success, d_fitness_score, icp_local_cov_);

    if (cfg_.b_debug_print) {
        STOP_TIMER_NAMED(registration, "[RunRegister]");
    }

    // Check Registration Sucess
    if (b_icp_success == false) {
        std::cout << YELLOW << "[PcmMatching] ICP FAIL!" << RESET << std::endl;
        return;
    }

    // Check Fitness score as success
    cfg_.d_icp_pose_std_m = d_fitness_score;

    // ICP pose 데이터 publish
    Eigen::Matrix4d icp_ego_pose = icp_lidar_pose * cfg_.tf_ego_to_lidar.inverse();
    PublishPcmOdom(icp_ego_pose, ros::Time(d_time_scan_end_), "world");

    if (cfg_.b_debug_print) {
        STOP_TIMER_NAMED(total, "[PcmMatching w/o debug]");
    }

    // vec_src_lidar_points 를 world 좌표계로 변환
    registration_.TransformPoints(icp_lidar_pose, vec_src_lidar_points);
    PointStruct2Pcl(vec_src_lidar_points, o_icp_map_pcptr_);

    // vec_src_ori_lidar_points 를 world 좌표계로 변환
    registration_.TransformPoints(icp_lidar_pose, vec_src_ori_lidar_points);
    PointStruct2Pcl(vec_src_ori_lidar_points, o_icp_full_map_pcptr_);

    // ICP 결과를 위한 데이터 publish
    PublishCloud(pub_icp_map_pc_, o_icp_map_pcptr_, ros::Time(d_time_scan_end_), "world");
    PublishCloud(pub_icp_full_map_pc_, o_icp_full_map_pcptr_, ros::Time(d_time_scan_end_), "world");

    static double last_pcm_print_time = d_time_scan_end_;
    if (cfg_.b_debug_print || d_time_scan_end_ - last_pcm_print_time > 1.0) {
        STOP_TIMER_NAMED(total, "[PcmMatching w/debug]");
        last_pcm_print_time = d_time_scan_end_;
    }
}

void PcmMatching::CallbackImu(const sensor_msgs::Imu::ConstPtr& msg) {
    // imu의 측정 값을 ego 기준으로 변환
    sensor_msgs::Imu thisImu = ImuConverterToSensorMsg(*msg, registration_config_.ego_to_imu_rot);
    std::lock_guard<std::mutex> lock1(mutex_imu_);
    if (deq_imu_.size() > 0) {
        if (deq_imu_.back().header.stamp.toSec() > thisImu.header.stamp.toSec()) {
            deq_imu_.clear();
        }
    }
    deq_imu_.push_back(thisImu);
}

void PcmMatching::CallbackEkfState(const nav_msgs::Odometry::ConstPtr& msg) {
    nav_msgs::Odometry this_odom_msg = *msg;

    // 초기화 안된 pose는 받지 않음. 차후 deskewing에 악영향을 줌
    if (fabs(this_odom_msg.pose.pose.position.x) < 1e-9 || fabs(this_odom_msg.pose.pose.position.y) < 1e-9) return;

    std::lock_guard<std::mutex> lock2(mutex_odom_);
    if (deq_odom_.size() > 0) {
        if (deq_odom_.back().header.stamp.toSec() > this_odom_msg.header.stamp.toSec()) {
            deq_odom_.clear();
        }
    }
    deq_odom_.push_back(this_odom_msg);
}

void PcmMatching::CallbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_pcl_);
    b_initializing_status_ = true;

    // 복사 msg -> pose_stamped
    geometry_msgs::PoseWithCovarianceStamped pose_stamped = *msg;
    nav_msgs::Odometry o_init_pose_odom;

    // pose_stamped에서 pose를 추출하여 Eigen::Matrix4d로 변환
    // pose_stamped 에는 X, Y, Yaw만 유효함. Map에서 해당 Point의 Z 값을 찾는 과정 필요.
    Eigen::Matrix4d rviz_pose = Eigen::Matrix4d::Identity();
    rviz_pose.block<3, 3>(0, 0) =
            Eigen::Quaterniond(pose_stamped.pose.pose.orientation.w, pose_stamped.pose.pose.orientation.x,
                               pose_stamped.pose.pose.orientation.y, pose_stamped.pose.pose.orientation.z)
                    .toRotationMatrix();
    rviz_pose.block<3, 1>(0, 3) << pose_stamped.pose.pose.position.x, pose_stamped.pose.pose.position.y,
            pose_stamped.pose.pose.position.z;

    Eigen::Vector3d init_euler = RotToVec(rviz_pose.block<3, 3>(0, 0));

    // Print initial point info
    std::cout << MAGENTA << "[Init Pose] Get Initial Pose x: " << pose_stamped.pose.pose.position.x
              << " y: " << pose_stamped.pose.pose.position.y << " yaw: " << init_euler.z() * 180.0 / M_PI
              << " from Rviz !!" << RESET << std::endl;

    std::cout << MAGENTA << BLINK << "[Init Pose] Matching init pose ..." << RESET << std::endl;

    Eigen::Matrix4d ground_pose = rviz_pose;

    // 1. local_map_ 에서 rviz_pose 의 x,y 와 일정 거리 이내에 있는 지면의 높이 구하기. ground_pose에 rviz_pose에 z를
    // 반영한 값 넣기
    double z_ground = 0.0; // Default ground level
    bool found_ground = local_map_.FindGroundHeight(rviz_pose.block<3, 1>(0, 3).head<2>(), z_ground);
    if (found_ground) {
        ground_pose(2, 3) = z_ground; // Update Z value with the ground height
        std::cout << GREEN << "[Init Pose] Found ground Z: " << z_ground << RESET << std::endl;
    }
    else {
        std::cout << RED << "[Init Pose] Could not find ground height!" << rviz_pose(2, 3) << RESET << std::endl;
        return;
    }

    // 2. ground_pose 를 lidar pose로 바꾼뒤, 초기값으로 하여 ICP 수행. point cloud는 i_lidar_pcptr_ 사용
    Eigen::Matrix4d init_lidar_pose = ground_pose * cfg_.tf_ego_to_lidar;

    pcl::PointCloud<PointType>::Ptr point_type_lidar(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*i_lidar_pcptr_, *point_type_lidar);

    std::vector<PointStruct> vec_lidar_points;
    Pcl2PointStruct(point_type_lidar, vec_lidar_points);

    std::vector<PointStruct> vec_ds_lidar_points =
            local_map_.VoxelDownsample(vec_lidar_points, cfg_.d_input_voxel_ds_m);

    bool b_icp_success = false;
    double fitness_score = 0.0;
    Eigen::Matrix4d icp_lidar_pose =
            registration_.RunRegister(vec_ds_lidar_points, local_map_, init_lidar_pose, registration_config_,
                                      b_icp_success, fitness_score, icp_local_cov_);

    Eigen::Matrix4d final_pose = icp_lidar_pose * cfg_.tf_ego_to_lidar.inverse();

    if (b_icp_success == false) {
        std::cout << RED << "[Init Pose] ICP failed!" << RESET << std::endl;
        return;
    }

    cfg_.d_icp_pose_std_m = fitness_score;

    // 3. 최종 o_init_pose_odom publish
    o_init_pose_odom.header.stamp = ros::Time(d_time_scan_end_);
    o_init_pose_odom.header.frame_id = "world";
    o_init_pose_odom.pose.pose.position.x = final_pose(0, 3);
    o_init_pose_odom.pose.pose.position.y = final_pose(1, 3);
    o_init_pose_odom.pose.pose.position.z = final_pose(2, 3);

    Eigen::Quaterniond final_quat(final_pose.block<3, 3>(0, 0));
    o_init_pose_odom.pose.pose.orientation.x = final_quat.x();
    o_init_pose_odom.pose.pose.orientation.y = final_quat.y();
    o_init_pose_odom.pose.pose.orientation.z = final_quat.z();
    o_init_pose_odom.pose.pose.orientation.w = final_quat.w();

    Eigen::Vector3d final_euler = RotToVec(final_quat.toRotationMatrix());

    std::cout << REVERSE << MAGENTA << "[Init Pose] Publish Init x: " << o_init_pose_odom.pose.pose.position.x
              << " y: " << o_init_pose_odom.pose.pose.position.y << " z: " << o_init_pose_odom.pose.pose.position.z
              << " yaw: " << final_euler.z() * 180.0 / M_PI << RESET << std::endl;

    pcl::transformPointCloud(*point_type_lidar, *point_type_lidar, final_pose);
    pub_pcm_init_odom_.publish(o_init_pose_odom);
    PublishCloud(pub_pcm_init_pc_, point_type_lidar, ros::Time(d_time_scan_end_), "world");
}

// Utils

void PcmMatching::FilterPointsByDistance(pcl::PointCloud<PointXYZIT>::Ptr& input_pcptr) {
    // Remove points efficiently using remove_if and erase
    input_pcptr->points.erase(
            std::remove_if(input_pcptr->points.begin(), input_pcptr->points.end(),
                           [this](const PointXYZIT& point) {
                               double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
                               return distance > cfg_.d_input_max_dist;
                           }),
            input_pcptr->points.end());

    // Resize the cloud to match the new size after erasing
    input_pcptr->width = input_pcptr->points.size();
    input_pcptr->height = 1;
    input_pcptr->is_dense = true;
}

bool PcmMatching::DeskewPointCloud(const pcl::PointCloud<PointXYZIT>::Ptr& distort_pcptr, double timestamp) {
    std::lock_guard<std::mutex> lock1(mutex_imu_);
    std::lock_guard<std::mutex> lock2(mutex_odom_);

    START_TIMER_NAMED(deskew);

    b_first_point_ = true;
    d_time_scan_cur_ = timestamp;
    d_time_scan_end_ = timestamp + distort_pcptr->points.back().time;

    if (cfg_.b_lidar_scan_time_end == true) {
        double front_time = distort_pcptr->points.front().time;

        d_time_scan_end_ = timestamp;                     // scan end가 대표시간
        d_time_scan_cur_ = d_time_scan_end_ + front_time; // scan 시작 시간 줄이기. front 가 음수

        for (size_t i = 0; i < distort_pcptr->size(); i++) {
            distort_pcptr->points[i].time -= front_time; // 포인트 시간에 front 빼서 양수로 만듦
        }
    }

    int i_point_num = distort_pcptr->points.size();
    undistort_pcptr_->points.resize(i_point_num);

    ImuDeskewInfo();
    OdomDeskewInfo();

    if (b_is_imu_available_ == false || b_is_odom_available_ == false) {
        return false;
    }

    if (cfg_.b_run_deskew == true) {
        tbb::parallel_for(tbb::blocked_range<int>(0, i_point_num), [&](const tbb::blocked_range<int>& range) {
            for (int i = range.begin(); i != range.end(); ++i) {
                PointType cur_point;
                cur_point.x = distort_pcptr->points[i].x;
                cur_point.y = distort_pcptr->points[i].y;
                cur_point.z = distort_pcptr->points[i].z;
                cur_point.intensity = distort_pcptr->points[i].intensity;

                cur_point = DeskewPoint(&cur_point, distort_pcptr->points[i].time);

                undistort_pcptr_->points[i] = cur_point;
            }
        });
    }
    else {
        tbb::parallel_for(tbb::blocked_range<int>(0, i_point_num), [&](const tbb::blocked_range<int>& range) {
            for (int i = range.begin(); i != range.end(); ++i) {
                PointType cur_point;
                cur_point.x = distort_pcptr->points[i].x;
                cur_point.y = distort_pcptr->points[i].y;
                cur_point.z = distort_pcptr->points[i].z;
                cur_point.intensity = distort_pcptr->points[i].intensity;

                undistort_pcptr_->points[i] = cur_point;
            }
        });
    }

    if (cfg_.b_debug_print) {
        STOP_TIMER_NAMED(deskew, "[DeskewPointCloud]");
    }
    return true;
}

void PcmMatching::ImuDeskewInfo() {
    b_is_imu_available_ = false;

    while (!deq_imu_.empty()) {
        // 현재 scan 보다 예전것 지움
        if (deq_imu_.front().header.stamp.toSec() < d_time_scan_cur_ - 0.01)
            deq_imu_.pop_front();
        else
            break;
    }

    if (deq_imu_.empty()) {
        std::cout << YELLOW << "[PcmMatching] ImuDeskewInfo: Imu que is empty" << RESET << std::endl;
        return;
    }

    i_imu_pointer_cur_ = 0;

    for (int i = 0; i < (int)deq_imu_.size(); ++i) {
        sensor_msgs::Imu thisImuMsg = deq_imu_[i];
        double currentImuTime = thisImuMsg.header.stamp.toSec();

        // imu 시간이 scan + 0.01 이후라면 안씀
        if (currentImuTime > d_time_scan_end_ + 0.01) break;

        if (i_imu_pointer_cur_ == 0) {
            vec_d_imu_rot_x_[0] = 0;
            vec_d_imu_rot_y_[0] = 0;
            vec_d_imu_rot_z_[0] = 0;
            vec_d_imu_time_[0] = currentImuTime;
            ++i_imu_pointer_cur_;
            continue;
        }

        // get angular velocity
        double angular_x, angular_y, angular_z;
        ImuAngular2RosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

        // integrate rotation
        double d_time_diff = currentImuTime - vec_d_imu_time_[i_imu_pointer_cur_ - 1];
        vec_d_imu_rot_x_[i_imu_pointer_cur_] = vec_d_imu_rot_x_[i_imu_pointer_cur_ - 1] + angular_x * d_time_diff;
        vec_d_imu_rot_y_[i_imu_pointer_cur_] = vec_d_imu_rot_y_[i_imu_pointer_cur_ - 1] + angular_y * d_time_diff;
        vec_d_imu_rot_z_[i_imu_pointer_cur_] = vec_d_imu_rot_z_[i_imu_pointer_cur_ - 1] + angular_z * d_time_diff;
        vec_d_imu_time_[i_imu_pointer_cur_] = currentImuTime;
        ++i_imu_pointer_cur_;
    }

    --i_imu_pointer_cur_;

    if (i_imu_pointer_cur_ <= 0) return;

    b_is_imu_available_ = true;
}

void PcmMatching::OdomDeskewInfo() {
    b_is_odom_available_ = false; // 초기화

    // 현재 scan - 0.1 보다 과거의 odom 제거
    while (!deq_odom_.empty()) {
        if (deq_odom_.front().header.stamp.toSec() < d_time_scan_cur_ - 0.1)
            deq_odom_.pop_front();
        else
            break;
    }

    if (deq_odom_.empty()) {
        // Odom data is too old
        std::cout << YELLOW << "[PcmMatching] OdomDeskewInfo: Odom is too old" << RESET << std::endl;
        return;
    }
    // lidar 시작 ~ 라이다 시작 - 0.1초 사이에 odom 데이터가 없음
    if (deq_odom_.front().header.stamp.toSec() > d_time_scan_cur_) {
        std::cout << YELLOW << "[PcmMatching] OdomDeskewInfo: Cannot find synced Odom with lidar" << RESET << std::endl;
        return;
    }

    // 1. scan 시작보다 이후이면서 가장작은 odom 값 찾기
    // get start odometry at the beinning of the scan
    nav_msgs::Odometry start_odom_msg;
    for (int i = 0; i < (int)deq_odom_.size(); ++i) {
        start_odom_msg = deq_odom_[i];

        if (start_odom_msg.header.stamp.toSec() < d_time_scan_cur_)
            continue;
        else
            break;
    }
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(start_odom_msg.pose.pose.orientation, orientation);

    double roll, pitch, yaw;
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    Eigen::Affine3f affine_trans_begin =
            pcl::getTransformation(start_odom_msg.pose.pose.position.x, start_odom_msg.pose.pose.position.y,
                                   start_odom_msg.pose.pose.position.z, roll, pitch, yaw);

    // 2. scan end 이후이면서 가장 작은 odom 값 있으면 trans end 저장, 없으면 velocity로 적분
    nav_msgs::Odometry end_odom_msg;
    if (deq_odom_.back().header.stamp.toSec() > d_time_scan_end_) {
        if (cfg_.b_debug_print) {
            std::cout.precision(20);
            std::cout << "[OdomDeskewInfo] Interpolate for undistortion for time "
                      << deq_odom_.back().header.stamp.toSec() << RESET << std::endl;
            std::cout.precision(5);
        }

        for (int i = 0; i < (int)deq_odom_.size(); ++i) {
            end_odom_msg = deq_odom_[i];

            if (end_odom_msg.header.stamp.toSec() < d_time_scan_end_)
                continue;
            else
                break;
        }
    }
    else {
        // scan end 보다 이후의 odom 이 없으면 가장 마지막 시간으로부터 속도로 적분해서 사용
        nav_msgs::Odometry latest_odom_msg = deq_odom_.back();
        double d_lastest_odom_to_scan_end_sec = d_time_scan_end_ - deq_odom_.back().header.stamp.toSec();

        if (cfg_.b_debug_print) {
            std::cout.precision(5);
            std::cout << YELLOW << "[OdomDeskewInfo] Integrate for undistortion, dt " << d_lastest_odom_to_scan_end_sec
                      << RESET << std::endl;
        }

        end_odom_msg.header.stamp = ros::Time(d_time_scan_end_);
        end_odom_msg.header.frame_id = latest_odom_msg.header.frame_id;
        end_odom_msg.child_frame_id = latest_odom_msg.child_frame_id;

        // 마지막 odom의 orientation에서 roll, pitch, yaw 추출
        double roll, pitch, yaw;
        tf::Quaternion q(latest_odom_msg.pose.pose.orientation.x, latest_odom_msg.pose.pose.orientation.y,
                         latest_odom_msg.pose.pose.orientation.z, latest_odom_msg.pose.pose.orientation.w);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 1. 로컬 속도 추출
        double vx_local = latest_odom_msg.twist.twist.linear.x;
        double vy_local = latest_odom_msg.twist.twist.linear.y;
        double vz_local = latest_odom_msg.twist.twist.linear.z;

        // 2. Roll, Pitch, Yaw 회전 행렬 생성
        Eigen::Matrix3d mat_rotation;
        mat_rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

        // 3. 로컬 속도를 글로벌 속도로 변환
        Eigen::Vector3d local_velocity(vx_local, vy_local, vz_local);
        Eigen::Vector3d global_velocity = mat_rotation * local_velocity;

        // 4. 위치 적분 (글로벌 속도 기반)
        end_odom_msg.pose.pose.position.x =
                latest_odom_msg.pose.pose.position.x + global_velocity.x() * d_lastest_odom_to_scan_end_sec;
        end_odom_msg.pose.pose.position.y =
                latest_odom_msg.pose.pose.position.y + global_velocity.y() * d_lastest_odom_to_scan_end_sec;
        end_odom_msg.pose.pose.position.z =
                latest_odom_msg.pose.pose.position.z + global_velocity.z() * d_lastest_odom_to_scan_end_sec;

        // 5. 각속도를 사용해 roll, pitch, yaw 적분
        double roll_vel = latest_odom_msg.twist.twist.angular.x;
        double pitch_vel = latest_odom_msg.twist.twist.angular.y;
        double yaw_vel = latest_odom_msg.twist.twist.angular.z;

        roll += roll_vel * d_lastest_odom_to_scan_end_sec;
        pitch += pitch_vel * d_lastest_odom_to_scan_end_sec;
        yaw += yaw_vel * d_lastest_odom_to_scan_end_sec;

        // 적분된 roll, pitch, yaw를 쿼터니언으로 변환
        tf::Quaternion updatedQuat;
        updatedQuat.setRPY(roll, pitch, yaw);
        end_odom_msg.pose.pose.orientation.x = updatedQuat.x();
        end_odom_msg.pose.pose.orientation.y = updatedQuat.y();
        end_odom_msg.pose.pose.orientation.z = updatedQuat.z();
        end_odom_msg.pose.pose.orientation.w = updatedQuat.w();
    }

    tf::quaternionMsgToTF(end_odom_msg.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    Eigen::Affine3f affine_trans_end =
            pcl::getTransformation(end_odom_msg.pose.pose.position.x, end_odom_msg.pose.pose.position.y,
                                   end_odom_msg.pose.pose.position.z, roll, pitch, yaw);

    Eigen::Affine3f affine_trans_between = affine_trans_begin.inverse() * affine_trans_end;

    // Interpolation
    double dt_scan = d_time_scan_end_ - d_time_scan_cur_;
    double dt_trans = end_odom_msg.header.stamp.toSec() - start_odom_msg.header.stamp.toSec();

    Eigen::Affine3f affine_interpolated_transform = InterpolateTfWithTime(affine_trans_between, dt_scan, dt_trans);

    float f_roll_incre, f_pitch_incre, f_yaw_incre;
    pcl::getTranslationAndEulerAngles(affine_interpolated_transform, f_odom_incre_x_, f_odom_incre_y_, f_odom_incre_z_,
                                      f_roll_incre, f_pitch_incre, f_yaw_incre);

    b_is_odom_available_ = true;
}

void PcmMatching::FindRotation(double d_point_time, float* f_rot_x_cur, float* f_rot_y_cur, float* f_rot_z_cur) {
    *f_rot_x_cur = 0;
    *f_rot_y_cur = 0;
    *f_rot_z_cur = 0;

    int i_imu_pointer_front = 0;

    while (i_imu_pointer_front < i_imu_pointer_cur_) {
        if (d_point_time < vec_d_imu_time_[i_imu_pointer_front]) break;
        ++i_imu_pointer_front;
    }

    if (d_point_time > vec_d_imu_time_[i_imu_pointer_front] || i_imu_pointer_front == 0) {
        *f_rot_x_cur = vec_d_imu_rot_x_[i_imu_pointer_front];
        *f_rot_y_cur = vec_d_imu_rot_y_[i_imu_pointer_front];
        *f_rot_z_cur = vec_d_imu_rot_z_[i_imu_pointer_front];
    }
    else {
        int i_imu_pointer_back = i_imu_pointer_front - 1;
        double d_ratio_front = (d_point_time - vec_d_imu_time_[i_imu_pointer_back]) /
                               (vec_d_imu_time_[i_imu_pointer_front] - vec_d_imu_time_[i_imu_pointer_back]);
        double d_ratio_back = (vec_d_imu_time_[i_imu_pointer_front] - d_point_time) /
                              (vec_d_imu_time_[i_imu_pointer_front] - vec_d_imu_time_[i_imu_pointer_back]);

        *f_rot_x_cur = vec_d_imu_rot_x_[i_imu_pointer_front] * d_ratio_front +
                       vec_d_imu_rot_x_[i_imu_pointer_back] * d_ratio_back;
        *f_rot_y_cur = vec_d_imu_rot_y_[i_imu_pointer_front] * d_ratio_front +
                       vec_d_imu_rot_y_[i_imu_pointer_back] * d_ratio_back;
        *f_rot_z_cur = vec_d_imu_rot_z_[i_imu_pointer_front] * d_ratio_front +
                       vec_d_imu_rot_z_[i_imu_pointer_back] * d_ratio_back;
    }
}

void PcmMatching::FindPosition(double d_rel_time, float* f_pos_x_cur, float* f_pos_y_cur, float* f_pos_z_cur) {
    *f_pos_x_cur = 0;
    *f_pos_y_cur = 0;
    *f_pos_z_cur = 0;

    if (b_is_odom_available_ == false) return;

    float f_ratio = d_rel_time / (d_time_scan_end_ - d_time_scan_cur_); // to d_time_scan_cur_
    // float f_ratio = -(d_time_scan_end_ - d_time_scan_cur_ - d_rel_time) / (d_time_scan_end_ - d_time_scan_cur_);
    // to d_time_scan_end_. f_ratio는 음수가 됨

    *f_pos_x_cur = f_ratio * f_odom_incre_x_;
    *f_pos_y_cur = f_ratio * f_odom_incre_y_;
    *f_pos_z_cur = f_ratio * f_odom_incre_z_;
}

PointType PcmMatching::DeskewPoint(PointType* pcl_point, double d_rel_time) {
    if (b_is_imu_available_ == false) return *pcl_point;

    double d_point_time = d_time_scan_cur_ + d_rel_time;

    float f_rot_x_end, f_rot_y_end, f_rot_z_end; // scan end의 imu
    f_rot_x_end = vec_d_imu_rot_x_[i_imu_pointer_cur_];
    f_rot_y_end = vec_d_imu_rot_y_[i_imu_pointer_cur_];
    f_rot_z_end = vec_d_imu_rot_z_[i_imu_pointer_cur_];

    float f_rot_x_cur, f_rot_y_cur, f_rot_z_cur;
    FindRotation(d_point_time, &f_rot_x_cur, &f_rot_y_cur, &f_rot_z_cur);

    float f_pos_x_cur, f_pos_y_cur, f_pos_z_cur;
    FindPosition(d_rel_time, &f_pos_x_cur, &f_pos_y_cur, &f_pos_z_cur);

    float f_rot_x_from_end, f_rot_y_from_end, f_rot_z_from_end;
    f_rot_x_from_end = f_rot_x_cur - f_rot_x_end;
    f_rot_y_from_end = f_rot_y_cur - f_rot_y_end;
    f_rot_z_from_end = f_rot_z_cur - f_rot_z_end;

    float f_pos_x_from_end, f_pos_y_from_end, f_pos_z_from_end;
    f_pos_x_from_end = f_pos_x_cur - f_odom_incre_x_;
    f_pos_y_from_end = f_pos_y_cur - f_odom_incre_y_;
    f_pos_z_from_end = f_rot_z_cur - f_odom_incre_z_;

    // transform points to start
    // Eigen::Affine3f mat_trans_final = pcl::getTransformation(f_pos_x_cur, f_pos_y_cur, f_pos_z_cur,
    //                                                          f_rot_x_cur, f_rot_y_cur, f_rot_z_cur);
    Eigen::Affine3f mat_trans_final = pcl::getTransformation(f_pos_x_from_end, f_pos_y_from_end, f_pos_z_from_end,
                                                             f_rot_x_from_end, f_rot_y_from_end, f_rot_z_from_end);
    // Eigen::Affine3f mat_trans_bt = mat_trans_start_inverse_ * mat_trans_final;
    Eigen::Affine3f mat_trans_bt = mat_trans_final;

    PointType pcl_point_new;
    pcl_point_new.x = mat_trans_bt(0, 0) * pcl_point->x + mat_trans_bt(0, 1) * pcl_point->y +
                      mat_trans_bt(0, 2) * pcl_point->z + mat_trans_bt(0, 3);
    pcl_point_new.y = mat_trans_bt(1, 0) * pcl_point->x + mat_trans_bt(1, 1) * pcl_point->y +
                      mat_trans_bt(1, 2) * pcl_point->z + mat_trans_bt(1, 3);
    pcl_point_new.z = mat_trans_bt(2, 0) * pcl_point->x + mat_trans_bt(2, 1) * pcl_point->y +
                      mat_trans_bt(2, 2) * pcl_point->z + mat_trans_bt(2, 3);
    pcl_point_new.intensity = pcl_point->intensity;

    return pcl_point_new;
}

void PcmMatching::SortEigenvaluesAndEigenvectors(Eigen::Vector3d& vec_eigenvalues, Eigen::Matrix3d& mat_eigenvectors) {
    std::vector<std::pair<double, Eigen::Vector3d>> vec_pairs;
    for (int i = 0; i < 3; ++i) {
        vec_pairs.emplace_back(vec_eigenvalues(i), mat_eigenvectors.col(i));
    }

    // Eigenvalue 크기에 따라 정렬 (내림차순)
    std::sort(vec_pairs.begin(), vec_pairs.end(), [](const auto& a, const auto& b) { return a.first > b.first; });

    for (int i = 0; i < 3; ++i) {
        vec_eigenvalues(i) = vec_pairs[i].first;
        mat_eigenvectors.col(i) = vec_pairs[i].second;
    }

    // 회전 행렬의 유효성 검사 (determinant가 1인지 확인)
    if (mat_eigenvectors.determinant() < 0) {
        mat_eigenvectors.col(0) = -mat_eigenvectors.col(0);
    }
}

void PcmMatching::VisualizeCovMap(ros::Publisher& marker_pub, const std::vector<CovStruct>& vec_cov_map,
                                  ros::Time thisStamp, std::string thisFrame) {
    visualization_msgs::MarkerArray marker_array;
    std::cout << "[VisualizeCovMap] cov num: " << vec_cov_map.size() << RESET << std::endl;

    for (size_t i = 0; i < vec_cov_map.size(); ++i) {
        const CovStruct& cov_data = vec_cov_map[i];

        visualization_msgs::Marker marker;
        marker.header.frame_id = thisFrame;
        marker.header.stamp = thisStamp;
        marker.ns = "covariance_cylinders";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;

        // 공분산 행렬을 분석하여 Eigenvalue와 Eigenvector 계산
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_solver(cov_data.cov);
        Eigen::Vector3d eigenvalues = eig_solver.eigenvalues();
        Eigen::Matrix3d eigenvectors = eig_solver.eigenvectors();

        // Eigenvalue와 Eigenvector를 크기순으로 정렬
        SortEigenvaluesAndEigenvectors(eigenvalues, eigenvectors);

        // Orientation을 설정하기 위해 Eigenvector를 Quaternion으로 변환
        Eigen::Quaterniond quat(eigenvectors);
        marker.pose.orientation.x = quat.x();
        marker.pose.orientation.y = quat.y();
        marker.pose.orientation.z = quat.z();
        marker.pose.orientation.w = quat.w();

        // 공분산의 중심 위치 설정
        marker.pose.position.x = cov_data.mean(0);
        marker.pose.position.y = cov_data.mean(1);
        marker.pose.position.z = cov_data.mean(2);

        // Cylinder의 크기 설정 (가장 큰 eigenvalue를 높이로 사용)
        marker.scale.x = 3.0 * sqrt(eigenvalues(0) + 0.01); // x축 (주방향의 크기)
        marker.scale.y = 3.0 * sqrt(eigenvalues(1) + 0.01); // y축 (중간 크기)
        marker.scale.z = 3.0 * sqrt(eigenvalues(2) + 0.01); // z축 (Cylinder의 높이)

        // 색상 설정 (Eigenvector 방향에 따라 색상 달리)
        Eigen::Vector3d main_axis = eigenvectors.col(2); // 주축 (가장 큰 Eigenvalue에 해당하는 Eigenvector)
        marker.color.r = fabs(main_axis(0));             // X축 방향 성분을 빨강색에 반영
        marker.color.g = fabs(main_axis(1));             // Y축 방향 성분을 녹색에 반영
        marker.color.b = fabs(main_axis(2));             // Z축 방향 성분을 파랑색에 반영
        marker.color.a = 0.5;                            // 투명도

        // MarkerArray에 추가
        marker_array.markers.push_back(marker);
    }

    // MarkerArray를 퍼블리시
    marker_pub.publish(marker_array);
}

pcl::PointCloud<PointXYZIT> PcmMatching::OusterCloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg) {
    pcl::PointCloud<PointXYZIT> cloudresult;

    pcl::PointCloud<OusterPointXYZIRT>::Ptr i_tmp_ouster_cloud_ptr_(new pcl::PointCloud<OusterPointXYZIRT>());
    pcl::PointCloud<PointXYZIT>::Ptr tmp_cloud_result(new pcl::PointCloud<PointXYZIT>());

    pcl::moveFromROSMsg(cloudmsg, *i_tmp_ouster_cloud_ptr_);

    tmp_cloud_result->points.resize(static_cast<int>(i_tmp_ouster_cloud_ptr_->size() / cfg_.i_input_index_sampling) +
                                    1);
    tmp_cloud_result->is_dense = i_tmp_ouster_cloud_ptr_->is_dense;
    size_t result_index = 0;
    for (size_t i = 0; i < i_tmp_ouster_cloud_ptr_->size(); i += cfg_.i_input_index_sampling) {
        auto& src = i_tmp_ouster_cloud_ptr_->points[i];
        auto& dst = tmp_cloud_result->points[result_index];
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        dst.intensity = src.reflectivity;
        dst.time = src.t * 1e-9f;
        result_index++;
    }

    cloudresult = *tmp_cloud_result;
    return cloudresult;
}
pcl::PointCloud<PointXYZIT> PcmMatching::Cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg) {
    pcl::PointCloud<PointXYZIT> cloudresult;
    pcl::fromROSMsg(cloudmsg, cloudresult);
    return cloudresult;
}

// deq_odom_ 에서 d_cur_time 에 맞는 시간을 interpolation 하여 찾아내는 함수
bool PcmMatching::GetInterpolatedPose(double d_cur_time, Eigen::Affine3f& o_transform) {
    // 이전과 이후의 메시지를 찾기 위한 변수
    nav_msgs::Odometry odom_before, odom_after;
    bool b_found_before = false, b_found_after = false;

    // 이전 메시지와 이후 메시지 찾기
    for (size_t i = 0; i < deq_odom_.size(); ++i) {
        if (deq_odom_[i].header.stamp.toSec() <= d_cur_time) {
            odom_before = deq_odom_[i];
            b_found_before = true;
        }
        if (deq_odom_[i].header.stamp.toSec() > d_cur_time) {
            odom_after = deq_odom_[i];
            b_found_after = true;
            break; // 이전과 이후 메시지를 모두 찾으면 루프 종료
        }
    }

    // 이전 또는 이후 메시지를 찾지 못했으면 기본 변환 행렬 반환
    if (!b_found_before || !b_found_after) {
        std::cout << YELLOW << "odom last to scan cur time: " << d_cur_time - deq_odom_.back().header.stamp.toSec()
                  << RESET << std::endl;
        std::cout.precision(20);
        std::cout << YELLOW << "d_cur_time: " << d_cur_time << " Odom: " << deq_odom_.back().header.stamp.toSec()
                  << RESET << std::endl;
        return false;
    }

    // 이전, 이후 odometry 시간 계산
    double d_time_before = odom_before.header.stamp.toSec();
    double d_time_after = odom_after.header.stamp.toSec();

    // 시간 차이 계산
    double dt_scan = d_cur_time - d_time_before;
    double dt_trans = d_time_after - d_time_before;

    // Odometry 메시지에서 변환 행렬로 변환 (Eigen::Affine3f)
    Eigen::Affine3f affine_pose_before = Eigen::Affine3f::Identity();
    affine_pose_before.translation() << odom_before.pose.pose.position.x, odom_before.pose.pose.position.y,
            odom_before.pose.pose.position.z;
    Eigen::Quaternionf quatBefore(odom_before.pose.pose.orientation.w, odom_before.pose.pose.orientation.x,
                                  odom_before.pose.pose.orientation.y, odom_before.pose.pose.orientation.z);
    affine_pose_before.rotate(quatBefore);

    Eigen::Affine3f poseAfter = Eigen::Affine3f::Identity();
    poseAfter.translation() << odom_after.pose.pose.position.x, odom_after.pose.pose.position.y,
            odom_after.pose.pose.position.z;
    Eigen::Quaternionf quatAfter(odom_after.pose.pose.orientation.w, odom_after.pose.pose.orientation.x,
                                 odom_after.pose.pose.orientation.y, odom_after.pose.pose.orientation.z);
    poseAfter.rotate(quatAfter);

    // 이전 상태에서 이후 상태로의 변환 계산
    Eigen::Affine3f affine_trans_between = affine_pose_before.inverse() * poseAfter;

    // 보간을 통해 변환 계산
    Eigen::Affine3f affine_interpolated_pose = InterpolateTfWithTime(affine_trans_between, dt_scan, dt_trans);

    // 최종 보간된 변환 행렬을 반환
    o_transform = affine_pose_before * affine_interpolated_pose;
    return true;
}

void PcmMatching::PublishPcmOdom(Eigen::Matrix4d icp_ego_pose, ros::Time thisStamp, std::string thisFrame) {
    nav_msgs::Odometry pcm_pose_odom;

    pcm_pose_odom.header.frame_id = thisFrame;
    pcm_pose_odom.header.stamp = thisStamp;

    // 위치 정보 추출
    Eigen::Vector3d position = icp_ego_pose.block<3, 1>(0, 3);

    pcm_pose_odom.pose.pose.position.x = position.x();
    pcm_pose_odom.pose.pose.position.y = position.y();
    pcm_pose_odom.pose.pose.position.z = position.z();

    // 회전 정보 추출
    Eigen::Matrix3d rotation_matrix = icp_ego_pose.block<3, 3>(0, 0);

    Eigen::Vector3d euler_angles = RotToVec(rotation_matrix);

    // 쿼터니언으로 변환
    Eigen::Quaterniond quat(rotation_matrix);

    pcm_pose_odom.pose.pose.orientation.x = quat.x();
    pcm_pose_odom.pose.pose.orientation.y = quat.y();
    pcm_pose_odom.pose.pose.orientation.z = quat.z();
    pcm_pose_odom.pose.pose.orientation.w = quat.w();

    /*
        ICP Covariance 계산

        icp JtJ 함수의 inverse 인 icp_local_cov_ 사용.
        1. Translation 과 Rotation을 별도로 계산
        2. Covariance의 대각 행렬중 최솟값을 기준으로 matrix 나머지 원소 정규화
        3. Translation은 정규화 매트릭스에 d_icp_pose_std_m 곱하고 Rotation은 angle_std를 곱함
    */

    double d_icp_pose_std_m = std::max(cfg_.d_icp_pose_std_m, 0.25);

    // Translation uncertainty (3x3 block)
    Eigen::Matrix3d translation_covariance =
            rotation_matrix * icp_local_cov_.block<3, 3>(0, 0) * rotation_matrix.transpose();
    Eigen::Vector3d translation_cov_norm = NormalizeDiagonalCovariance(translation_covariance);

    // Rotation uncertainty (3x3 block)
    Eigen::Matrix3d rotation_covariance = icp_local_cov_.block<3, 3>(3, 3);
    Eigen::Vector3d rotation_cov_norm = NormalizeDiagonalCovariance(rotation_covariance);

    double angle_std = d_icp_pose_std_m * M_PI / 180.0; // 라디안 단위

    Eigen::Matrix3d translation_cov_norm_mat =
            NormalizeCovariance(translation_covariance) * d_icp_pose_std_m * d_icp_pose_std_m;
    Eigen::Matrix3d rotation_cov_norm_mat = NormalizeCovariance(rotation_covariance) * angle_std * angle_std;
    UpdateCovarianceField(pcm_pose_odom.pose.covariance, translation_cov_norm_mat, rotation_cov_norm_mat);

    pub_pcm_odom_.publish(pcm_pose_odom);
}

void PcmMatching::Exec(int num_thread) {
    boost::thread main_thread(boost::bind(&PcmMatching::MainLoop, this));

    ros::AsyncSpinner spinner(num_thread);
    spinner.start();
    ros::waitForShutdown();

    main_thread.join();
}

void PcmMatching::MainLoop() {
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
        Publish();

        loop_rate.sleep();
    }
}

int main(int argc, char** argv) {
    std::string node_name = "pcm_matching";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double period;
    if (!nh.getParam("task_period/period_pcm_matching", period)) {
        period = 1.0;
    }

    PcmMatching main_task(node_name, period);
    main_task.Exec(5); // 5개의 callback 함수 사용

    return 0;
}