/****************************************************************************/
// Module:      registration.cpp
// Description: registration algorithm
//
// Authors: Jaeyoung Jo (wodud3743@gmail.com)
// Version: 0.1
//
// Revision History
//      Oct 18, 2024: Jaeyoung Jo - Created.
//      XXXX XX, 2023: XXXXXXX XX -
/****************************************************************************/

#include "registration.hpp"

Eigen::Matrix4d Registration::AlignCloudsLocal(std::vector<PointStruct>& source_global,
                                               const std::vector<PointStruct>& target_global,
                                               Eigen::Matrix4d& last_icp_pose, double trans_th,
                                               RegistrationConfig m_config) {
    // sensor_velocity : 지난 센서 프레임 기준 상대 속도

    Eigen::Matrix6d JTJ = Eigen::Matrix6d::Zero(); // 6x6 J^T J
    Eigen::Vector6d JTr = Eigen::Vector6d::Zero(); // 6x1 J^T R

    Eigen::Matrix4d last_icp_pose_inv = last_icp_pose.inverse();

    double d_residual_sum = 0.0;

    for (size_t i = 0; i < source_global.size(); ++i) {
        Eigen::Vector4d target_hom_global(target_global[i].pose.x(), target_global[i].pose.y(),
                                          target_global[i].pose.z(), 1.0);
        Eigen::Vector4d target_hom_local = last_icp_pose_inv * target_hom_global;

        const Eigen::Vector3d target_local = target_hom_local.head<3>();
        const Eigen::Vector3d residual_local = target_local - source_global[i].local.head<3>();

        Eigen::Matrix3_6d J_g; // Jacobian Geometry (3x6)

        // 1. Jacobian Geometry
        // [ I(3x3), -(T p_k)^ ]
        J_g.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_g.block<3, 3>(0, 3) = -1.0 * vectorToSkewSymmetricMatrix(source_global[i].local);

        // double sensor_dist = source_global[i].local.norm();
        double weight_g = square(trans_th) / square(trans_th + residual_local.squaredNorm());

        // 6x6 += (3x6).t * (3x3) * (3x6)
        JTJ.noalias() += weight_g * J_g.transpose() * J_g;            // 6x3 3x6 = 6x6
        JTr.noalias() += weight_g * J_g.transpose() * residual_local; // 6x3 3x1 = 6x1

        d_residual_sum += residual_local.norm();
    }

    d_fitness_score_ = d_residual_sum / source_global.size();

    Eigen::Matrix6d JTJ_diag = JTJ.diagonal().asDiagonal();
    const Eigen::Vector6d x_tot = (JTJ + m_config.lm_lambda * JTJ_diag).ldlt().solve(JTr);

    Eigen::Vector3d rotation_vector = x_tot.tail<3>(); // rpy
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) =
            (Eigen::AngleAxisd(rotation_vector.norm(), rotation_vector.normalized())).toRotationMatrix(); // rotation
    transformation.block<3, 1>(0, 3) = x_tot.head<3>(); // transform xyz

    // Source Point 의 센서 좌표 기준 미소 transformation
    return transformation;
}

Eigen::Matrix4d Registration::AlignCloudsLocalSeperated(std::vector<PointStruct>& source_global,
                                                        const std::vector<PointStruct>& target_global,
                                                        Eigen::Matrix4d& last_icp_pose, double trans_th,
                                                        RegistrationConfig m_config) {
    // z, roll, pitch에 대한 최적화용 3x3 행렬 및 3x1 벡터
    Eigen::Matrix3d JTJ_zrp = Eigen::Matrix3d::Zero();
    Eigen::Vector3d JTr_zrp = Eigen::Vector3d::Zero();

    // x, y, yaw에 대한 최적화용 3x3 행렬 및 3x1 벡터
    Eigen::Matrix3d JTJ_xyy = Eigen::Matrix3d::Zero();
    Eigen::Vector3d JTr_xyy = Eigen::Vector3d::Zero();

    Eigen::Matrix4d last_icp_pose_inv = last_icp_pose.inverse();
    double d_residual_sum = 0.0;

    for (size_t i = 0; i < source_global.size(); ++i) {
        Eigen::Vector4d target_hom_global(target_global[i].pose.x(), target_global[i].pose.y(),
                                          target_global[i].pose.z(), 1.0);
        Eigen::Vector4d target_hom_local = last_icp_pose_inv * target_hom_global;

        const Eigen::Vector3d target_local = target_hom_local.head<3>();
        const Eigen::Vector3d residual_local = target_local - source_global[i].local.head<3>();

        double weight_g = square(trans_th) / square(trans_th + residual_local.squaredNorm());

        if (source_global[i].local.z() <= -m_config.ego_to_lidar_trans.z() + 1.0) {
            // z, roll, pitch에 대한 최적화
            Eigen::Vector3d J_g_zrp; // 1x3 : resi_z * (z, roll, pitch)
            J_g_zrp.x() = 1.0;       // z jacobian
            J_g_zrp.tail<2>() << source_global[i].local(1), -source_global[i].local(0); // roll pitch jacobian

            JTJ_zrp.noalias() += weight_g * J_g_zrp * J_g_zrp.transpose();
            JTr_zrp.noalias() += weight_g * J_g_zrp * residual_local.z();

            d_residual_sum += fabs(residual_local.z());
        }
        else {
            // x, y, yaw에 대한 최적화
            Eigen::Matrix2_3d J_g_xyy;                               // 2x3 : (resi_x, resi_y) * (x, y, yaw)
            J_g_xyy.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity(); // x , y jacobian
            J_g_xyy.block<2, 1>(0, 2) << -source_global[i].local(1),
                    source_global[i].local(0); // yaw jacobian (2x2 skewsymmetric)

            JTJ_xyy.noalias() += weight_g * J_g_xyy.transpose() * J_g_xyy;
            JTr_xyy.noalias() += weight_g * J_g_xyy.transpose() * residual_local.head<2>();

            d_residual_sum += residual_local.head<2>().norm();
        }

        // d_residual_sum += residual_local.norm();
    }

    d_fitness_score_ = d_residual_sum / source_global.size();

    // Levenberg-Marquardt 댐핑 적용하여 두 개의 해 계산
    Eigen::Matrix3d JTJ_zrp_diag = JTJ_zrp.diagonal().asDiagonal();
    Eigen::Matrix3d JTJ_xyy_diag = JTJ_xyy.diagonal().asDiagonal();

    const Eigen::Vector3d x_tot_zrp = (JTJ_zrp + m_config.lm_lambda * JTJ_zrp_diag).ldlt().solve(JTr_zrp);
    const Eigen::Vector3d x_tot_xyy = (JTJ_xyy + m_config.lm_lambda * JTJ_xyy_diag).ldlt().solve(JTr_xyy);

    Eigen::Vector3d rotation_vector = Eigen::Vector3d(x_tot_zrp.y(), x_tot_zrp.z(), x_tot_xyy.z()); // rpy
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) =
            (Eigen::AngleAxisd(rotation_vector.norm(), rotation_vector.normalized())).toRotationMatrix(); // rotation
    transformation.block<3, 1>(0, 3) << x_tot_xyy.x(), x_tot_xyy.y(), x_tot_zrp.x(); // transform xyz

    return transformation;
}

Eigen::Matrix4d Registration::AlignCloudsLocalPointCov(std::vector<PointStruct>& source_global,
                                                       const std::vector<PointStruct>& target_global,
                                                       Eigen::Matrix6d& local_cov, Eigen::Matrix4d& last_icp_pose,
                                                       double trans_th, RegistrationConfig m_config) {
    // sensor_velocity : 지난 센서 프레임 기준 상대 속도

    Eigen::Matrix6d JTJ = Eigen::Matrix6d::Zero(); // 6x6 J^T J
    Eigen::Vector6d JTr = Eigen::Vector6d::Zero(); // 6x1 J^T R

    Eigen::Matrix3d mahalanobis_local = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d sensor_rot = last_icp_pose.block<3, 3>(0, 0);
    Eigen::Matrix3d sensor_rot_inv = sensor_rot.inverse();

    Eigen::Matrix4d last_icp_pose_inv = last_icp_pose.inverse();

    double d_residual_sum = 0.0;

    for (size_t i = 0; i < source_global.size(); ++i) {
        const auto& target_cov = target_global[i].covariance;

        // 공분산 행렬에서 고유값과 고유벡터 계산
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(target_cov.cov);
        // 가장 작은 고유값에 해당하는 고유벡터가 법선 벡터
        Eigen::Vector3d vec_normal_global = eigen_solver.eigenvectors().col(0);

        // global 좌표계의 법선 벡터를 local 좌표계로 변환
        Eigen::Vector3d vec_normal_local = sensor_rot_inv.matrix() * vec_normal_global;
        vec_normal_local.normalize(); // 정규화

        Eigen::Vector4d target_hom_global(target_cov.mean(0), target_cov.mean(1), target_cov.mean(2), 1.0);
        Eigen::Vector4d target_hom_local = last_icp_pose_inv * target_hom_global;

        const Eigen::Vector3d target_local = target_hom_local.head<3>();
        const Eigen::Vector3d residual_local = target_local - source_global[i].local.head<3>();

        Eigen::Matrix3_6d J_g; // Jacobian Geometry (3x6)

        const auto& cov_A = Eigen::Matrix3d::Identity(); // source 센서 로컬 기준
        const auto& cov_B = target_cov.cov;              // target cov는 global 맵 기준
        // Eigen::Matrix3d RCR = cov_A + sensor_rot_inv.matrix() * cov_B * sensor_rot_inv.matrix().transpose(); // local
        Eigen::Matrix3d RCR = sensor_rot_inv.matrix() * cov_B * sensor_rot_inv.matrix().transpose(); // local

        if (m_config.use_radar_cov == true) {
            RCR += source_global[i].covariance.cov;
        }

        mahalanobis_local = RCR.inverse(); // local

        // 1. Jacobian Geometry
        // [ I(3x3), -(T p_k)^ ]
        J_g.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_g.block<3, 3>(0, 3) = -1.0 * vectorToSkewSymmetricMatrix(source_global[i].local);

        // double sensor_dist = source_global[i].local.norm();
        double weight_g = square(trans_th) / square(trans_th + residual_local.squaredNorm()) * 0.8 + 0.2;
        // weight_g = 1.0;

        // 6x6 += (3x6).t * (3x3) * (3x6)
        JTJ.noalias() += weight_g * J_g.transpose() * mahalanobis_local * J_g;            // 6x3 3x6 = 6x6
        JTr.noalias() += weight_g * J_g.transpose() * mahalanobis_local * residual_local; // 6x3 3x1 = 6x1

        // Point-to-plane 거리 계산 및 누적
        double d_point_to_plane_dist = std::fabs(residual_local.dot(vec_normal_local));

        // d_residual_sum += residual_local.norm();
        d_residual_sum += d_point_to_plane_dist;
    }

    d_fitness_score_ = d_residual_sum / source_global.size();

    Eigen::Matrix6d JTJ_diag = JTJ.diagonal().asDiagonal();
    Eigen::Matrix6d regularized_JTJ = JTJ + m_config.lm_lambda * JTJ_diag;
    const Eigen::Vector6d x_tot = regularized_JTJ.ldlt().solve(JTr);

    // Inverse of JTJ to compute covariance
    Eigen::Matrix6d covariance = regularized_JTJ.inverse();
    local_cov = covariance;

    Eigen::Vector3d rotation_vector = x_tot.tail<3>(); // rpy
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) =
            (Eigen::AngleAxisd(rotation_vector.norm(), rotation_vector.normalized())).toRotationMatrix(); // rotation
    transformation.block<3, 1>(0, 3) = x_tot.head<3>(); // transform xyz

    // Source Point 의 센서 좌표 기준 미소 transformation
    return transformation;
}

Eigen::Matrix4d Registration::AlignCloudsLocalVoxelCov(std::vector<PointStruct>& source_global,
                                                       const std::vector<CovStruct>& target_cov_global,
                                                       Eigen::Matrix4d& last_icp_pose, double trans_th,
                                                       RegistrationConfig m_config) {
    // sensor_velocity : 지난 센서 프레임 기준 상대 속도

    Eigen::Matrix6d JTJ = Eigen::Matrix6d::Zero(); // 6x6 J^T J
    Eigen::Vector6d JTr = Eigen::Vector6d::Zero(); // 6x1 J^T R

    Eigen::Matrix3d mahalanobis_local = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d sensor_rot = last_icp_pose.block<3, 3>(0, 0);
    Eigen::Matrix3d sensor_rot_inv = sensor_rot.inverse();

    Eigen::Matrix4d last_icp_pose_inv = last_icp_pose.inverse();

    double d_residual_sum = 0.0;
    // 각각의 source_global 포인트에 대해 가장 가까운 target_cov_global 공분산을 찾음
    for (size_t i = 0; i < source_global.size(); ++i) {
        // 타겟 포인트의 공분산의 중심 찾기
        const auto& target_cov = target_cov_global[i];

        // 타겟 공분산의 중심 좌표를 homogeneous 좌표로 변환 후 local 좌표로 변환
        Eigen::Vector4d target_hom_global(target_cov.mean(0), target_cov.mean(1), target_cov.mean(2), 1.0);
        Eigen::Vector4d target_hom_local = last_icp_pose_inv * target_hom_global;

        // 타겟 포인트의 로컬 좌표와 잔차 계산
        const Eigen::Vector3d target_local = target_hom_local.head<3>();
        const Eigen::Vector3d residual_local = target_local - source_global[i].local.head<3>();

        Eigen::Matrix3_6d J_g; // Jacobian Geometry (3x6)

        const auto& cov_A = Eigen::Matrix3d::Identity(); // source 센서 로컬 기준
        const auto& cov_B = target_cov.cov;              // target cov는 global 맵 기준
        // Eigen::Matrix3d RCR = cov_A + sensor_rot_inv.matrix() * cov_B * sensor_rot_inv.matrix().transpose(); // local
        Eigen::Matrix3d RCR = sensor_rot_inv.matrix() * cov_B * sensor_rot_inv.matrix().transpose(); // local
        if (m_config.use_radar_cov == true) {
            RCR += source_global[i].covariance.cov;
        }
        mahalanobis_local = RCR.inverse(); // local

        // 1. Jacobian Geometry
        // [ I(3x3), -(T p_k)^ ]
        J_g.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_g.block<3, 3>(0, 3) = -1.0 * vectorToSkewSymmetricMatrix(source_global[i].local);

        // 각 포인트에 대한 가중치 계산 (공분산 기반)
        double weight_g = square(trans_th) / square(trans_th + residual_local.squaredNorm());

        if (weight_g < 0.01) continue;

        // 6x6 += (3x6).t * (3x3) * (3x6)
        JTJ.noalias() += weight_g * J_g.transpose() * mahalanobis_local * J_g;            // 6x3 3x6 = 6x6
        JTr.noalias() += weight_g * J_g.transpose() * mahalanobis_local * residual_local; // 6x3 3x1 = 6x1

        d_residual_sum += residual_local.norm();
    }

    d_fitness_score_ = d_residual_sum / source_global.size();

    // Levenberg-Marquardt 방식 적용
    Eigen::Matrix6d JTJ_diag = JTJ.diagonal().asDiagonal();
    const Eigen::Vector6d x_tot = (JTJ + m_config.lm_lambda * JTJ_diag).ldlt().solve(JTr);

    // 회전 벡터 및 변환 계산
    Eigen::Vector3d rotation_vector = x_tot.tail<3>(); // rpy
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) =
            (Eigen::AngleAxisd(rotation_vector.norm(), rotation_vector.normalized())).toRotationMatrix(); // 회전 변환
    transformation.block<3, 1>(0, 3) = x_tot.head<3>();                                                   // 변환 xyz

    // Source Point 의 센서 좌표 기준 미소 transformation 반환
    return transformation;
}

Eigen::Matrix4d Registration::AlignCloudsGlobal(std::vector<PointStruct>& source_global,
                                                const std::vector<PointStruct>& target_global, double trans_th,
                                                RegistrationConfig m_config) {
    // sensor_velocity : 지난 센서 프레임 기준 상대 속도

    Eigen::Matrix6d JTJ = Eigen::Matrix6d::Zero(); // 6x6 J^T J
    Eigen::Vector6d JTr = Eigen::Vector6d::Zero(); // 6x1 J^T R

    for (size_t i = 0; i < source_global.size(); ++i) {
        const Eigen::Vector3d residual_global = target_global[i].pose - source_global[i].pose;

        Eigen::Matrix3_6d J_g;   // Jacobian Geometry (3x6)
        Eigen::Matrix1_3d R_tot; // Residual Total (1x3)

        // 1. Jacobian Geometry
        // [ I(3x3), -(T p_k)^ ]
        J_g.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_g.block<3, 3>(0, 3) = -1.0 * vectorToSkewSymmetricMatrix(source_global[i].pose);

        double weight_g = square(trans_th) / square(trans_th + residual_global.squaredNorm());

        if (std::isnan(weight_g)) {
            continue;
        }

        // Geometric Residual with weight
        R_tot.block<1, 3>(0, 0) = residual_global;

        // 6x6 += (3x6).t * (3x3) * (3x6)
        JTJ.noalias() += J_g.transpose() * weight_g * J_g;               // 6x3 3x6 = 6x6
        JTr.noalias() += J_g.transpose() * weight_g * R_tot.transpose(); // 6x3 3x1 = 6x1
    }

    Eigen::Matrix6d JTJ_diag = JTJ.diagonal().asDiagonal();
    const Eigen::Vector6d x_tot = (JTJ + m_config.lm_lambda * JTJ_diag).ldlt().solve(JTr);
    // const Eigen::Vector6d x_tot = JTJ.ldlt().solve(JTr);

    Eigen::Vector3d rotation_vector = x_tot.tail<3>(); // rpy
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) =
            (Eigen::AngleAxisd(rotation_vector.norm(), rotation_vector.normalized())).toRotationMatrix(); // rotation
    transformation.block<3, 1>(0, 3) = x_tot.head<3>(); // transform xyz

    // global transform
    return transformation;
}

// Local 좌표계의 frame과, voxel mmap, 초기위치 initial_guess
Eigen::Matrix4d Registration::RunRegister(const std::vector<PointStruct>& source_local, const VoxelHashMap& voxel_map,
                                          const Eigen::Matrix4d& initial_guess, RegistrationConfig m_config,
                                          bool& is_success, double& fitness_score, Eigen::Matrix6d& local_cov) {
    std::set<int> source_set; // source_global 중 association된 point의 index를 담음
    std::vector<std::pair<int, int>> vec_cor_origin_pair;
    std::vector<PointStruct> source_c_global, target_c_global;
    std::vector<CovStruct> target_cov_c_global;
    local_cov = Eigen::Matrix6d::Identity();

    int i_source_total_num = source_local.size();
    int i_source_corr_num = 0;
    double corres_ratio = 0.0;

    // 1. Global 좌표계의 source 생성 및 point uncertaintly 계산
    std::vector<PointStruct> source_global;
    source_global.resize(source_local.size());
    TransformPoints(initial_guess, source_local, source_global);

    if (voxel_map.Empty()) {
        std::cout << YELLOW << "VOXEL MAP EMPTY!" << RESET << std::endl;
        is_success = false;
        return initial_guess;
    }

    // 2. ICP 수행
    Eigen::Matrix4d last_icp_pose = initial_guess;
    Eigen::Matrix4d estimation_local = Eigen::Matrix4d::Identity(); // 센서 좌표계 상에서의 ICP 미소 변화량
    Velocity iter_velocity;

    if (m_config.use_radar_cov == true) {
        CalFramePointCov(source_global, m_config.range_variance_m, m_config.azimuth_variance_deg,
                         m_config.elevation_variance_deg);
    }

    auto start = std::chrono::steady_clock::now();
    int i_iteration = 0;
    double total_correspondence_time_ms = 0;
    for (int j = 0; j < m_config.max_iteration; ++j) {
        i_iteration++;
        // Get Correspodence in global frame
        // source_c_global : source_global 중에 유효하게 연관된 포인트
        // target_c_global : map에서 source_c_global과 연관된 포인트 혹은 cov

        auto correspondence_start = std::chrono::steady_clock::now();

        switch (m_config.icp_method) {
        case IcpMethod::P2P:
            std::tie(source_c_global, target_c_global) =
                    voxel_map.GetCorrespondencePoints(source_global, m_config.max_search_dist);
            break;
        case IcpMethod::GICP:
            std::tie(source_c_global, target_c_global) =
                    voxel_map.GetCorrespondencePoints(source_global, m_config.max_search_dist);
            break;
        case IcpMethod::VGICP:
            std::tie(source_c_global, target_cov_c_global) =
                    voxel_map.GetCorrespondencesCov(source_global, m_config.max_search_dist);
            break;
        case IcpMethod::AVGICP:
            std::tie(source_c_global, target_cov_c_global) =
                    voxel_map.GetCorrespondencesAllCov(source_global, m_config.max_search_dist);
            break;
        }

        auto correspondence_end = std::chrono::steady_clock::now();
        auto correspondence_elapsed_ms =
                std::chrono::duration_cast<std::chrono::microseconds>(correspondence_end - correspondence_start)
                        .count();
        total_correspondence_time_ms += correspondence_elapsed_ms;

        if (m_config.b_debug_print) {
            std::cout << "[Registration] Total Correspondence Time for: " << i_iteration << " in "
                      << (correspondence_elapsed_ms / 1000.0) << " ms, and cores num: " << source_c_global.size()
                      << RESET << std::endl;
        }

        i_source_corr_num = source_c_global.size();

        corres_ratio = (float)i_source_corr_num / i_source_total_num;
        if (corres_ratio < m_config.min_overlap_ratio) {
            std::cout << YELLOW << "[RunRegister] Small corresponding  ratio. " << corres_ratio << RESET << std::endl;
            is_success = false;
            return last_icp_pose;
        }

        switch (m_config.icp_method) {
        case IcpMethod::P2P:
            estimation_local = AlignCloudsLocal(source_c_global, target_c_global, last_icp_pose,
                                                m_config.max_search_dist, m_config);
            break;
        case IcpMethod::GICP:
            estimation_local = AlignCloudsLocalPointCov(source_c_global, target_c_global, local_cov, last_icp_pose,
                                                        m_config.max_search_dist, m_config);
            break;
        case IcpMethod::VGICP:
            estimation_local = AlignCloudsLocalVoxelCov(source_c_global, target_cov_c_global, last_icp_pose,
                                                        m_config.max_search_dist, m_config);
            break;
        case IcpMethod::AVGICP:
            estimation_local = AlignCloudsLocalVoxelCov(source_c_global, target_cov_c_global, last_icp_pose,
                                                        m_config.max_search_dist, m_config);
            break;
        }

        // 전역 좌표계상의 추정된 포즈 계산
        last_icp_pose = last_icp_pose * estimation_local;

        // ICP 종료 조건 확인
        Eigen::AngleAxisd angleAxis(estimation_local.block<3, 3>(0, 0));
        double rot_norm = angleAxis.angle();

        double transform_norm = rot_norm + estimation_local.block<3, 1>(0, 3).norm();
        if (transform_norm < m_config.icp_termination_threshold_m) {
            break;
        }

        // 전역 좌표계 변화량으로 source global 미소이동
        TransformPoints(last_icp_pose, source_local, source_global);
    }

    auto end = std::chrono::steady_clock::now();
    auto elapsed_ns = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    if (m_config.b_debug_print) {
        std::cout << "[Registration] Total Correspondence Time: " << (total_correspondence_time_ms / 1000.0) << " ms"
                  << RESET << std::endl;
        std::cout << "[Registration] RunRegister: iteration " << i_iteration << " executed in " << (elapsed_ns / 1000.0)
                  << " ms" << RESET << std::endl;

        std::cout << GREEN << "[RunRegister] Corresponding ratio " << corres_ratio << RESET << std::endl;
    }

    if (d_fitness_score_ > m_config.max_fitness_score) {
        std::cout << YELLOW << "[RunRegister] ICP Fitness Score Low " << d_fitness_score_ << RESET << std::endl;
        is_success = false;
        return last_icp_pose;
    }

    if (m_config.b_debug_print) {
        std::cout << GREEN << "[RunRegister] ICP Fitness Score " << d_fitness_score_ << RESET << std::endl;
    }

    fitness_score = d_fitness_score_;
    is_success = true;
    return last_icp_pose;
}