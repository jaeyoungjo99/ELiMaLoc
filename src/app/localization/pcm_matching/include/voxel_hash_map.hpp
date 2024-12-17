/****************************************************************************/
// Module:      voxel_hash_map.hpp
// Description: voxel_hash_map algorithm
//
// Authors: Jaeyoung Jo (wodud3743@gmail.com)
// Version: 0.1
//
// Revision History
//      June 19, 2024: Jaeyoung Jo - Created.
//      XXXX XX, 2023: XXXXXXX XX -
/****************************************************************************/

#ifndef __VOXEL_HASH_MAP_HPP__
#define __VOXEL_HASH_MAP_HPP__
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

// PCL
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/eigen.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/impl/angles.hpp>

#include "localization_functions.hpp"
#include "localization_struct.hpp"

struct CovStruct {
    Eigen::Matrix3d cov;  // 3x3 공분산 행렬
    Eigen::Vector3d mean; // 3D 평균 벡터

    CovStruct() : cov(Eigen::Matrix3d::Identity()), mean(Eigen::Vector3d::Zero()) {}

    CovStruct(const Eigen::Matrix3d &c, const Eigen::Vector3d &m) : cov(c), mean(m) {}

    void reset() {
        cov = Eigen::Matrix3d::Identity();
        mean = Eigen::Vector3d::Zero();
    }
};

struct PointStruct {
    Eigen::Vector3d pose;
    Eigen::Vector3d local;
    CovStruct covariance;
    float vel;       // mps
    float azi_angle; // deg Right to Left
    float ele_angle; // deg. Down to Up

    double intensity;

    // 생성자
    PointStruct()
        : pose(Eigen::Vector3d::Zero()),
          local(Eigen::Vector3d::Zero()),
          covariance(CovStruct()),
          vel(0.0),
          azi_angle(0.0),
          ele_angle(0.0),
          intensity(0.0) {}

    ~PointStruct(){};

    // reset 함수
    void reset() {
        pose.setZero();
        local.setZero();
        covariance.reset();
        vel = 0.0;
        azi_angle = 0.0;
        ele_angle = 0.0;
        intensity = 0.0;
    }
};

struct VoxelHashMap {
    using RadarPointVector = std::vector<PointStruct>;
    using RadarPointVectorTuple = std::tuple<RadarPointVector, RadarPointVector>;
    using Voxel = Eigen::Vector3i;

    struct VoxelBlock {
        // buffer of points with a max limit of n_points
        std::vector<PointStruct> points;
        CovStruct covariance;
        int num_points;
        double map_resolution;

        inline void AddPoint(const PointStruct &point, const int max_point) {
            if (points.size() < static_cast<size_t>(num_points)) {
                points.push_back(point);
            }
        }
        inline void AddPointWithSpacing(const PointStruct &point, const int max_point) {
            if (points.size() < static_cast<size_t>(num_points) &&
                std::none_of(points.cbegin(), points.cend(), [&](const auto &voxel_point) {
                    return (voxel_point.pose - point.pose).norm() < map_resolution;
                })) {
                points.push_back(point);
            }
        }
        inline void CalVoxelCov() {
            int n = points.size();

            covariance.cov = Eigen::Matrix3d::Identity();
            covariance.mean = Eigen::Vector3d::Zero();

            if (n == 0) {
                return;
            }
            else if (n == 1) {
                covariance.mean = points[0].pose;
                return;
            }

            // 행: 3(x,y,z), 열: 포인트 개수
            Eigen::Matrix<double, 3, -1> neighbors(3, n);

            // points 데이터를 neighbors에 복사
            for (int j = 0; j < n; j++) {
                neighbors.col(j) = points[j].pose;
            }

            // 각 행(축)별로 평균 계산
            Eigen::Vector3d mean = neighbors.rowwise().mean();

            // 평균을 각 열에서 빼고, 공분산 계산
            neighbors.colwise() -= mean;
            Eigen::Matrix3d cov = (neighbors * neighbors.transpose()) / (n - 1);

            // Plane regularization
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Vector3d values = Eigen::Vector3d(1, 1, 1e-3);

            cov = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();

            covariance.cov = cov;
            covariance.mean = mean;
        }
    };
    struct VoxelHash {
        size_t operator()(const Voxel &voxel) const {
            const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
            return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
        }
    };

    VoxelHashMap() {}
    VoxelHashMap(double voxel_size, int max_points_per_voxel) {
        voxel_size_ = voxel_size;
        max_points_per_voxel_ = max_points_per_voxel;
    }

    void Init(double voxel_size, int max_points_per_voxel);

    std::tuple<std::vector<PointStruct>, std::vector<PointStruct>> GetCorrespondencePoints(
            const RadarPointVector &vec_points, double d_max_correspondence_dist) const;

    std::tuple<std::vector<PointStruct>, std::vector<CovStruct>> GetCorrespondencesCov(
            const RadarPointVector &vec_points, double d_max_correspondence_dist) const;

    std::tuple<std::vector<PointStruct>, std::vector<CovStruct>> GetCorrespondencesAllCov(
            const RadarPointVector &vec_points, double d_max_correspondence_dist) const;

    std::vector<Voxel> GetAdjacentVoxels(const PointStruct &point, int range) const;

    inline Voxel PointToVoxel(const Eigen::Vector3d &point, const double voxel_size) const {
        return VoxelHashMap::Voxel(static_cast<int>(std::floor(point.x() / voxel_size)),
                                   static_cast<int>(std::floor(point.y() / voxel_size)),
                                   static_cast<int>(std::floor(point.z() / voxel_size)));
    }

    // 각 voxel에 대한 voxel 내 covariance 계산
    inline void CalVoxelCovAll() {
        // for (auto &[voxel, voxel_block] : map_) { // 각 복셀 순회
        //     (void)voxel;                          // 사용하지 않음을 컴파일러에 명시
        //     voxel_block.CalVoxelCov();
        // }

        tbb::parallel_for_each(map_.begin(), map_.end(), [](auto &voxel_pair) {
            (void)voxel_pair.first; // 사용하지 않음을 컴파일러에 명시
            voxel_pair.second.CalVoxelCov();
        });
    }

    inline void ProcessVoxelBlock(VoxelBlock &voxel_block, const double d_search_dist_squared) {
        // Iterate over each point in the voxel
        for (auto &point : voxel_block.points) {
            // Container for neighbors
            std::vector<Eigen::Vector3d> neighbors;

            // Include the current point's pose
            neighbors.push_back(point.pose);

            // Get adjacent voxels using the helper function
            std::vector<VoxelHashMap::Voxel> adjacent_voxels = GetAdjacentVoxels(point, 2);

            // Iterate over adjacent voxels
            for (const auto &neighbor_voxel : adjacent_voxels) {
                // Check if the neighboring voxel exists
                auto neighbor_it = map_.find(neighbor_voxel);
                if (neighbor_it == map_.end()) continue;

                const VoxelBlock &neighbor_block = neighbor_it->second;

                // Iterate over points in the neighboring voxel
                for (const auto &neighbor_point : neighbor_block.points) {
                    // Check distance
                    if ((neighbor_point.pose - point.pose).squaredNorm() <= d_search_dist_squared) {
                        neighbors.push_back(neighbor_point.pose);
                    }
                }
            }

            // Calculate covariance for the current point
            if (neighbors.size() == 1) {
                point.covariance.cov = Eigen::Matrix3d::Identity();
                point.covariance.mean = point.pose;
            }
            else {
                Eigen::Matrix<double, 3, -1> neighbor_matrix(3, neighbors.size());

                for (size_t i = 0; i < neighbors.size(); ++i) {
                    neighbor_matrix.col(i) = neighbors[i];
                }

                Eigen::Vector3d mean = neighbor_matrix.rowwise().mean();
                neighbor_matrix.colwise() -= mean;
                Eigen::Matrix3d cov = (neighbor_matrix * neighbor_matrix.transpose()) / (neighbors.size() - 1);

                // Plane regularization
                Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
                Eigen::Vector3d values = Eigen::Vector3d(1, 1, 1e-3);

                cov = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();

                point.covariance.cov = cov;
                point.covariance.mean = mean;
            }
        }
    }

    inline void CalPointCovAll(double d_search_dist) {
        // Square of the search distance for efficient comparison
        double d_search_dist_squared = d_search_dist * d_search_dist;
        tbb::parallel_for_each(map_.begin(), map_.end(),
                               [&](auto &voxel_pair) { ProcessVoxelBlock(voxel_pair.second, d_search_dist_squared); });
    }

    // voxel grid 통해 voxel 내 한 포인트만 남기는 sampling
    inline std::vector<PointStruct> VoxelDownsample(const std::vector<PointStruct> &points, const double voxel_size) {
        // Voxel을 키로, 첫 번째 SRadarPoint를 값으로 저장하는 해시맵
        std::unordered_map<VoxelHashMap::Voxel, PointStruct, VoxelHashMap::VoxelHash> grid;
        grid.reserve(points.size());

        // 각 SRadarPoint의 pose를 기반으로 Voxel 좌표를 계산한 후, voxel이 없으면 추가
        std::for_each(points.cbegin(), points.cend(), [&](const PointStruct &point) {
            VoxelHashMap::Voxel voxel = VoxelHashMap::PointToVoxel(point.pose, voxel_size);
            // voxel 내에 이미 point 있으면 skip TODO: 가장 중점이 남도록?
            if (grid.find(voxel) == grid.end()) {
                grid.insert({voxel, point});
            }
        });

        // 다운샘플링된 SRadarPoint들을 저장할 벡터
        std::vector<PointStruct> points_downsampled;
        points_downsampled.reserve(grid.size());

        // 해시맵에 있는 SRadarPoint들을 다운샘플링된 결과로 저장
        std::for_each(grid.cbegin(), grid.cend(),
                      [&](const auto &voxel_and_point) { points_downsampled.emplace_back(voxel_and_point.second); });

        return points_downsampled;
    }

    inline bool FindGroundHeight(const Eigen::Vector2d &position, double &ground_z) const {
        double d_search_range = 5.0;
        double d_squred_search_range = d_search_range * d_search_range;
        // 1. std::vector<PointStruct> Pointcloud() const; 함수를 사용하여, vec_map_points 를 추출.
        std::vector<PointStruct> vec_map_points = Pointcloud();

        // 2. vec_map_points 에서 x,y 값이 position 으로부터 d_search_range 이내에 있는 포인트 추출. 이때 point가 3개
        // 이하로만 서치된다면 return false
        std::vector<PointStruct> points_within_range;
        for (const auto &point : vec_map_points) {
            Eigen::Vector2d point_xy(point.pose.x(), point.pose.y());
            if ((point_xy - position).squaredNorm() <= d_squred_search_range) {
                points_within_range.push_back(point);
            }
        }

        // If less than 3 points are found, return false
        if (points_within_range.size() <= 3) {
            return false;
        }

        // 3. 가장 z가 작은 point N 개 추출.
        constexpr size_t N = 5; // Number of points to use for calculating ground height
        std::partial_sort(points_within_range.begin(),
                          points_within_range.begin() + std::min(N, points_within_range.size()),
                          points_within_range.end(),
                          [](const PointStruct &a, const PointStruct &b) { return a.pose.z() < b.pose.z(); });

        std::vector<Eigen::Vector3d> ground_points;
        for (size_t i = 0; i < std::min(N, points_within_range.size()); ++i) {
            ground_points.push_back(points_within_range[i].pose);
        }

        // 4. 해당 포인트의 covariance 계산하여 mean z 를 ground_z에 대입.
        Eigen::Matrix<double, 3, -1> neighbor_matrix(3, ground_points.size());
        for (size_t i = 0; i < ground_points.size(); ++i) {
            neighbor_matrix.col(i) = ground_points[i];
        }

        Eigen::Vector3d mean = neighbor_matrix.rowwise().mean();
        ground_z = mean.z(); // Set the calculated mean Z value as ground height

        return true;
    }

    inline void Clear() { map_.clear(); }
    inline bool Empty() const { return map_.empty(); }

    void Update(const RadarPointVector &points, const Eigen::Vector3d &origin);
    void AddPoints(const RadarPointVector &points);
    std::vector<PointStruct> Pointcloud() const;
    std::vector<CovStruct> Covariances() const;

    double voxel_size_;
    int max_points_per_voxel_;
    std::unordered_map<Voxel, VoxelBlock, VoxelHash> map_;
};

#endif // __VOXEL_HASH_MAP_HPP__