#include "voxel_hash_map.hpp"

// Variables and functions only used in this cpp
namespace {
struct ResultTuple {
    ResultTuple() = default;
    ResultTuple(std::size_t n) {
        vec_source.reserve(n);
        vec_target.reserve(n);
    }
    std::vector<PointStruct> vec_source;
    std::vector<PointStruct> vec_target;
};

struct ResultTupleCov {
    ResultTupleCov() = default;
    ResultTupleCov(std::size_t n) {
        vec_source.reserve(n);
        vec_target.reserve(n);
    }
    std::vector<PointStruct> vec_source;
    std::vector<CovStruct> vec_target;
};
} // namespace

void VoxelHashMap::Init(double voxel_size, int max_points_per_voxel) {
    voxel_size_ = voxel_size;
    max_points_per_voxel_ = max_points_per_voxel;
}

std::tuple<std::vector<PointStruct>, std::vector<PointStruct>> VoxelHashMap::GetCorrespondencePoints(
        const RadarPointVector& vec_points, double d_max_correspondence_dist) const {
    const double d_max_dist_squared = d_max_correspondence_dist * d_max_correspondence_dist;

    auto GetClosestNeighbor = [&](const PointStruct& point) -> PointStruct {
        std::vector<Voxel> vec_voxels = GetAdjacentVoxels(point, 2);
        PointStruct closest_neighbor;
        double d_closest_dist_squared = std::numeric_limits<double>::max();

        for (const auto& voxel : vec_voxels) {
            auto search = map_.find(voxel);
            if (search != map_.end()) {
                for (const auto& neighbor : search->second.points) {
                    double d_dist_squared = (neighbor.pose - point.pose).squaredNorm();
                    if (d_dist_squared < d_closest_dist_squared) {
                        closest_neighbor = neighbor;
                        d_closest_dist_squared = d_dist_squared;
                    }
                }
            }
        }
        return closest_neighbor;
    };

    using points_iterator = RadarPointVector::const_iterator;

    auto result = tbb::parallel_reduce(
            tbb::blocked_range<points_iterator>(vec_points.cbegin(), vec_points.cend()), ResultTuple(0),
            [&](const tbb::blocked_range<points_iterator>& r, ResultTuple res) -> ResultTuple {
                auto& [vec_source, vec_target] = res;

                for (auto it = r.begin(); it != r.end(); ++it) {
                    const auto& point = *it;
                    PointStruct closest_neighbor = GetClosestNeighbor(point);

                    if ((closest_neighbor.pose - point.pose).squaredNorm() < d_max_dist_squared) {
                        vec_source.emplace_back(point);
                        vec_target.emplace_back(closest_neighbor);
                    }
                }

                return res;
            },
            [](ResultTuple a, const ResultTuple& b) -> ResultTuple {
                auto& [vec_source_a, vec_target_a] = a;
                const auto& [vec_source_b, vec_target_b] = b;

                vec_source_a.insert(vec_source_a.end(), std::make_move_iterator(vec_source_b.begin()),
                                    std::make_move_iterator(vec_source_b.end()));
                vec_target_a.insert(vec_target_a.end(), std::make_move_iterator(vec_target_b.begin()),
                                    std::make_move_iterator(vec_target_b.end()));

                return a;
            });

    auto& [vec_source, vec_target] = result;
    return std::make_pair(std::move(vec_source), std::move(vec_target));
}

std::tuple<std::vector<PointStruct>, std::vector<CovStruct>> VoxelHashMap::GetCorrespondencesCov(
        const RadarPointVector& vec_points, double d_max_correspondence_dist) const {
    auto GetClosestCovariance = [&](const PointStruct& point) {
        std::vector<Voxel> vec_voxels = GetAdjacentVoxels(point, 2);
        std::vector<CovStruct> vec_neighbors_cov;
        vec_neighbors_cov.reserve(vec_voxels.size());

        for (const auto& voxel : vec_voxels) {
            auto search = map_.find(voxel);
            if (search != map_.end() && search->second.points.size() > 0) {
                vec_neighbors_cov.push_back(search->second.covariance);
            }
        }

        CovStruct closest_cov;
        double d_closest_dist_squared = std::numeric_limits<double>::max();
        for (const auto& cov : vec_neighbors_cov) {
            double d_dist_squared = (cov.mean - point.pose).squaredNorm();
            if (d_dist_squared < d_closest_dist_squared) {
                closest_cov = cov;
                d_closest_dist_squared = d_dist_squared;
            }
        }

        return closest_cov;
    };

    using points_iterator = RadarPointVector::const_iterator;
    const double d_max_dist_squared = d_max_correspondence_dist * d_max_correspondence_dist;

    auto result = tbb::parallel_reduce(
            tbb::blocked_range<points_iterator>(vec_points.cbegin(), vec_points.cend()), ResultTupleCov(0),
            [&](const tbb::blocked_range<points_iterator>& r, ResultTupleCov res) -> ResultTupleCov {
                auto& [vec_source, vec_target] = res;

                for (auto it = r.begin(); it != r.end(); ++it) {
                    const auto& point = *it;
                    CovStruct closest_cov = GetClosestCovariance(point);

                    if ((closest_cov.mean - point.pose).squaredNorm() < d_max_dist_squared) {
                        vec_source.emplace_back(point);
                        vec_target.emplace_back(closest_cov);
                    }
                }

                return res;
            },
            [](ResultTupleCov a, const ResultTupleCov& b) -> ResultTupleCov {
                auto& [vec_source_a, vec_target_a] = a;
                const auto& [vec_source_b, vec_target_b] = b;

                vec_source_a.insert(vec_source_a.end(), std::make_move_iterator(vec_source_b.begin()),
                                    std::make_move_iterator(vec_source_b.end()));
                vec_target_a.insert(vec_target_a.end(), std::make_move_iterator(vec_target_b.begin()),
                                    std::make_move_iterator(vec_target_b.end()));

                return a;
            });

    auto& [vec_source, vec_target] = result;
    return std::make_tuple(std::move(vec_source), std::move(vec_target));
}

std::tuple<std::vector<PointStruct>, std::vector<CovStruct>> VoxelHashMap::GetCorrespondencesAllCov(
        const RadarPointVector& vec_points, double d_max_correspondence_dist) const {
    auto GetAllCovariances = [&](const PointStruct& point) {
        std::vector<Voxel> vec_voxels = GetAdjacentVoxels(point, 1);
        std::vector<CovStruct> vec_neighbors_cov;
        vec_neighbors_cov.reserve(vec_voxels.size());

        for (const auto& voxel : vec_voxels) {
            auto search = map_.find(voxel);
            if (search != map_.end() && !search->second.points.empty()) {
                vec_neighbors_cov.emplace_back(search->second.covariance);
            }
        }

        return vec_neighbors_cov;
    };

    using points_iterator = RadarPointVector::const_iterator;
    const double d_max_dist_squared = d_max_correspondence_dist * d_max_correspondence_dist;

    auto result = tbb::parallel_reduce(
            tbb::blocked_range<points_iterator>(vec_points.cbegin(), vec_points.cend()), ResultTupleCov(0),
            [&](const tbb::blocked_range<points_iterator>& r, ResultTupleCov res) -> ResultTupleCov {
                auto& [vec_source, vec_target] = res;

                for (auto it = r.begin(); it != r.end(); ++it) {
                    const auto& point = *it;
                    auto vec_neighbors_cov = GetAllCovariances(point);

                    for (const auto& cov : vec_neighbors_cov) {
                        if ((cov.mean - point.pose).squaredNorm() < d_max_dist_squared) {
                            vec_source.emplace_back(point);
                            vec_target.emplace_back(cov);
                        }
                    }
                }

                return res;
            },
            [](ResultTupleCov a, const ResultTupleCov& b) -> ResultTupleCov {
                auto& [vec_source_a, vec_target_a] = a;
                const auto& [vec_source_b, vec_target_b] = b;

                vec_source_a.insert(vec_source_a.end(), std::make_move_iterator(vec_source_b.begin()),
                                    std::make_move_iterator(vec_source_b.end()));
                vec_target_a.insert(vec_target_a.end(), std::make_move_iterator(vec_target_b.begin()),
                                    std::make_move_iterator(vec_target_b.end()));

                return a;
            });

    auto& [vec_source, vec_target] = result;
    return std::make_tuple(std::move(vec_source), std::move(vec_target));
}

std::vector<VoxelHashMap::Voxel> VoxelHashMap::GetAdjacentVoxels(const PointStruct& point, int range) const {
    std::vector<VoxelHashMap::Voxel> voxels;

    VoxelHashMap::Voxel voxel = PointToVoxel(point.pose, voxel_size_);

    int voxel_x = voxel.x();
    int voxel_y = voxel.y();
    int voxel_z = voxel.z();

    int voxel_neighbor = 1;
    int voxel_search_num = 27; // 3x3x3 = 27

    if (range == 0) {
        return std::vector<VoxelHashMap::Voxel>{voxel};
    }
    else if (range == 1) {
        return std::vector<VoxelHashMap::Voxel>{VoxelHashMap::Voxel(voxel_x + 0, voxel_y + 0, voxel_z + 0),
                                                VoxelHashMap::Voxel(voxel_x + 1, voxel_y + 0, voxel_z + 0),
                                                VoxelHashMap::Voxel(voxel_x - 1, voxel_y + 0, voxel_z + 0),
                                                VoxelHashMap::Voxel(voxel_x + 0, voxel_y + 1, voxel_z + 0),
                                                VoxelHashMap::Voxel(voxel_x + 0, voxel_y - 1, voxel_z + 0),
                                                VoxelHashMap::Voxel(voxel_x + 0, voxel_y + 0, voxel_z + 1),
                                                VoxelHashMap::Voxel(voxel_x + 0, voxel_y + 0, voxel_z - 1)};
    }
    else {
        voxels.reserve(voxel_search_num);
        for (int i = voxel.x() - voxel_neighbor; i < voxel.x() + voxel_neighbor + 1; ++i) {
            for (int j = voxel.y() - voxel_neighbor; j < voxel.y() + voxel_neighbor + 1; ++j) {
                for (int k = voxel.z() - voxel_neighbor; k < voxel.z() + voxel_neighbor + 1; ++k) {
                    voxels.emplace_back(i, j, k);
                }
            }
        }
        return voxels;
    }
}

std::vector<PointStruct> VoxelHashMap::Pointcloud() const {
    std::vector<PointStruct> points;
    points.reserve(max_points_per_voxel_ * map_.size()); // max points per voxel * total voxels
    for (const auto& [voxel, voxel_block] : map_) {      // iterate over each voxel
        (void)voxel;                                     // unused
        for (const auto& point : voxel_block.points) {   // iterate over points in voxel
            points.push_back(point);
        }
    }
    return points;
}

std::vector<CovStruct> VoxelHashMap::Covariances() const {
    std::vector<CovStruct> covariances;
    covariances.reserve(map_.size()); // max points per voxel * total voxels
    for (const auto& [voxel, voxel_block] : map_) { // iterate over each voxel
        (void)voxel;                                // unused
        if (voxel_block.points.size() > 2) covariances.push_back(voxel_block.covariance);
    }
    return covariances;
}

// update points in global frame
void VoxelHashMap::Update(const RadarPointVector& points, const Eigen::Vector3d& origin) { AddPoints(points); }

void VoxelHashMap::AddPoints(const RadarPointVector& points) {
    if (points.size() == 0) return;
    const double map_resolution = std::sqrt(voxel_size_ * voxel_size_ / max_points_per_voxel_);

    std::for_each(points.cbegin(), points.cend(), [&](const auto& point) { // point = PointStruct
        auto voxel = Voxel((point.pose / voxel_size_).template cast<int>());
        auto search = map_.find(voxel);
        if (search != map_.end()) {
            auto& voxel_block = search->second;
            voxel_block.AddPointWithSpacing(point, max_points_per_voxel_);
        }
        else {
            map_.insert({voxel, VoxelBlock{{point}, CovStruct(), max_points_per_voxel_, map_resolution}});
        }
    });
}
