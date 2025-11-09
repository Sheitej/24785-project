// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// NOTE: This implementation is heavily inspired in the original CT-ICP VoxelHashMap implementation,
// although it was heavily modifed and drastically simplified, but if you are using this module you
// should at least acknoowledge the work from CT-ICP by giving a star on GitHub
#pragma once

#include <tsl/robin_map.h>

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>
#include <queue>

// #include "VoxelUtils.hpp"
#include "lo_dev/VoxelMap/VoxelUtils.hpp"

// namespace kiss_icp {
namespace lo_dev {
struct VoxelHashMap 
{
    VoxelHashMap(){}; // added to default kiss-icp voxel map
    explicit VoxelHashMap(double voxel_size, double max_distance, unsigned int max_points_per_voxel)
        : voxel_size_(voxel_size),
          max_distance_(max_distance),
          max_points_per_voxel_(max_points_per_voxel) {}

    inline void Clear() { map_.clear(); }
    inline unsigned int Size() { return static_cast<unsigned int>(map_.size()); } // added to default kiss-icp voxel map
    int SizePoints(); // added to default kiss-icp voxel map
    inline bool Empty() const { return map_.empty(); }
    void Initialize(double voxel_size, double max_distance, unsigned int max_points_per_voxel); // added to default kiss-icp voxel map
    void Update(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &origin);
    void Update(const std::vector<Eigen::Vector3d> &points, const Sophus::SE3d &pose);
    void AddPoints(const std::vector<Eigen::Vector3d> &points);
    void RemovePointsFarFromLocation(const Eigen::Vector3d &origin);
    std::vector<Eigen::Vector3d> Pointcloud() const;
    std::tuple<Eigen::Vector3d, double> GetClosestNeighbor(const Eigen::Vector3d &query) const;
    void GetClosestNNeighbors(const Eigen::Vector3d &query, unsigned int num_neighbors, std::vector<Eigen::Vector3d>& out_pts, std::vector<float>& out_dists, double search_radius);

    void CheckVoxelIds();
    void DumpVoxelCorrdinates(double t);

    double voxel_size_;
    double max_distance_;
    unsigned int max_points_per_voxel_;
    tsl::robin_map<Voxel, std::vector<Eigen::Vector3d>> map_;   // 3rd party hash map

    int num_points_ = 0;

    const std::string knn_search_strategy_ = "max_heap"; // "min_heap" or "nth_element", for small k (like 5) -> min_heap would be better
    // const std::string knn_search_strategy_ = "nth_element"; // "min_heap" or "nth_element", for small k (like 5) -> min_heap would be better

    // ref: CT-ICP map.h -> move this to local in GetClosestNNeighbors() for threading
    // using dist_pos = std::pair<double, Eigen::Vector3d>;
    // struct FartherFirst {
    //     bool operator()(const dist_pos& a, const dist_pos& b) const {
    //         return a.first < b.first;
    //     }
    // };
    // struct CloserFirst {
    //     bool operator()(const dist_pos& a, const dist_pos& b) const {
    //         return a.first > b.first;
    //     }
    // };
    // std::priority_queue<dist_pos, std::vector<dist_pos>, FartherFirst> max_heap_;

};
}   // lo_dev
// }  // namespace kiss_icp
