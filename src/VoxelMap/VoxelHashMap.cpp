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
// #include "VoxelHashMap.hpp"
#include "lo_dev/VoxelMap/VoxelHashMap.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <sophus/se3.hpp>
#include <vector>
#include <string>
#include <fstream>
// #include <ofstream>
// #include <iomanip>

namespace {
// using kiss_icp::Voxel;
using lo_dev::Voxel;
static const std::array<Voxel, 27> voxel_shifts{
    {Voxel{0, 0, 0},   Voxel{1, 0, 0},   Voxel{-1, 0, 0},  Voxel{0, 1, 0},   Voxel{0, -1, 0},
     Voxel{0, 0, 1},   Voxel{0, 0, -1},  Voxel{1, 1, 0},   Voxel{1, -1, 0},  Voxel{-1, 1, 0},
     Voxel{-1, -1, 0}, Voxel{1, 0, 1},   Voxel{1, 0, -1},  Voxel{-1, 0, 1},  Voxel{-1, 0, -1},
     Voxel{0, 1, 1},   Voxel{0, 1, -1},  Voxel{0, -1, 1},  Voxel{0, -1, -1}, Voxel{1, 1, 1},
     Voxel{1, 1, -1},  Voxel{1, -1, 1},  Voxel{1, -1, -1}, Voxel{-1, 1, 1},  Voxel{-1, 1, -1},
     Voxel{-1, -1, 1}, Voxel{-1, -1, -1}}};
}  // namespace

namespace lo_dev {

// this function returns a closest neighbor point and distance via NN search
std::tuple<Eigen::Vector3d, double> VoxelHashMap::GetClosestNeighbor(
    const Eigen::Vector3d &query) const 
{
    // Convert the point to voxel coordinates
    const auto &voxel = PointToVoxel(query, voxel_size_); // Voxel = Eigen::Vector3i voxel
    // Find the nearest neighbor
    Eigen::Vector3d closest_neighbor = Eigen::Vector3d::Zero();
    double closest_distance = std::numeric_limits<double>::max();

    // std::for_each performs an operation for each element in voxel_shifts
    std::for_each(voxel_shifts.cbegin(), voxel_shifts.cend(), [&](const auto &voxel_shift) {
        // get the neighborhood voxel index
        const auto &query_voxel = voxel + voxel_shift;
        // find points in the voxel using hash map (probably map _.find finds)
        auto search = map_.find(query_voxel);   // map_ is a hash map: tsl::robin_map<Voxel, std::vector<Eigen::Vector3d>> map_
        if (search != map_.end()) 
        {
            const auto &points = search.value(); // value() returns the mapped value (std::vector<Eigen::Vector3d>)
            
            // nearest neighbor search of const Eigen::Vector3d &query
            const Eigen::Vector3d &neighbor = *std::min_element(
                points.cbegin(), points.cend(), [&](const auto &lhs, const auto &rhs) {
                    return (lhs - query).norm() < (rhs - query).norm();
                }); // the element with the smallest distance to query is returned (as an iterator) from std::min_element()
                // This can be (lhs - query).squaredNorm() avoiding square-root
            double distance = (neighbor - query).norm();

            // update closest neighbor for the input query
            // (since we perform the NN search on all the possible 27 neighbor voxels)
            if (distance < closest_distance) {
                closest_neighbor = neighbor;
                closest_distance = distance;
            }
        }
    });
    return std::make_tuple(closest_neighbor, closest_distance);
}


// [Added based on kiss-icp]: this function returns a closest neighbor point and distance via NN search
void VoxelHashMap::GetClosestNNeighbors(
    const Eigen::Vector3d &query, unsigned int num_neighbors, 
    std::vector<Eigen::Vector3d>& out_pts, std::vector<float>& out_dists, double search_radius) 
{
    out_pts.clear(); 
    out_dists.clear();
    if (num_neighbors == 0) return;

    using dist_pos = std::pair<double, Eigen::Vector3d>;
    auto farther = [](const dist_pos& a, const dist_pos& b){ return a.first < b.first; };
    std::priority_queue<dist_pos, std::vector<dist_pos>, decltype(farther)> max_heap(farther);

    const double sqrd_search_radius = search_radius * search_radius;
    // std::cout << "sqrd_search_radius = " << sqrd_search_radius << std::endl;

    // Convert the point to voxel coordinates
    const auto &voxel = PointToVoxel(query, voxel_size_); // Voxel = Eigen::Vector3i voxel
 
    if (knn_search_strategy_ == "max_heap")
    {
        for (const auto& voxel_shift : voxel_shifts)
        {
            // get the neighborhood voxel index
            const auto &query_voxel = voxel + voxel_shift;

            // for debugging
            // std::cout << "                       NN Voxel:" << "(x,y,z) = (" << query_voxel.x() << ", " << query_voxel.y() << ", " << query_voxel.z() << ")" << std::endl;

            // find points in the voxel using hash map (probably map _.find finds)
            auto search = map_.find(query_voxel);   // map_ is a hash map: tsl::robin_map<Voxel, std::vector<Eigen::Vector3d>> map_
            if (search != map_.end())
            {
                const auto &points = search.value(); // value() returns the mapped value (std::vector<Eigen::Vector3d>)
                // loop over all the points in the voxel 
                for (const auto &pt: points)
                {
                    double sqrd_distance = (pt - query).squaredNorm();
                    // std::cout << "sqrd_search_radius = " << sqrd_search_radius << std::endl;
                    // std::cout << "sqrd_distance = " << sqrd_distance << std::endl;
                    // std::cout << "(sqrd_search_radius < sqrd_distance) = " << (sqrd_search_radius < sqrd_distance) << std::endl;

                    // you can put a radius search distance threshold
                    if (sqrd_search_radius < sqrd_distance) continue;

                    if(max_heap.size() < num_neighbors)
                    {
                        max_heap.emplace(sqrd_distance, pt);
                    }
                    else if(sqrd_distance < max_heap.top().first)
                    {
                        max_heap.pop();
                        max_heap.emplace(sqrd_distance, pt);
                    }
                }
            }
        }

        if(max_heap.empty())
        {
            // std::cout << __FUNCTION__ << __LINE__ << std::endl;
            // std::cout << "max_heap.size() is zero. Exit the function." << std::endl;
            return;
        }

        // now that max_heap has the 5 smallest distance neighbors in a decending order (e.g., {5,4,3,2,1}, 'top'=5)
        // store then in the output container
        size_t heap_size = static_cast<size_t>(max_heap.size());
        out_pts.reserve(heap_size); 
        out_dists.reserve(heap_size);
        while(!max_heap.empty())
        {
            out_pts.emplace_back(max_heap.top().second);
            out_dists.emplace_back(static_cast<float>(std::sqrt(max_heap.top().first)));
            max_heap.pop();
        }
        // We have to sort out the elements in the output container in acending order
        std::reverse(out_pts.begin(), out_pts.end());
        std::reverse(out_dists.begin(), out_dists.end());
    }
    else if (knn_search_strategy_ == "nth_element")
    { 
        // Create a neighborhood container
        std::vector<Eigen::Vector3d> cand;

        // for debugging
        // std::cout << "                    Query Voxel:" << "(x,y,z) = (" << voxel.x() << ", " << voxel.y() << ", " << voxel.z() << ")" << std::endl;

        // Gather candidates from neighbor voxels
        for (const auto& voxel_shift : voxel_shifts)
        {
            // get the neighborhood voxel index
            const auto &query_voxel = voxel + voxel_shift;

            // for debugging
            // std::cout << "                       NN Voxel:" << "(x,y,z) = (" << query_voxel.x() << ", " << query_voxel.y() << ", " << query_voxel.z() << ")" << std::endl;

            // find points in the voxel using hash map (probably map _.find finds)
            auto search = map_.find(query_voxel);   // map_ is a hash map: tsl::robin_map<Voxel, std::vector<Eigen::Vector3d>> map_
            if (search != map_.end())
            {
                const auto &points = search.value(); // value() returns the mapped value (std::vector<Eigen::Vector3d>)
                cand.reserve(cand.size() + points.size()); // update the size
                for (const auto& pt: points)
                {
                    cand.emplace_back(pt);
                }
            }
        }

        if (cand.empty())
        { 
            // for debugging
            // std::cout << "NN has not been found for " << "(x,y,z) = (" << query.x() << ", " << query.y() << ", " << query.z() << ")" << std::endl;
            // std::cout << "                    Voxel:" << "(x,y,z) = (" << voxel.x() << ", " << voxel.y() << ", " << voxel.z() << ")" << std::endl;
            return;
        }
        else
        {
            // for debugging
            // std::cout << "NN has been found for     " << "(x,y,z) = (" << query.x() << ", " << query.y() << ", " << query.z() << ")" << std::endl;
            // std::cout << "                    Voxel:" << "(x,y,z) = (" << voxel.x() << ", " << voxel.y() << ", " << voxel.z() << ")" << std::endl;

            // for debugging
            // std::cout << "num of nn candidate in all the neighboring voxel : " << cand.size() << std::endl;
        }

        // Comparator by squared distance to 'query' (no sqrt)
        const auto cmp = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b){
            return (a - query).squaredNorm() < (b - query).squaredNorm();
        };

        // const std::size_t m = std::min(num_neighbors, cand.size());
        const unsigned int m = std::min(num_neighbors, static_cast<unsigned int>(cand.size())); // might cause an issue that static_cast<unsigned int>(cand.size()) loops backs to small values
        if (m == 0) return;  

        std::nth_element(cand.begin(), cand.begin()+m, cand.end(), cmp); // first m are the nearest k (unsorted)
        std::sort(cand.begin(), cand.begin()+m, cmp);                    // (optional) sort those m

        out_pts.clear(); out_dists.clear();
        out_pts.reserve(m); out_dists.reserve(m);
        for (std::size_t i = 0; i < m; ++i) 
        {
            const auto& p = cand[i];
            out_pts.push_back(p);
            out_dists.push_back(static_cast<float>((p - query).norm())); // or sqrt of squaredNorm
        }
    }

    // for debugging 
    // std::cout << "        " << "   query: " << "(x,y,z) = (" << query.x() << ", " << query.y() << ", " << query.z() << ")" << std::endl;
    // for (auto pt: out_pts)
    // {
    //     std::cout << "        " << "       nns: " << "(x,y,z) = (" << pt.x() << ", " << pt.y() << ", " << pt.z() << ")" << std::endl;
    // }

}

// this function returns all the points in the voxel map as a vector
std::vector<Eigen::Vector3d> VoxelHashMap::Pointcloud() const 
{
    // define point cloud container
    std::vector<Eigen::Vector3d> points;

    // allocate the necessary memory
    points.reserve(map_.size() * static_cast<size_t>(max_points_per_voxel_));

    // loop over all the elements in the hash map (map_)
    std::for_each(map_.cbegin(), map_.cend(), [&](const auto &map_element) {
        // get points (std::vector<Eigen::Vector3d>>)
        const auto &voxel_points = map_element.second;
        // store them in std::vector<Eigen::Vector3d> points
        points.insert(points.end(), voxel_points.cbegin(), voxel_points.cend());
    });

    // adjust the memory space to the size of the vector
    points.shrink_to_fit();
    return points;
}

// This function just wraps up AddPoints() and RemovePointsFarFromLocation()
void VoxelHashMap::Update(const std::vector<Eigen::Vector3d> &points,
                          const Eigen::Vector3d &origin) 
{
    AddPoints(points);
    RemovePointsFarFromLocation(origin); // commented out for debugging
}

// This function wraps up point transformation given a pose and AddPoints() and RemovePointsFarFromLocation()
void VoxelHashMap::Update(const std::vector<Eigen::Vector3d> &points, const Sophus::SE3d &pose) 
{
    std::vector<Eigen::Vector3d> points_transformed(points.size());
    std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
                   [&](const auto &point) { return pose * point; });
    const Eigen::Vector3d &origin = pose.translation();
    Update(points_transformed, origin);
}

// this function add points to existing voxel or a new voxel
void VoxelHashMap::AddPoints(const std::vector<Eigen::Vector3d> &points) 
{
    const double map_resolution = std::sqrt(voxel_size_ * voxel_size_ / max_points_per_voxel_);
    
    // loop over all the points in the input std::vector<Eigen::Vector3d> &points
    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
        // Convert the point to voxel coordinate
        const auto voxel = PointToVoxel(point, voxel_size_);
        // get an iterator to the voxel
        auto search = map_.find(voxel);
        if (search != map_.end()) {
            // get the mapped value std::vector<Eigen::Vector3d>
            auto &voxel_points = search.value(); // We can modify this voxel_points because it's not const

            // check if distance between any point in the voxel and an input point processed currently is smaller than the map resolution  
            if (voxel_points.size() == max_points_per_voxel_ ||
                std::any_of(voxel_points.cbegin(), voxel_points.cend(),
                            [&](const auto &voxel_point) {
                                return (voxel_point - point).norm() < map_resolution;
                            })) {
                return;
            }
            // commented out for debugging 

            // if yes, the processed point falls into an existing voxel
            // so add a point to the voxel point
            voxel_points.emplace_back(point);
        } else {
            // if not, we need to allocate a new voxel
            std::vector<Eigen::Vector3d> voxel_points;
            voxel_points.reserve(max_points_per_voxel_);
            voxel_points.emplace_back(point);
            map_.insert({voxel, std::move(voxel_points)});
        }
    });
}

// This function erase voxel and points that are far from the input origin
void VoxelHashMap::RemovePointsFarFromLocation(const Eigen::Vector3d &origin) 
{
    const auto max_distance2 = max_distance_ * max_distance_;
    // loop over all the map elements
    for (auto it = map_.begin(); it != map_.end();) 
    {
        // convert it to Voxel voxel and  std::vector<Eigen::Vector3d> voxel_points
        const auto &[voxel, voxel_points] = *it;

        // get the oldest point from voxel points 
        const auto &pt = voxel_points.front();
        // If the distance from the input origin exceeds max_distance_^2, erase the point and voxel from the map
        if ((pt - origin).squaredNorm() >= (max_distance2)) {
            it = map_.erase(it);
        } else {
            ++it;
        }
    }
}

void VoxelHashMap::Initialize(double voxel_size, double voxel_max_distance, unsigned int max_points_per_voxel)
{
    voxel_size_ = voxel_size;
    max_distance_ = voxel_max_distance;
    max_points_per_voxel_ = max_points_per_voxel;
}

void VoxelHashMap::CheckVoxelIds()
{
    size_t cnt = 0;
    for (auto it = map_.begin(); it != map_.end(); ++it) 
    {
        std::cout << "Voxel: (" << it->first.x() << ", " << it->first.y() << ", " << it->first.z() << ")" << std::endl;
        std::cout << "        " << it->second.size() << " points" << std::endl;
         
        for (auto pt: it->second)
        {
            std::cout << "        " << "(x,y,z) = (" << pt.x() << ", " << pt.y() << ", " << pt.z() << ")" << std::endl;
        }

        cnt++;
        if(cnt>10) break;
    }
}

int VoxelHashMap::SizePoints()
{
    int numPt = 0;
    
    for (auto it = map_.begin(); it != map_.end(); ++it) 
    {
        numPt += it->second.size();
        // std::cout << "Voxel: (" << it->first.x() << ", " << it->first.y() << ", " << it->first.z() << ")" << std::endl;
        // std::cout << "        " << it->second.size() << " points" << std::endl;
         
        // for (auto pt: it->second)
        // {
        //     std::cout << "        " << "(x,y,z) = (" << pt.x() << ", " << pt.y() << ", " << pt.z() << ")" << std::endl;
        // }

        // cnt++;
        // if(cnt>10) break;
    }

    std::cout << "        total points in voxelmap: " << numPt << std::endl;
}

void VoxelHashMap::DumpVoxelCorrdinates(double t)
{
    std::string filePath;
    std::string fileName;

    filePath = "/sandbox/ysugano/ros2_ws/src/livo_dev/lo_dev/Log/voxel_check_2";

    fileName = filePath + "/" + std::to_string(t) + ".txt";
    std::ofstream ofs(fileName, std::ios::out);
    if (!ofs.is_open())
    {
        std::cout << "Failed to open file: " << fileName << std::endl;
        // RCLCPP_INFO_STREAM(logger_, "Failed to open file: " << fileName);
        return;
    }
    else
    {
        std::cout << "Dump Time Records into file: " << fileName << std::endl;
        // RCLCPP_INFO_STREAM(logger_, "Dump Time Records into file: " << fileName);
    }

    std::vector<std::pair<decltype(map_)::key_type, decltype(map_)::mapped_type>> items;
    items.reserve(map_.size());
    for (const auto& kv : map_) items.push_back(kv);

    // std::sort(items.begin(), items.end(),
    //         [](const auto& a, const auto& b) {
    //             // lexicographic by x, then y, then z
    //             return std::tie(a.first.x(), a.first.y(), a.first.z())
    //                 < std::tie(b.first.x(), b.first.y(), b.first.z());
    //         });

    std::sort(items.begin(), items.end(),
            [](const auto& a, const auto& b) {
                const auto& A = a.first;
                const auto& B = b.first;
                if (A.x() != B.x()) return A.x() > B.x();
                if (A.y() != B.y()) return A.y() > B.y();
                if (A.z() != B.z()) return A.z() > B.z();
                return false; // keep original order if equal
            });
            
    // for (const auto &iter : map_)
    for (const auto &iter : items)
    {
        // ofs.setf(std::ios::fixed);
        // ofs << std::setprecision(std::numeric_limits<double>::max_digits10);
        ofs << iter.first.x() << ", " << iter.first.y() << ", " << iter.first.z() << " : " << iter.second.size() << "pts" << std::endl;
    }
    ofs.close();
}

}  // namespace lo_dev
// }  // namespace kiss_icp
