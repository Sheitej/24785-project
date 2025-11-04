// #include "VoxelUtils.hpp"
#include "lo_dev/VoxelMap/VoxelUtils.hpp"

#include <tsl/robin_map.h>

// namespace kiss_icp {
namespace lo_dev {

// This function voxel-downsample points such that only a point lies in a voxel_size space
std::vector<Eigen::Vector3d> VoxelDownsample(const std::vector<Eigen::Vector3d> &frame,
                                             const double voxel_size) 
{
    // create a hash map
    tsl::robin_map<Voxel, Eigen::Vector3d> grid;
    grid.reserve(frame.size());

    // loop over all the points in input frame, convert them to Voxel index and insert it to the hash map
    std::for_each(frame.cbegin(), frame.cend(), [&](const auto &point) {
        // convert point to a voxed id
        const auto voxel = PointToVoxel(point, voxel_size);
        // if grid does not contain the voxel id yet, add the voxel and point
        if (!grid.contains(voxel)) grid.insert({voxel, point});
        // so, basically a point per voxel
    });

    // prepare downsampled container
    std::vector<Eigen::Vector3d> frame_dowsampled;
    frame_dowsampled.reserve(grid.size());
    // add voxel-downsampled point grid
    std::for_each(grid.cbegin(), grid.cend(), [&](const auto &voxel_and_point) {
        frame_dowsampled.emplace_back(voxel_and_point.second);
    });
    return frame_dowsampled;
}

// // pcl::PointCloud ver. of KissICP::VoxelDownsample() (KissICP::VoxelDownsample() uses std::vector<Eigen::Vector3d> as point cloud data)
// std::vector<Eigen::Vector3d> VoxelDownsample(const std::vector<Eigen::Vector3d> &frame, const double voxel_size) 
// {
//     // create a hash map
//     tsl::robin_map<Voxel, Eigen::Vector3d> grid;
//     grid.reserve(frame.size());

//     // loop over all the points in input frame, convert them to Voxel index and insert it to the hash map
//     std::for_each(frame.cbegin(), frame.cend(), [&](const auto &point) {
//         // convert point to a voxed id
//         const auto voxel = PointToVoxel(point, voxel_size);
//         // if grid does not contain the voxel id yet, add the voxel and point
//         if (!grid.contains(voxel)) grid.insert({voxel, point});
//         // so, basically a point per voxel
//     });

//     // prepare downsampled container
//     std::vector<Eigen::Vector3d> frame_dowsampled;
//     frame_dowsampled.reserve(grid.size());
//     // add voxel-downsampled point grid
//     std::for_each(grid.cbegin(), grid.cend(), [&](const auto &voxel_and_point) {
//         frame_dowsampled.emplace_back(voxel_and_point.second);
//     });
//     return frame_dowsampled;
// }

}  // namespace lo_dev
// }  // namespace kiss_icp
