#ifndef COMMON_H
#define COMMON_H

// [20250915] Copied from common.h in liliom

#include <sensor_msgs/msg/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/pcl_macros.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/NavState.h>

#include <Eigen/Dense>
#include <Eigen/Core>


struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)


struct OusterPointXYZIRT 
{
    PCL_ADD_POINT4D
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, ambient, ambient) (uint32_t, range, range)
)

typedef pcl::PointXYZINormal PointType;
// use PointXYZINormal format as the default PointType
// If you use OusterPointXYZIRT or VelodynePointXYZIRT as PointType, it gives you an error
// We need to store time [sec] in PointXYZINormal._curvature

// uint16_t ambient could be uint16_t noise
// uint16_t ambient: automine ouster lidar, fast-lio2 ouster lidar
// uint16_t noise: lio-sam ouster lidar

// If not inline, there was multiple definition error
inline void getEigenFromGtsam(const gtsam::NavState& stateIn, Eigen::Quaterniond& qOut, Eigen::Vector3d& tOut)
{
    const gtsam::Rot3 rotIn = stateIn.attitude();
    const gtsam::Quaternion quatGIn = rotIn.toQuaternion().normalized();
    Eigen::Quaterniond quatEIn(quatGIn.w(),quatGIn.x(),quatGIn.y(),quatGIn.z());
    quatEIn.normalize();

    const gtsam::Point3 tranIn = stateIn.position();
    Eigen::Vector3d tranEIn(tranIn.x(),tranIn.y(),tranIn.z());

    qOut = quatEIn;
    tOut = tranEIn;
}

inline void getGtsamFromEigen(const Eigen::Quaterniond& qIn, const Eigen::Vector3d& tIn, gtsam::NavState& stateOut)
{
    const gtsam::Rot3 rotIn = gtsam::Rot3(qIn);
    const gtsam::Point3 ptIn = gtsam::Point3(tIn);
    const gtsam::Velocity3 velIn = gtsam::Velocity3();

    stateOut = gtsam::NavState(rotIn, ptIn, velIn);
}

// from math_tool.h in liliom
template <typename T>
Eigen::Quaternion<T> unifyQuaternion(const Eigen::Quaternion<T> &q)
{
    if(q.w() >= 0) return q;
    else {
        Eigen::Quaternion<T> resultQ(-q.w(), -q.x(), -q.y(), -q.z());
        return resultQ;
    }
}



#endif // COMMON_H