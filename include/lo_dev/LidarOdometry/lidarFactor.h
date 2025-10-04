#ifndef LIDARFACTOR_H
#define LIDARFACTOR_H

#include <cmath>
#include <iostream>
#include <queue>
#include <string>
#include <vector>
#include <iomanip>
#include <mutex>
#include <thread>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/opencv.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "rclcpp/rclcpp.hpp"
// #include "rclcpp/time_source.hpp"  
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h> 
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/transform_datatypes.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include "lo_dev/parameter/parameter.h"
#include <std_msgs/msg/string.hpp>
#include "lo_dev/utils/common.h"


namespace lo_dev 
{

// copied from LidarKeyframeFactor in Liliom, which is used in ceres icp
// This struct defines the residual and jacobian to be used in iterative non-linear optimization
struct LidarPlaneNormIncreFactor 
{
    static constexpr int NumResiduals() { return 1; } // added referring to ct-lio

    // Constructor
    LidarPlaneNormIncreFactor(Eigen::Vector3d curr_point_,
                              Eigen::Vector3d plane_unit_norm_,
                              double negative_OA_dot_norm_): 
        curr_point(curr_point_),
        plane_unit_norm(plane_unit_norm_),
        negative_OA_dot_norm(negative_OA_dot_norm_) {}

    // operator() (= a functor) definition -> use ceres::AutomaticDiff
    template <typename T> bool operator()(const T *q, const T *t, T *residual) const {
        Eigen::Quaternion<T> q_inc{q[0], q[1], q[2], q[3]}; // increment rot
        // q_inc.normalize(); // added just in case // didn't do any good
        Eigen::Matrix<T, 3, 1> t_inc{t[0], t[1], t[2]};     // increment trans
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())}; // current point in the body frame
        Eigen::Matrix<T, 3, 1> point_w;     // point in the world (map)
        point_w = q_inc * cp + t_inc;       // Is this like transformation of point into the world coordinate already encoded in the cost function??

        Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));    // plane normal

        // Common point-to-plane residual
        residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm); // residual=(plane normal)*(point position vector in world)+(distance from the plane to the world origin)

        // should be RCLCPP
        // std::cout << __FUNCTION__ << __LINE__ << std::endl;
        // std::cout << "q_inc: " << q_inc.coeffs().transpose() << std::endl;
        // std::cout << "t_inc: " << t_inc.transpose() << std::endl;
        // std::cout << "cp: " << cp << std::endl;
        // std::cout << "q_inc: " << q_inc << std::endl;
        // std::cout << "t_inc: " << t_inc << std::endl;
        // std::cout << "point_w: " << point_w << std::endl;

        return true;
    }

    // Function to create a ceres cost function
    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                       const Eigen::Vector3d plane_unit_norm_,
                                       const double negative_OA_dot_norm_) {
        return (new ceres::AutoDiffCostFunction<LidarPlaneNormIncreFactor, 1, 4, 3>(new LidarPlaneNormIncreFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_)));
    }
    // Example of ceres::AutoDiffCostFunction - http://ceres-solver.org/nnls_modeling.html#autodiffcostfunction
    // MyScalarCostFunctor functor(1.0)
    // auto* cost_function = new AutoDiffCostFunction<MyScalarCostFunctor, 1, 2, 2>(&functor, DO_NOT_TAKE_OWNERSHIP);
    // To get an auto differentiated cost function, you must define a class with a templated operator() (a functor) that computes the cost function in terms of the template parameter T.
    // like:
        // class MyScalarCostFunctor {
        // MyScalarCostFunctor(double k): k_(k) {}

        // template <typename T>
        // bool operator()(const T* const x , const T* const y, T* e) const {
        //     e[0] = k_ - x[0] * y[0] - x[1] * y[1];
        //     return true;
        // }

        // private:
        // double k_;
        // };
    // functor: bool operator() should take (const T* const x , const T* const y, T* e) where e is the residual

    Eigen::Vector3d curr_point;
    Eigen::Vector3d plane_unit_norm;
    double negative_OA_dot_norm;
};


} // namespace lo_dev

#endif // LIDARFACTOR_H
