#ifndef PARAMETER_H
#define PARAMETER_H

#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
// #include <opencv2/core/eigen.hpp>
// #include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

enum class SensorType 
{
    VELODYNE, 
    OUSTER, 
    LIVOX
};

extern std::string IMU_TOPIC;
extern std::string LIDAR_TOPIC;
extern std::string ODOM_TOPIC;
extern std::string ProjectName;

extern std::string MAP_FRAME;
extern std::string ODOM_FRAME;
extern std::string BASE_FRAME;
extern std::string IMU_FRAME;
extern std::string LIDAR_FRAME;

// Always identity, but explicitly declared for clarity
extern Eigen::Matrix3d R_MAP_ODOM;
extern Eigen::Vector3d t_MAP_ODOM;
extern Eigen::Quaterniond q_MAP_ODOM;

// Always identity, but explicitly declared for clarity
extern Eigen::Matrix3d R_BASE_IMU;
extern Eigen::Vector3d t_BASE_IMU;
extern Eigen::Quaterniond q_BASE_IMU;

extern Eigen::Matrix3d R_IMU_LIDAR;
extern Eigen::Vector3d t_IMU_LIDAR;
extern Eigen::Quaterniond q_IMU_LIDAR;

extern float IMU_ACC_X_LIMIT;
extern float IMU_ACC_Y_LIMIT;
extern float IMU_ACC_Z_LIMIT;

extern std::string SENSOR; 
extern SensorType sensor;


// for debugging 
extern bool debug_print_imu_forward_propagation_state;
extern bool debug_print_get_imu_pose_at_measurement_time;
extern bool debug_print_imu_pose_timeline;
extern bool debug_print_deskew_pointcloud;
extern bool debug_print_initial_guess_lo;
extern bool debug_print_relative_transform_lo;
extern bool debug_print_num_residuals;
extern bool debug_print_point_plane_residual_preparation;
extern bool debug_print_scan_imu_time_sync;
extern bool debug_print_first_point_in_current_scan;
extern bool debug_print_cloud_map_size;
extern bool debug_print_keyframe_id;
extern bool debug_print_lo_relative_pose_fg;
extern bool debug_print_PreintegratedImuMeasurements_fg;
extern bool debug_print_imu_factor_fg;
extern bool debug_print_imu_bias_factor_fg;
extern bool debug_print_lo_factor_fg;
extern bool debug_print_imu_prop_state_fg;
extern bool debug_print_estimated_state_fg;
extern bool debug_print_published_pose;
extern bool debug_print_key_timestamp_in_window;
extern bool debug_print_num_factors_values;
extern bool debug_print_num_downsampled_point;
extern bool debug_print_num_point_in_voxel_map;
extern bool debug_dump_log_file_voxel_ids;
extern bool debug_print_qp_active_set;
extern bool debug_print_qp_active_set_init_guess;
extern bool debug_print_qp_active_set_matrices;
extern bool debug_print_qp_active_set_matrices_jacobian_residual;
extern bool debug_print_qp_active_set_solution_analysis;
extern bool debug_print_qp_active_set_solution;
extern bool debug_print_qp_active_set_solution_pose_update;
extern bool debug_print_qp_icp_iter_num;
extern bool debug_print_qp_sqp_iter_num;
extern bool debug_try_qp_active_set_Cholesky_Decomposition_unconstrained;


// for initialization parameter
extern bool init_imu_init_fastlio2;

// for imu handling parameter
extern bool imu_isotropic_scale_calibration_to_G;
extern bool imu_enalbe_init_guess_for_lo;

bool readGlobalparam(rclcpp::Node::SharedPtr);
bool readCalibration(rclcpp::Node::SharedPtr);

#endif // PARAMETER_H



