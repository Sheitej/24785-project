#ifndef FRONTEND_H
#define FRONTEND_H

#include <cmath>
#include <string>
#include <vector>
#include <sophus/so3.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/time_source.hpp" 
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include "lo_dev/parameter/parameter.h"

#include <mutex>

#include "lo_dev/utils/common.h"
#include "lo_dev/LidarOdometry/lidarFactor.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/navigation/InvariantEKF.h>

#include <tsl/robin_map.h>
#include "lo_dev/VoxelMap/VoxelHashMap.hpp"

#include <omp.h>

// qpmad InEqualityConstrained Opt. Module
#include <qpmad/solver.h>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <LBFGS.h>

#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <ifopt/variable_set.h>
#include <ifopt/problem.h>
#include <ifopt/solver.h>
#include <ifopt/ipopt_solver.h>


namespace lo_dev 
{

using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y) should be 'P'

struct Config_Frontend
{
    // ---- for general ---- 
    bool set_main_process_timer; // true: run() is called in timer callback, false: run() is called in pointcloud msg callback
    bool turn_off_pcl_conversion_error_printing;

    // ---- for voxel map ----
    bool use_voxel_map;
    double voxel_size;
    double voxel_max_distance;
    unsigned int max_points_per_voxel;
    bool turn_on_voxel_downsample;
    double voxel_downsample_resolution_raw_to_mapping;
    double voxel_downsample_resolution_mapping_to_icpsource;
    double threshold_voxel_nn_search_radius;

    // ---- for preprocessing ---- 
    int num_scans;
    double lidar_min_range;
    int filter_point_size;
    SensorType sensor;
    double imu_acc_x_limit;
    double imu_acc_y_limit;
    double imu_acc_z_limit;
    float imu_acc_noise;
    float imu_acc_bias_noise;
    float imu_gyr_noise;
    float imu_gyr_bias_noise;
    float imu_gravity;
    bool enable_min_range_filter;
    double lidar_scan_rate;
    std::string point_cloud_msg_timestamp;
    bool print_sync_status;

    // ---- for lidar odometry ---- 
    int max_iterations;
    double max_solver_time_in_seconds;
    double voxel_filter_size;
    int max_cloud_frame_for_local_map;
    int icp_iteration_num;
    bool use_liosam_gauss_newton;
    bool use_fastlio_point_plane_residual_param;
    bool build_local_map_from_all_global_map;
    std::string initial_guess_source;
    std::string motion_compensation_source;

    // ---- for factor graph----
    bool turn_on_factor_graph;
    float lidar_correction_noise;
    float smooth_factor;        // what's this?
    // bool  use_imu_roll_pitch;
    double fixed_lag; 
    bool use_lo_prior_factor_wo_between_factor; 
    std::string factor_graph_init_guess_source;
    std::string fixed_lag_smoother_batch_or_incremental;
    bool fix_lidar_odom_covariance;

    // ---- for inequality constraints ---- 
    bool turn_on_qp_ineq_constraints_active_set;
    bool turn_on_ineq_constraints;
    std::string ineq_constraints_type;
    int sqp_iteration_num_qp_active_set;
    bool turn_on_levenberg_marquardt_qp_active_set;
    double levenberg_marquardt_lambda_qp_active_set;
    bool turn_on_levenberg_marquardt_qp_active_set_marquardt_damping;
    bool turn_on_robust_kernel_qp_active_set;
    bool turn_on_mad_based_scaling_for_kernel_weights_qp_active_set;
    std::string robust_kernel_qp_active_set;
    bool turn_on_jacobian_column_scaling_qp_active_set;
    std::string pose_increment_multiplication;
    bool turn_on_Sophus_SE3_update;
    std::string qp_active_set_Hessian_computation;
    bool qp_compute_ineq_from_imu_cov;
    double qp_compute_ineq_from_imu_cov_sigma_coef;
    bool levenberg_marquardt_trust_region_lambda_adjustment;
    double levenberg_marquardt_trust_region_lambda_min;
    double levenberg_marquardt_trust_region_step_quality_threshold;

    bool bfgs_on_inequality_constraint;
    bool ifopt_on_inequality_constraint;
    std::string inequality_constraints_solver;

    double vehicle_kinematic_constraints_max_steering_angle;
    double vehicle_kinematic_constraints_min_steering_angle;
    double vehicle_kinematic_constraints_wheel_base;

    // ---- for ekf imu propagation ----
    bool ekf_imu_covariance_prop_on;
    bool ekf_use_const_prior_cov;
    double ekf_const_diagonal_elem_prior_cov;
};


class Frontend : public rclcpp::Node 
{
    // enum class Kernel { Huber, Cauchy, Tukey };
public:
    Frontend(const rclcpp::NodeOptions& options);
    bool readParameters();
    void initializeInterface();
    void run();
    void publishOdometry();
    void publishCloud();
    void publishTf();
    void publishStaticTf();
    void clearProcess();

    // ---- for preprocessing ---- 
    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr msgIn);
    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msgIn);
    void initializeImu();
    bool synchronizeMeasurements();
    void convertCloudMsgToPcl(const sensor_msgs::msg::PointCloud2& msgIn, pcl::PointCloud<PointType>::Ptr cloudOut);
    void propagateImu();
    void transformPointCloudInImuFrame();
    void getImuPoseAtPointMeasurementTime(double pointTime, Eigen::Quaterniond& qOut, Eigen::Vector3d& tOut);
    void makeInitialGuessLO();
    void deskewPointCloud();

    // ---- for lidar odometry ----
    bool initializeCloudMap();
    void setInitialPoseLO();
    void buildLocalMap();
    void downsampleCloud();
    void preparePointPlaneResidual();
    void solveLeastSquares_Ceres_LeftMultiplying();
    void solveLeastSquares_Ceres_RightMultiplying();
    void computeCovarianceLO();
    void transformPointToWorldFrame(const PointType&  pi, PointType& po);
    void updatePoseLO();
    void updateCloudMap();
    void transformPointCloudInWorldFrame();

    // ---- for lidar odometry (inequality constraints) ----
    void solveLeastSquares_InequalityConstraints_ActiveSet();
    void computeWeightsFromResiduals(Eigen::Ref<Eigen::VectorXd> r, Eigen::VectorXd& w, double s, std::string kernel, double c, bool mad_scale_estimation);
    void solveLeastSquares_InequalityConstraints_BFGS();
    void solveLeastSquares_InequalityConstraints_IFOPT();
    double computeResidualWithDeltaX_OriginalObjective(Eigen::VectorXd delta_x, Eigen::VectorXd weights);
    // double computeModelPredictedReduction(Eigen::VectorXd delta_x, Eigen::Matrix<double,6,6> ATWA, Eigen::Matrix<double,6,1> ATWb);
    double computeModelPredictedReduction(Eigen::Vector<double, 6> delta_x, Eigen::Matrix<double,6,6> ATWA, Eigen::Matrix<double,6,1> ATWb);

    // ---- for factor graph ---- 
    bool initializeFG();
    void resetSmoother();
    void performFixedLagSmoothing();
    void logRelativePoseEstimateEachSensor();
    void logFactorGraphState();

    void updateState();



private:
    // ---- for general ---- 
    Config_Frontend config_;
    std::atomic_bool isProcessing_; // for process flag avoiding racing in multi-thread
    rclcpp::TimerBase::SharedPtr mainProcessTimer;  // added to periodically check if isProcessing_ and start the next Kf process
    std::deque<sensor_msgs::msg::Imu> imuMsgKfWindow_;
    pcl::PointCloud<PointType>::Ptr cloudKfWindow_;
    TimeLogger timeLogger_;
    StateLogger stateLogger_;
    ImuRawLogger imuRawLogger_;
    FactorGraphLogger fgStateLogger_;
    InequalityConstraintsLogger ineqConstLogger_;
    TwistLogger twistLogger_;

    // ---- for voxel map ---- 
    VoxelHashMap voxelMap_;

    // pose(transformation) variables
    Eigen::Quaterniond q_w_bCurrKf_;    // imu pose(rotation) w.r.t. world at the current keyframe(current scan)
    Eigen::Vector3d t_w_bCurrKf_;       // imu pose(translation) w.r.t. world at the current keyframe(current scan)
    Eigen::Quaterniond q_w_bPrevKf_;    // imu pose(rotation) w.r.t. world at the previous keyframe(previous scan)
    Eigen::Vector3d t_w_bPrevKf_;       // imu pose(translation) w.r.t. world at the previous keyframe(previous scan)
    Eigen::Quaterniond q_bPrevKf_bCurrKf_lo_; 
    Eigen::Vector3d t_bPrevKf_bCurrKf_lo_;
    Eigen::Quaterniond q_bPrevKf_bCurrKf_initGuess_;
    Eigen::Vector3d t_bPrevKf_bCurrKf_initGuess_;  
    Eigen::Quaterniond q_bPrev2Kf_bPrevKf_;     // for constant velocity model used in deskewing and LO init guess
    Eigen::Vector3d t_bPrev2Kf_bPrevKf_;
    // naming rule for transformation
    // q: rotation(quaternion)
    // t: translation
    // q_X_Y: rotation of X with respect to Y frame
    // t_X_Y: translation of X with respect to Y frame
    // e.g.) q_w_bPrevKf_: translation of imu pose at previous keyframe(scan)(=bPrevKf) with respect to the world frame(=w)
    

    // for ROS2 message handling
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdom_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloud_;
    std::shared_ptr<tf2_ros::Buffer>                 tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener>      tfListener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>   tfBroadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfStaticBroadcaster_;
    rclcpp::TimerBase::SharedPtr tfBroadcastTimer_;


    // ---- for preprocessing ---- 
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPointCloud_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;
    std::mutex mtxImu_;
    std::mutex mtxCloud_;
    std::deque<sensor_msgs::msg::Imu> imuMsgBuffer_;
    std::deque<sensor_msgs::msg::PointCloud2> cloudMsgBuffer_;
    bool isImuInitialized_;
    bool isCloudMapInitialized_;   
    bool isStateInitialized_;   
    bool isCurrFrameLidarOnlyEstimation_;   
    double timeCurrScanBeg_;
    double timeCurrScanEnd_;    
    double timePrevScanEnd_; 
    double timePrev2ScanEnd_;
    std::shared_ptr<gtsam::PreintegratedImuMeasurements> imuPropagator_;
    std::map<double, gtsam::NavState> imuPoseTimeline_;
    bool pointsTimestampAvailable_;
    Eigen::Vector3d meanAccInit_, meanGyrInit_;
    Eigen::Vector3d gravInit_;


    // ---- for lidar odometry ---- 
    pcl::PointCloud<PointType>::Ptr cloudMapLocal_; // map point cloud extracted from the global map // surf_from_map in LidarOdometry.cpp in Liliom
    pcl::PointCloud<PointType>::Ptr cloudMapLocalDs_; // map point cloud downsampled // surf_from_map_ds in LidarOdometry.cpp in Liliom
    pcl::KdTreeFLANN<PointType>::Ptr kdTreeMapLocal_; // kd tree for nn search to search map point cloud
    pcl::PointCloud<PointType>::Ptr cloudScanCurr_; // current scan point cloud downsampled
    pcl::PointCloud<PointType>::Ptr cloudScanCurrDs_; // current scan point cloud downsampled
    std::vector<Eigen::Vector3d> cloudScanCurrDs_vecEigen_; // to use kiss icp Voxel map
    std::vector<Eigen::Vector3d> cloudScanCurrDsToMap_vecEigen_;
    pcl::PointCloud<PointType>::Ptr cloudCurrentScanInWorld_;
    std::vector<Eigen::Vector3d> cloudScanCurrInWorld_vecEigen_; // to use kiss icp Voxel map
    pcl::VoxelGrid<PointType> downsizeFilterScanCurr_; // downsampling filter for current scan
    pcl::VoxelGrid<PointType> downsizeFilterMapLocal_; // downsampling filter for map point cloud
    std::vector<pcl::PointCloud<PointType>::Ptr> cloudFramesMapGlobal_; // surf_frames in LidarOdometry.cpp in Liliom
    int numResidual_ = 0;
    int numPtsNnFound_ = 0;
    std::vector<Eigen::Vector3d> pointScanCurrInBForResidual_;
    // std::vector<Eigen::Vector3d> pointScanCurrInWForResidual_;
    std::vector<Eigen::Vector3d> planeNormalForResidual_;
    std::vector<double> planeDistFromOriginForResidual_;


    // ---- for factor graph ---- 
    bool isFGInitialized_; 
    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr constLidarOdomNoise_;
    gtsam::noiseModel::Gaussian::shared_ptr lidarOdomNoise_;
    gtsam::Vector noiseModelBetweenBias_;
    gtsam::NonlinearFactorGraph graphFactors_;
    gtsam::Values graphValues_;
    int key_;
    std::shared_ptr<gtsam::PreintegratedImuMeasurements> imuIntegrator_;
    std::shared_ptr<gtsam::BatchFixedLagSmoother> batchFixedLagSmoother_;
    std::shared_ptr<gtsam::IncrementalFixedLagSmoother> isam2FixedLagSmoother_;
    gtsam::FixedLagSmoother::KeyTimestampMap keyTimestamps_;
    gtsam::Pose3 T_bPrevKf_bCurrKf_lo_;             // for logging
    gtsam::Pose3 T_w_bCurrKf_lo_;                   // for logging
    gtsam::Pose3 T_bPrevKf_bCurrKf_imu_log_;        // for logging

    gtsam::imuBias::ConstantBias biasPrevKf_;
    gtsam::NavState statePrevKf_;
    gtsam::imuBias::ConstantBias biasCurrKf_;
    gtsam::NavState stateCurrKf_;
    gtsam::Matrix9 stateCovPrevKf_ = Eigen::MatrixXd::Identity(9, 9);
    gtsam::Matrix9 stateCovPredCurrKf_imu_ = Eigen::MatrixXd::Identity(9, 9);
    gtsam::Matrix9 stateCovCurrKf_ = Eigen::MatrixXd::Identity(9, 9);


    // for logging 
    // gtsam::imuBias::ConstantBias biasCurrKf_initGuess_;
    // gtsam::Pose3 posCurrKf_initGuess_;
    // gtsam::Velocity3 velCurrKf_initGuess_;

    gtsam::NavState stateCurrKf_initGuess_lo_log_;
    gtsam::imuBias::ConstantBias biasCurrKf_initGuess_lo_log_;
    gtsam::NavState stateCurrKf_initGuess_imu_log_;
    gtsam::imuBias::ConstantBias biasCurrKf_initGuess_imu_log_;

    Eigen::MatrixXd lidarOdomCovariance_log_ = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd imuPreintCovariance_log_ = Eigen::MatrixXd::Identity(9, 9);
    Eigen::MatrixXd imuBiasCovariance_log_ = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd icpHessianLatest_log_ = Eigen::MatrixXd::Identity(6, 6);


    // ---- for qp inequality constraints ---- 
    // bool velocity_ready_ = false; // for the first iteration, run unconsrained least squares(solveLeastSquares()) to set timeScanBeg and timeScanCurr for velocity computation for ineq. constraints.
    double lm_lambda_; 
    
};

} // namespace lo_dev

#endif // FRONTEND_H
