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


namespace lo_dev 
{

struct Config_Frontend
{
    // ---- for general ---- 
    bool set_main_process_timer; // true: run() is called in timer callback, false: run() is called in pointcloud msg callback

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

    // ---- for lidar odometry ---- 
    int max_iterations;
    double max_solver_time_in_seconds;
    double voxel_filter_size;
    int icp_iteration_num;
    bool use_liosam_gauss_newton;
    bool use_fastlio_point_plane_residual_param;
    bool build_local_map_from_all_global_map;

    // ---- for factor graph----
    float lidar_correction_noise;
    float smooth_factor;
    bool  use_imu_roll_pitch;
    double lag; 
};

class Frontend : public rclcpp::Node 
{
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
    void synchronizeMeasurements();
    void convertCloudMsgToPcl(const sensor_msgs::msg::PointCloud2& msgIn, pcl::PointCloud<PointType>::Ptr cloudOut);
    void propagateImu();
    void transformPointCloudInImuFrame();
    void getImuPoseAtPointMeasurementTime(double pointTime, Eigen::Quaterniond& qOut, Eigen::Vector3d& tOut);
    void makeInitialGuess();
    void deskewPointCloud();

    // ---- for lidar odometry ----
    void initializeCloudMap();
    void setInitialPose();
    void buildLocalMap();
    void downsampleCloud();
    void preparePointPlaneResidual();
    void solveLeastSquares();
    void transformPointToWorldFrame(const PointType&  pi, PointType& po);
    void updatePoseLO();
    void updateCloudMap();
    void transformPointCloudInWorldFrame();


    // factor graph is commented out now (currently not implemented and might or might not be used in the future)
    // // ---- for factor graph ---- 
    // void resetInitFlags();  // resetParams() in original
    // void resetSmoother();   // resetOptimization() in original
    // void resetKeyframesPrior();  // reset_graph() in original
    // void initializeSystem(double initialTime);  // initial_system(double currentCorrectionTime, gtsam::Pose3 lidarPose) 
    // bool readParameters();
    // void initializeInterface();
    // void integrateImu();
    // void performFixedLagSmoothing();


private:
    // ---- for general ---- 
    Config_Frontend config_;
    std::atomic_bool isProcessing_; // for process flag avoiding racing in multi-thread
    rclcpp::TimerBase::SharedPtr mainProcessTimer;  // added to periodically check if isProcessing_ and start the next Kf process
    std::deque<sensor_msgs::msg::Imu> imuMsgKfWindow_;
    pcl::PointCloud<PointType>::Ptr cloudKfWindow_;

    // pose(transformation) variables
    Eigen::Quaterniond q_w_bPrevKf_;    // imu pose(rotation) w.r.t. world at the previous keyframe(previous scan)
    Eigen::Vector3d t_w_bPrevKf_;       // imu pose(translation) w.r.t. world at the previous keyframe(previous scan)
    Eigen::Quaterniond q_w_bCurrKf_;    // imu pose(rotation) w.r.t. world at the current keyframe(current scan)
    Eigen::Vector3d t_w_bCurrKf_;       // imu pose(translation) w.r.t. world at the current keyframe(current scan)
    Eigen::Quaterniond q_bPrevKf_bCurrKf_lo_; 
    Eigen::Vector3d t_bPrevKf_bCurrKf_lo_;
    Eigen::Quaterniond q_bPrevKf_bCurrKf_initGuess_;
    Eigen::Vector3d t_bPrevKf_bCurrKf_initGuess_;  
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
    double timePrevScanEnd_;
    double timeCurrScanBeg_;
    double timeCurrScanEnd_;
    std::shared_ptr<gtsam::PreintegratedImuMeasurements> imuPropagator_;
    std::map<double, gtsam::NavState> imuPoseTimeline_;
    bool isImuFirstPropagation_;
    Eigen::Vector3d meanAccInit_, meanGyrInit_;
    Eigen::Vector3d gravInit_;


    // ---- for lidar odometry ---- 
    pcl::PointCloud<PointType>::Ptr cloudMapLocal_; // map point cloud extracted from the global map // surf_from_map in LidarOdometry.cpp in Liliom
    pcl::PointCloud<PointType>::Ptr cloudMapLocalDs_; // map point cloud downsampled // surf_from_map_ds in LidarOdometry.cpp in Liliom
    pcl::KdTreeFLANN<PointType>::Ptr kdTreeMapLocal_; // kd tree for nn search to search map point cloud
    pcl::PointCloud<PointType>::Ptr cloudScanCurr_; // current scan point cloud downsampled
    pcl::PointCloud<PointType>::Ptr cloudScanCurrDs_; // current scan point cloud downsampled
    pcl::PointCloud<PointType>::Ptr cloudCurrentScanInWorld_;
    pcl::VoxelGrid<PointType> downsizeFilterScanCurr_; // downsampling filter for current scan
    pcl::VoxelGrid<PointType> downsizeFilterMapLocal_; // downsampling filter for map point cloud
    std::vector<pcl::PointCloud<PointType>::Ptr> cloudFramesMapGlobal_; // surf_frames in LidarOdometry.cpp in Liliom
    int numResidual_;
    pcl::PointCloud<PointType>::Ptr pointScanCurrForResidual_;
    pcl::PointCloud<PointType>::Ptr planeNormDistForResidual_;
    // int scan_match_cnt_ = 10; // num. of Icp iteration, which needed to be in config param


    // ---- for factor graph ---- (currently not used and might or might not be used in the future)
    bool isSystemInited_; 
    bool isFirstSmoothingDone_; 
    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise_;
    gtsam::Vector noiseModelBetweenBias_;
    gtsam::NonlinearFactorGraph graphFactors_;
    gtsam::Values graphValues_;
    int key_;
    std::shared_ptr<gtsam::PreintegratedImuMeasurements> imuIntegrator_;
    std::shared_ptr<gtsam::BatchFixedLagSmoother> fixedLagSmoother_;
    gtsam::FixedLagSmoother::KeyTimestampMap keyTimestamps_;
    gtsam::Pose3 T_bPrevKf_bCurrKf_lo_;
    gtsam::Pose3 posePrevKf_;
    gtsam::Vector3 velPrevKf_;
    gtsam::imuBias::ConstantBias biasPrevKf_;
    gtsam::NavState statePrevKf_;
    gtsam::Pose3 poseCurrKf_; 
    gtsam::Vector3 velCurrKf_;
    gtsam::imuBias::ConstantBias biasCurrKf_;
    gtsam::NavState stateCurrKf_;
};

} // namespace lo_dev

#endif // FRONTEND_H
