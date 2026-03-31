#include <lo_dev/Frontend/frontend.h>

// TODO left:
//  - deskewpointcloud() can be multi-threaded by replacing push_back
//  - all the pose should be GTSAM::Pose3 for consistency throughout the code


// novelty:
//  we are focusing on improving lidar icp using other sensors, as one module of our odometry in sensor fusion module (i.e., factor graph)
//  common approach: fuse lidar(degenerated) + imu + vision in factor graph, tuning covariance matrix(weight)
//  our approach: if lidar is degenerated, get lidar odom improved as much as possible using inequality constraints, getting a reasonable local minima in icp
//                 -> then fuse them in factor graph or anything
//                 -> also, we can use vision ekf


// Issue:
//  QP formulation fails with factor graph for some reason because of a lack of control on step size (probably)


// Current Observation:
//  - Imu noise parameters and lidar noise parameters are really important


// [TODO]
//  - comment out all the print out
//  - put icp iteration more (~20?)
//  try to remove pointcloud loading failure "Failed to find match for field 'range'"
//  - log inequality constraint activated time
//  - check the vehicle used in HeLiPR
//  - make the robot position bigger in rviz


namespace lo_dev
{    

Frontend::Frontend(const rclcpp::NodeOptions & options):Node("frontend_node", options)
{   
}

bool Frontend::readParameters()
{
    // ---- parameters for general ---- 
    this->declare_parameter<bool>("frontend_node.set_main_process_timer", true);
    this->declare_parameter<bool>("frontend_node.turn_off_pcl_conversion_error_printing", false);
    config_.set_main_process_timer = this->get_parameter("frontend_node.set_main_process_timer").as_bool();
    config_.turn_off_pcl_conversion_error_printing = this->get_parameter("frontend_node.turn_off_pcl_conversion_error_printing").as_bool();
    RCLCPP_INFO_STREAM(get_logger(), "set_main_process_timer: " << config_.set_main_process_timer);
    RCLCPP_INFO_STREAM(get_logger(), "turn_off_pcl_conversion_error_printing: " << config_.turn_off_pcl_conversion_error_printing);

    // ---- parameters for voxel map ---- 
    this->declare_parameter<bool>("frontend_node.use_voxel_map", false);
    this->declare_parameter<double>("frontend_node.voxel_size", 0.5);
    this->declare_parameter<double>("frontend_node.voxel_max_distance", 100.0);
    this->declare_parameter<int>("frontend_node.max_points_per_voxel", 20);
    this->declare_parameter<bool>("frontend_node.turn_on_voxel_downsample", true);
    this->declare_parameter<double>("frontend_node.voxel_downsample_resolution_raw_to_mapping", 0.5);
    this->declare_parameter<double>("frontend_node.voxel_downsample_resolution_mapping_to_icpsource", 0.5);
    this->declare_parameter<double>("frontend_node.threshold_voxel_nn_search_radius", 1.5);
    config_.use_voxel_map= this->get_parameter("frontend_node.use_voxel_map").as_bool();
    config_.voxel_size= this->get_parameter("frontend_node.voxel_size").as_double();
    config_.voxel_max_distance= this->get_parameter("frontend_node.voxel_max_distance").as_double();
    config_.max_points_per_voxel= static_cast<unsigned int>(this->get_parameter("frontend_node.max_points_per_voxel").as_int());
    config_.turn_on_voxel_downsample = this->get_parameter("frontend_node.turn_on_voxel_downsample").as_bool();
    config_.voxel_downsample_resolution_raw_to_mapping = this->get_parameter("frontend_node.voxel_downsample_resolution_raw_to_mapping").as_double();
    config_.voxel_downsample_resolution_mapping_to_icpsource = this->get_parameter("frontend_node.voxel_downsample_resolution_mapping_to_icpsource").as_double();
    config_.threshold_voxel_nn_search_radius = this->get_parameter("frontend_node.threshold_voxel_nn_search_radius").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "use_voxel_map: " << config_.use_voxel_map);
    RCLCPP_INFO_STREAM(get_logger(), "voxel_size: " << config_.voxel_size);
    RCLCPP_INFO_STREAM(get_logger(), "voxel_max_distance: " << config_.voxel_max_distance);
    RCLCPP_INFO_STREAM(get_logger(), "max_points_per_voxel: " << config_.max_points_per_voxel);
    RCLCPP_INFO_STREAM(get_logger(), "turn_on_voxel_downsample: " << config_.turn_on_voxel_downsample);
    RCLCPP_INFO_STREAM(get_logger(), "voxel_downsample_resolution_raw_to_mapping: " << config_.voxel_downsample_resolution_raw_to_mapping);
    RCLCPP_INFO_STREAM(get_logger(), "voxel_downsample_resolution_mapping_to_icpsource: " << config_.voxel_downsample_resolution_mapping_to_icpsource);
    RCLCPP_INFO_STREAM(get_logger(), "threshold_voxel_nn_search_radius: " << config_.threshold_voxel_nn_search_radius);

    // ---- parameters for preprocessing ---- 
    this->declare_parameter<float>("frontend_node.acc_n", 1e-3);
    this->declare_parameter<float>("frontend_node.acc_w", 1e-3);
    this->declare_parameter<float>("frontend_node.gyr_n", 1e-6);
    this->declare_parameter<float>("frontend_node.gyr_w", 1e-6);
    this->declare_parameter<float>("frontend_node.g_norm", 9.80511);
    this->declare_parameter<int>("frontend_node.scan_line", 4);
    this->declare_parameter<double>("frontend_node.imu_acc_x_limit", 1.0);
    this->declare_parameter<double>("frontend_node.imu_acc_y_limit", 1.0);
    this->declare_parameter<double>("frontend_node.imu_acc_z_limit", 1.0);
    this->declare_parameter<std::string>("frontend_node.sensor", "ouster");
    this->declare_parameter<double>("frontend_node.lidar_min_range", 2.0);
    this->declare_parameter<bool>("frontend_node.enable_min_range_filter", true);
    this->declare_parameter<double>("frontend_node.lidar_scan_rate", 100.0);
    this->declare_parameter<std::string>("frontend_node.point_cloud_msg_timestamp", "scan_end_time");
    this->declare_parameter<bool>("frontend_node.print_sync_status", true);

    config_.imu_acc_noise = this->get_parameter("frontend_node.acc_n").as_double();
    config_.imu_acc_bias_noise = this->get_parameter("frontend_node.acc_w").as_double();
    config_.imu_gyr_noise = this->get_parameter("frontend_node.gyr_n").as_double();
    config_.imu_gyr_bias_noise = this->get_parameter("frontend_node.gyr_w").as_double();
    config_.imu_gravity = this->get_parameter("frontend_node.g_norm").as_double();
    config_.num_scans = this->get_parameter("frontend_node.scan_line").as_int();
    config_.imu_acc_x_limit = this->get_parameter("frontend_node.imu_acc_x_limit").as_double();
    config_.imu_acc_y_limit = this->get_parameter("frontend_node.imu_acc_y_limit").as_double();
    config_.imu_acc_z_limit = this->get_parameter("frontend_node.imu_acc_z_limit").as_double();
    config_.lidar_min_range = this->get_parameter("frontend_node.lidar_min_range").as_double();
    config_.enable_min_range_filter = this->get_parameter("frontend_node.enable_min_range_filter").as_bool();
    config_.lidar_scan_rate = this->get_parameter("frontend_node.lidar_scan_rate").as_double();
    config_.point_cloud_msg_timestamp = this->get_parameter("frontend_node.point_cloud_msg_timestamp").as_string();
    config_.print_sync_status = this->get_parameter("frontend_node.print_sync_status").as_bool();

    if (SENSOR == "livox") 
    {
        // currently livox sensor is not supported
        config_.sensor = SensorType::LIVOX;
    } 
    else if (SENSOR == "velodyne") 
    {
        config_.sensor = SensorType::VELODYNE;
    }
    else if (SENSOR == "ouster") 
    {
        config_.sensor = SensorType::OUSTER;
    } 

    RCLCPP_INFO_STREAM(this->get_logger(), "sensor: " << SENSOR);
    RCLCPP_INFO_STREAM(this->get_logger(), "acc_n: " << config_.imu_acc_noise);
    RCLCPP_INFO_STREAM(this->get_logger(), "acc_w: " << config_.imu_acc_bias_noise);
    RCLCPP_INFO_STREAM(this->get_logger(), "gyr_n: " << config_.imu_gyr_noise);
    RCLCPP_INFO_STREAM(this->get_logger(), "gyr_w: " << config_.imu_gyr_bias_noise);
    RCLCPP_INFO_STREAM(this->get_logger(), "g_norm: " << config_.imu_gravity);
    RCLCPP_INFO_STREAM(this->get_logger(), "scan_line: " << config_.num_scans);
    RCLCPP_INFO_STREAM(this->get_logger(), "imu_acc_x_limit: " << config_.imu_acc_x_limit);
    RCLCPP_INFO_STREAM(this->get_logger(), "imu_acc_y_limit: " << config_.imu_acc_y_limit);
    RCLCPP_INFO_STREAM(this->get_logger(), "imu_acc_z_limit: " << config_.imu_acc_z_limit);
    RCLCPP_INFO_STREAM(this->get_logger(), "lidar_min_range: " << config_.lidar_min_range);
    RCLCPP_INFO_STREAM(this->get_logger(), "enable_min_range_filter: " << config_.enable_min_range_filter);
    RCLCPP_INFO_STREAM(this->get_logger(), "lidar_scan_rate: " << config_.lidar_scan_rate);
    RCLCPP_INFO_STREAM(this->get_logger(), "point_cloud_msg_timestamp: " << config_.point_cloud_msg_timestamp);
    RCLCPP_INFO_STREAM(this->get_logger(), "print_sync_status: " << config_.print_sync_status);



    // ---- parameters for lidar odometry ---- 
    this->declare_parameter<int>("frontend_node.max_iterations", 4);
    this->declare_parameter<double>("frontend_node.max_solver_time_in_seconds", 0.015);
    this->declare_parameter<double>("frontend_node.voxel_filter_size", 0.4);
    this->declare_parameter<int>("frontend_node.max_cloud_frame_for_local_map", 5);
    this->declare_parameter<int>("frontend_node.icp_iteration_num", 10);
    this->declare_parameter<bool>("frontend_node.use_liosam_gauss_newton", false);
    this->declare_parameter<bool>("frontend_node.use_fastlio_point_plane_residual_param", false);
    this->declare_parameter<bool>("frontend_node.build_local_map_from_all_global_map", false);
    this->declare_parameter<std::string>("frontend_node.initial_guess_source", "");
    this->declare_parameter<std::string>("frontend_node.motion_compensation_source", "");

    config_.max_iterations = this->get_parameter("frontend_node.max_iterations").as_int();
    config_.max_solver_time_in_seconds = this->get_parameter("frontend_node.max_solver_time_in_seconds").as_double();
    config_.voxel_filter_size = this->get_parameter("frontend_node.voxel_filter_size").as_double();
    config_.max_cloud_frame_for_local_map = this->get_parameter("frontend_node.max_cloud_frame_for_local_map").as_int();
    config_.icp_iteration_num = this->get_parameter("frontend_node.icp_iteration_num").as_int();
    config_.use_liosam_gauss_newton = this->get_parameter("frontend_node.use_liosam_gauss_newton").as_bool();
    config_.use_fastlio_point_plane_residual_param = this->get_parameter("frontend_node.use_fastlio_point_plane_residual_param").as_bool();
    config_.build_local_map_from_all_global_map = this->get_parameter("frontend_node.build_local_map_from_all_global_map").as_bool();
    config_.initial_guess_source = this->get_parameter("frontend_node.initial_guess_source").as_string();
    config_.motion_compensation_source = this->get_parameter("frontend_node.motion_compensation_source").as_string();

    RCLCPP_INFO_STREAM(this->get_logger(), "max_iterations: " << config_.max_iterations);
    RCLCPP_INFO_STREAM(this->get_logger(), "max_solver_time_in_seconds: " << config_.max_solver_time_in_seconds);
    RCLCPP_INFO_STREAM(this->get_logger(), "voxel_filter_size: " << config_.voxel_filter_size);
    RCLCPP_INFO_STREAM(this->get_logger(), "max_cloud_frame_for_local_map: " << config_.max_cloud_frame_for_local_map);
    RCLCPP_INFO_STREAM(this->get_logger(), "icp_iteration_num: " << config_.icp_iteration_num);
    RCLCPP_INFO_STREAM(this->get_logger(), "use_liosam_gauss_newton: " << config_.use_liosam_gauss_newton);
    RCLCPP_INFO_STREAM(this->get_logger(), "use_fastlio_point_plane_residual_param: " << config_.use_fastlio_point_plane_residual_param);
    RCLCPP_INFO_STREAM(this->get_logger(), "build_local_map_from_all_global_map: " << config_.build_local_map_from_all_global_map);
    RCLCPP_INFO_STREAM(this->get_logger(), "initial_guess_source: " << config_.initial_guess_source);
    RCLCPP_INFO_STREAM(this->get_logger(), "motion_compensation_source: " << config_.motion_compensation_source);


    // ---- parameters for factor graph (currently not implemented and might or might not be used in the future) ---- 
    this->declare_parameter<bool>("frontend_node.turn_on_factor_graph",false);
    this->declare_parameter<float>("frontend_node.lidar_correction_noise",0.01);
    this->declare_parameter<float>("frontend_node.smooth_factor",0.9);
    this->declare_parameter<double>("frontend_node.fixed_lag", 2.0);
    this->declare_parameter<bool>("frontend_node.use_lo_prior_factor_wo_between_factor", true);
    this->declare_parameter<std::string>("frontend_node.factor_graph_init_guess_source", "lidar");
    this->declare_parameter<std::string>("frontend_node.fixed_lag_smoother_batch_or_incremental", "batch");
    this->declare_parameter<bool>("frontend_node.fix_lidar_odom_covariance", false);

    config_.turn_on_factor_graph = this->get_parameter("frontend_node.turn_on_factor_graph").as_bool();
    config_.lidar_correction_noise = this->get_parameter("frontend_node.lidar_correction_noise").as_double();
    config_.smooth_factor = this->get_parameter("frontend_node.smooth_factor").as_double();
    config_.fixed_lag = this->get_parameter("frontend_node.fixed_lag").as_double();
    config_.use_lo_prior_factor_wo_between_factor = this->get_parameter("frontend_node.use_lo_prior_factor_wo_between_factor").as_bool();
    config_.factor_graph_init_guess_source = this->get_parameter("frontend_node.factor_graph_init_guess_source").as_string();
    config_.fixed_lag_smoother_batch_or_incremental = this->get_parameter("frontend_node.fixed_lag_smoother_batch_or_incremental").as_string();
    config_.fix_lidar_odom_covariance = this->get_parameter("frontend_node.fix_lidar_odom_covariance").as_bool();

    RCLCPP_INFO_STREAM(this->get_logger(), "turn_on_factor_graph: " << config_.turn_on_factor_graph);
    RCLCPP_INFO_STREAM(this->get_logger(), "lidar_correction_noise: " << config_.lidar_correction_noise);
    RCLCPP_INFO_STREAM(this->get_logger(), "smooth_factor: " << config_.smooth_factor);
    RCLCPP_INFO_STREAM(this->get_logger(), "fixed_lag: " << config_.fixed_lag);
    RCLCPP_INFO_STREAM(this->get_logger(), "use_lo_prior_factor_wo_between_factor: " << config_.use_lo_prior_factor_wo_between_factor);
    RCLCPP_INFO_STREAM(this->get_logger(), "factor_graph_init_guess_source: " << config_.factor_graph_init_guess_source);
    RCLCPP_INFO_STREAM(this->get_logger(), "fixed_lag_smoother_batch_or_incremental: " << config_.fixed_lag_smoother_batch_or_incremental);
    RCLCPP_INFO_STREAM(this->get_logger(), "fix_lidar_odom_covariance: " << config_.fix_lidar_odom_covariance);


    // ---- parameters for inequality constraints ---- 
    this->declare_parameter<bool>("frontend_node.turn_on_qp_ineq_constraints_active_set",false);
    this->declare_parameter<bool>("frontend_node.turn_on_ineq_constraints",false);
    this->declare_parameter<std::string>("frontend_node.ineq_constraints_type", "curvature");
    this->declare_parameter<int>("frontend_node.sqp_iteration_num_qp_active_set", 10);
    this->declare_parameter<bool>("frontend_node.turn_on_levenberg_marquardt_qp_active_set", false);
    this->declare_parameter<double>("frontend_node.levenberg_marquardt_lambda_qp_active_set", 2.0);
    this->declare_parameter<bool>("frontend_node.turn_on_levenberg_marquardt_qp_active_set_marquardt_damping", true);
    this->declare_parameter<bool>("frontend_node.turn_on_robust_kernel_qp_active_set", true);
    this->declare_parameter<bool>("frontend_node.turn_on_mad_based_scaling_for_kernel_weights_qp_active_set", true);
    this->declare_parameter<std::string>("frontend_node.robust_kernel_qp_active_set", "");
    this->declare_parameter<bool>("frontend_node.turn_on_jacobian_column_scaling_qp_active_set", "");
    this->declare_parameter<std::string>("frontend_node.pose_increment_multiplication", "left");
    this->declare_parameter<bool>("frontend_node.turn_on_Sophus_SE3_update", true);
    this->declare_parameter<std::string>("frontend_node.qp_active_set_Hessian_computation", "for_loop");
    this->declare_parameter<double>("frontend_node.vehicle_kinematic_constraints.max_steering_angle", 0.0);
    this->declare_parameter<double>("frontend_node.vehicle_kinematic_constraints.min_steering_angle", 0.0);
    this->declare_parameter<double>("frontend_node.vehicle_kinematic_constraints.wheel_base", 0.0);
    this->declare_parameter<bool>("frontend_node.qp_compute_ineq_from_imu_cov", false);
    this->declare_parameter<double>("frontend_node.qp_compute_ineq_from_imu_cov_sigma_coef", 0.01);
    this->declare_parameter<bool>("frontend_node.levenberg_marquardt_trust_region_lambda_adjustment", true);
    this->declare_parameter<double>("frontend_node.levenberg_marquardt_trust_region_lambda_min", 1e-3);
    this->declare_parameter<double>("frontend_node.levenberg_marquardt_trust_region_step_quality_threshold", 0.5);
    this->declare_parameter<bool>("frontend_node.bfgs_on_inequality_constraint", false);
    this->declare_parameter<bool>("frontend_node.ifopt_on_inequality_constraint", false);
    this->declare_parameter<std::string>("frontend_node.inequality_constraints_solver", "");

    config_.turn_on_qp_ineq_constraints_active_set = this->get_parameter("frontend_node.turn_on_qp_ineq_constraints_active_set").as_bool();
    config_.turn_on_ineq_constraints = this->get_parameter("frontend_node.turn_on_ineq_constraints").as_bool();
    config_.ineq_constraints_type = this->get_parameter("frontend_node.ineq_constraints_type").as_string();
    config_.sqp_iteration_num_qp_active_set = this->get_parameter("frontend_node.sqp_iteration_num_qp_active_set").as_int();
    config_.turn_on_levenberg_marquardt_qp_active_set = this->get_parameter("frontend_node.turn_on_levenberg_marquardt_qp_active_set").as_bool();
    config_.levenberg_marquardt_lambda_qp_active_set = this->get_parameter("frontend_node.levenberg_marquardt_lambda_qp_active_set").as_double();
    config_.turn_on_levenberg_marquardt_qp_active_set_marquardt_damping = this->get_parameter("frontend_node.turn_on_levenberg_marquardt_qp_active_set_marquardt_damping").as_bool();
    config_.turn_on_robust_kernel_qp_active_set = this->get_parameter("frontend_node.turn_on_robust_kernel_qp_active_set").as_bool();
    config_.turn_on_mad_based_scaling_for_kernel_weights_qp_active_set = this->get_parameter("frontend_node.turn_on_mad_based_scaling_for_kernel_weights_qp_active_set").as_bool();
    config_.robust_kernel_qp_active_set = this->get_parameter("frontend_node.robust_kernel_qp_active_set").as_string();
    config_.turn_on_jacobian_column_scaling_qp_active_set = this->get_parameter("frontend_node.turn_on_jacobian_column_scaling_qp_active_set").as_bool();
    config_.pose_increment_multiplication = this->get_parameter("frontend_node.pose_increment_multiplication").as_string();
    config_.turn_on_Sophus_SE3_update = this->get_parameter("frontend_node.turn_on_Sophus_SE3_update").as_bool();
    config_.qp_active_set_Hessian_computation = this->get_parameter("frontend_node.qp_active_set_Hessian_computation").as_string();
    config_.vehicle_kinematic_constraints_max_steering_angle = this->get_parameter("frontend_node.vehicle_kinematic_constraints.max_steering_angle").as_double();
    config_.vehicle_kinematic_constraints_min_steering_angle = this->get_parameter("frontend_node.vehicle_kinematic_constraints.min_steering_angle").as_double();
    config_.vehicle_kinematic_constraints_wheel_base = this->get_parameter("frontend_node.vehicle_kinematic_constraints.wheel_base").as_double();
    config_.qp_compute_ineq_from_imu_cov = this->get_parameter("frontend_node.qp_compute_ineq_from_imu_cov").as_bool();
    config_.qp_compute_ineq_from_imu_cov_sigma_coef = this->get_parameter("frontend_node.qp_compute_ineq_from_imu_cov_sigma_coef").as_double();
    config_.levenberg_marquardt_trust_region_lambda_adjustment = this->get_parameter("frontend_node.levenberg_marquardt_trust_region_lambda_adjustment").as_bool();
    config_.levenberg_marquardt_trust_region_lambda_min = this->get_parameter("frontend_node.levenberg_marquardt_trust_region_lambda_min").as_double();
    config_.levenberg_marquardt_trust_region_step_quality_threshold = this->get_parameter("frontend_node.levenberg_marquardt_trust_region_step_quality_threshold").as_double();
    config_.bfgs_on_inequality_constraint = this->get_parameter("frontend_node.bfgs_on_inequality_constraint").as_bool();
    config_.ifopt_on_inequality_constraint = this->get_parameter("frontend_node.ifopt_on_inequality_constraint").as_bool();
    config_.inequality_constraints_solver = this->get_parameter("frontend_node.inequality_constraints_solver").as_string();

    RCLCPP_INFO_STREAM(this->get_logger(), "turn_on_qp_ineq_constraints_active_set: " << config_.turn_on_qp_ineq_constraints_active_set);
    RCLCPP_INFO_STREAM(this->get_logger(), "turn_on_ineq_constraints: " << config_.turn_on_ineq_constraints);
    RCLCPP_INFO_STREAM(this->get_logger(), "ineq_constraints_type: " << config_.ineq_constraints_type);
    RCLCPP_INFO_STREAM(this->get_logger(), "sqp_iteration_num: " << config_.sqp_iteration_num_qp_active_set);
    RCLCPP_INFO_STREAM(this->get_logger(), "turn_on_levenberg_marquardt: " << config_.turn_on_levenberg_marquardt_qp_active_set);
    RCLCPP_INFO_STREAM(this->get_logger(), "levenberg_marquardt_lambda: " << config_.levenberg_marquardt_lambda_qp_active_set);
    RCLCPP_INFO_STREAM(this->get_logger(), "turn_on_levenberg_marquardt_qp_active_set_marquardt_damping: " << config_.turn_on_levenberg_marquardt_qp_active_set_marquardt_damping);
    RCLCPP_INFO_STREAM(this->get_logger(), "turn_on_robust_kernel_qp_active_set: " << config_.turn_on_robust_kernel_qp_active_set);
    RCLCPP_INFO_STREAM(this->get_logger(), "turn_on_mad_based_scaling_for_kernel_weights_qp_active_set: " << config_.turn_on_mad_based_scaling_for_kernel_weights_qp_active_set);
    RCLCPP_INFO_STREAM(this->get_logger(), "robust_kernel_qp_active_set: " << config_.robust_kernel_qp_active_set);
    RCLCPP_INFO_STREAM(this->get_logger(), "turn_on_jacobian_column_scaling_qp_active_set: " << config_.turn_on_jacobian_column_scaling_qp_active_set);
    RCLCPP_INFO_STREAM(this->get_logger(), "pose_increment_multiplication: " << config_.pose_increment_multiplication);
    RCLCPP_INFO_STREAM(this->get_logger(), "turn_on_Sophus_SE3_update: " << config_.turn_on_Sophus_SE3_update);
    RCLCPP_INFO_STREAM(this->get_logger(), "qp_active_set_Hessian_computation: " << config_.qp_active_set_Hessian_computation);
    RCLCPP_INFO_STREAM(this->get_logger(), "vehicle_kinematic_constraints_max_steering_angle: " << config_.vehicle_kinematic_constraints_max_steering_angle);
    RCLCPP_INFO_STREAM(this->get_logger(), "vehicle_kinematic_constraints.min_steering_angle: " << config_.vehicle_kinematic_constraints_min_steering_angle);
    RCLCPP_INFO_STREAM(this->get_logger(), "vehicle_kinematic_constraints.wheel_base: " << config_.vehicle_kinematic_constraints_wheel_base);
    RCLCPP_INFO_STREAM(this->get_logger(), "qp_compute_ineq_from_imu_cov: " << config_.qp_compute_ineq_from_imu_cov);
    RCLCPP_INFO_STREAM(this->get_logger(), "qp_compute_ineq_from_imu_cov_sigma_coef: " << config_.qp_compute_ineq_from_imu_cov_sigma_coef);
    RCLCPP_INFO_STREAM(this->get_logger(), "levenberg_marquardt_trust_region_lambda_adjustment: " << config_.levenberg_marquardt_trust_region_lambda_adjustment);
    RCLCPP_INFO_STREAM(this->get_logger(), "levenberg_marquardt_trust_region_lambda_min: " << config_.levenberg_marquardt_trust_region_lambda_min);
    RCLCPP_INFO_STREAM(this->get_logger(), "levenberg_marquardt_trust_region_step_quality_threshold: " << config_.levenberg_marquardt_trust_region_step_quality_threshold);
    RCLCPP_INFO_STREAM(this->get_logger(), "bfgs_on_inequality_constraint: " << config_.bfgs_on_inequality_constraint);
    RCLCPP_INFO_STREAM(this->get_logger(), "ifopt_on_inequality_constraint: " << config_.ifopt_on_inequality_constraint);
    RCLCPP_INFO_STREAM(this->get_logger(), "inequality_constraints_solver: " << config_.inequality_constraints_solver);


    // ---- for ekf imu propagation ---- 
    this->declare_parameter<bool>("frontend_node.ekf_imu_covariance_prop_on",false);
    this->declare_parameter<bool>("frontend_node.ekf_use_const_prior_cov",false);
    this->declare_parameter<double>("frontend_node.ekf_const_diagonal_elem_prior_cov",0.01);

    config_.ekf_imu_covariance_prop_on = this->get_parameter("frontend_node.ekf_imu_covariance_prop_on").as_bool();
    config_.ekf_use_const_prior_cov = this->get_parameter("frontend_node.ekf_use_const_prior_cov").as_bool();
    config_.ekf_const_diagonal_elem_prior_cov = this->get_parameter("frontend_node.ekf_const_diagonal_elem_prior_cov").as_double();

    RCLCPP_INFO_STREAM(this->get_logger(), "ekf_imu_covariance_prop_on: " << config_.ekf_imu_covariance_prop_on);
    RCLCPP_INFO_STREAM(this->get_logger(), "ekf_use_const_prior_cov: " << config_.ekf_use_const_prior_cov);
    RCLCPP_INFO_STREAM(this->get_logger(), "ekf_const_diagonal_elem_prior_cov: " << config_.ekf_const_diagonal_elem_prior_cov);

    return true;
}

void Frontend::initializeInterface()
{
    // // check OpenMP
    // #pragma omp parallel
    // std::cout << "Hello from thread " << omp_get_thread_num() << " / " << omp_get_num_threads() << std::endl;

    // get the config parameters
    if(!readGlobalparam(shared_from_this()))
    {
        RCLCPP_ERROR(this->get_logger(), "[lo_dev::Frontend] Could not read calibration. Exiting...");
        rclcpp::shutdown();
    }

    if (!readParameters())
    {
        RCLCPP_ERROR(this->get_logger(), "[lo_dev::Frontend] Could not read parameters. Exiting...");
        rclcpp::shutdown();
    }

    // make sure the number of lidar scan is valid (actually not important but keep this just in case)
    if (config_.num_scans != 16 && config_.num_scans != 32 && config_.num_scans != 64 && config_.num_scans != 4 && config_.num_scans != 128)
    {
        RCLCPP_ERROR(this->get_logger(), "only support velodyne, livox, ouster with 16, 32, 64 or 128 scan line! and livox mid 360");
        rclcpp::shutdown();
    }
    
    // if a param 'set_main_process_timer' is true, call the main process (run()) in timer 
    if (config_.set_main_process_timer == true)
    {
        mainProcessTimer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1.)), std::bind(&Frontend::run, this));   
    }

    // set measurement subscriber callback group 'Reentrant' to ensure multi-threading process.
    rclcpp::CallbackGroup::SharedPtr subCbGr = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions subOpt;
    subOpt.callback_group = subCbGr;
    
    // set ROS2 qos
    rclcpp::QoS imuQos(10);
    // imuQos.best_effort();  // use BEST_EFFORT reliability
    imuQos.reliable();  // use RELIABLE reliability
    // imuQos.keep_last(10);  // keep last 10 messages
    imuQos.keep_last(50);  // keep last 10 messages

    rclcpp::QoS lidarQos(10);
    lidarQos.best_effort();  // use BEST_EFFORT reliability
    // lidarQos.keep_last(2);  // keep last 2 messages
    lidarQos.keep_last(10);  // keep last 10 messages

    // initialize point cloud subscriber
    if (config_.sensor == SensorType::VELODYNE || config_.sensor == SensorType::OUSTER) 
    {
        subPointCloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            LIDAR_TOPIC, lidarQos, 
            std::bind(&Frontend::cloudHandler, this, std::placeholders::_1),
            subOpt);
    } 
    else if (config_.sensor == SensorType::LIVOX) 
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Currently Livox sensors are not supported.");
        // Livox point cloud is not currently implemented
        // subLivoxCloud = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        //     LIDAR_TOPIC, 20, 
        //     std::bind(&Frontend::cloudHandler, this, std::placeholders::_1), 
        //     sub_options);
    }
    else
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Input sensor type is not supported.");
    }

    // initialize imu subscriber
    subImu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        IMU_TOPIC, imuQos, 
        std::bind(&Frontend::imuHandler, this, std::placeholders::_1), 
        subOpt);


    // initialize point cloud and odometry(estimated pose) publisher
    pubCloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(ProjectName+"/result/pointcloud", 2);
    pubOdom_ = this->create_publisher<nav_msgs::msg::Odometry>(ProjectName+"/result/odometry", 10); // for odometry evaluation

    // initialize ROS2 transform broadcaster just for Rviz visualization
    rclcpp::TimeSource ts(shared_from_this());
    ts.attachClock(this->get_clock());
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, shared_from_this(), false);
    tfStaticBroadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());
    tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
    tfBroadcastTimer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Frontend::publishTf, this));   
    publishStaticTf();

    // initialize variables
    isImuInitialized_ = false;
    isCloudMapInitialized_ = false;
    isStateInitialized_ = false;
    isCurrFrameLidarOnlyEstimation_ = false;
    timePrevScanEnd_ = 0.0;
    timeCurrScanBeg_ = 0.0;
    timeCurrScanEnd_ = 0.0;
    timePrev2ScanEnd_ = 0.0;
    numResidual_ = 0;   

    // initialize imu propagator(integrator)
    std::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(config_.imu_gravity);
    // p->accelerometerCovariance=gtsam::Matrix33::Identity(3,3)*pow(config_.imu_acc_noise,2); // acc white noise in continuous
    // p->gyroscopeCovariance=gtsam::Matrix33::Identity(3,3)*pow(config_.imu_gyr_noise,2); // gyro white noise in continuous
    // p->integrationCovariance=gtsam::Matrix33::Identity(3,3)*pow(1e-4,2); // error committed in integrating position from velocities
    p->accelerometerCovariance=gtsam::Matrix33::Identity(3,3)*1e-2; 
    p->gyroscopeCovariance=gtsam::Matrix33::Identity(3,3)*1e-2;
    p->integrationCovariance=gtsam::Matrix33::Identity(3,3)*1e-2;
    gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0,0,0,0,0,0).finished());
    imuPropagator_=std::make_shared<gtsam::PreintegratedImuMeasurements>(p, prior_imu_bias);
    // is there any parameter left for preintegration parameter?

    // initialize point cloud container
    cloudKfWindow_.reset(new pcl::PointCloud<PointType>());
    cloudMapLocal_.reset(new pcl::PointCloud<PointType>());
    cloudMapLocalDs_.reset(new pcl::PointCloud<PointType>());
    kdTreeMapLocal_.reset(new pcl::KdTreeFLANN<PointType>());
    cloudScanCurr_.reset(new pcl::PointCloud<PointType>());
    cloudScanCurrDs_.reset(new pcl::PointCloud<PointType>());
    cloudCurrentScanInWorld_.reset(new pcl::PointCloud<PointType>());
    const double size = config_.voxel_filter_size;
    downsizeFilterScanCurr_.setLeafSize(size, size, size);
    downsizeFilterMapLocal_.setLeafSize(size, size, size); //config_.voxel_filter_size

    // initialize pose variables
    q_w_bPrevKf_ = Eigen::Quaterniond::Identity();
    t_w_bPrevKf_ = Eigen::Vector3d::Zero();
    q_bPrevKf_bCurrKf_lo_ = Eigen::Quaterniond::Identity();
    t_bPrevKf_bCurrKf_lo_ = Eigen::Vector3d::Zero();
    q_w_bCurrKf_ = Eigen::Quaterniond::Identity();
    t_w_bCurrKf_ = Eigen::Vector3d::Zero();
    q_bPrevKf_bCurrKf_initGuess_ = Eigen::Quaterniond::Identity();
    t_bPrevKf_bCurrKf_initGuess_ = Eigen::Vector3d::Zero();
    q_bPrev2Kf_bPrevKf_ = Eigen::Quaterniond::Identity(); 
    t_bPrev2Kf_bPrevKf_ = Eigen::Vector3d::Zero();

    T_bPrevKf_bCurrKf_lo_ = gtsam::Pose3();
    T_w_bCurrKf_lo_ = gtsam::Pose3();
    T_bPrevKf_bCurrKf_imu_log_ = gtsam::Pose3();

    statePrevKf_ = gtsam::NavState();
    stateCurrKf_ = gtsam::NavState();

    // ---- initialization for factor graph optimization (currently not implemented and might or might not be used in the future) ---- 
    key_ = 0;
    priorPoseNoise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2,1e-2,1e-2,1e-2,1e-2,1e-2).finished()); // rad,rad,rad,m,m,m,should be a config parameter
    priorVelNoise_ = gtsam::noiseModel::Isotropic::Sigma(3,1e-2);                      // m/s , should be a config parameter
    priorBiasNoise_ = gtsam::noiseModel::Isotropic::Sigma(6,1e-3);                    // 1e-2 ~ 1e-3 seems to be good, should be a config parameter
    // priorBiasNoise_ = gtsam::noiseModel::Isotropic::Sigma(6,1e-2);                    // 1e-2 ~ 1e-3 seems to be good, should be a config parameter
    // priorBiasNoise_ = gtsam::noiseModel::Isotropic::Sigma(6,1e-1);                    // 1e-2 ~ 1e-3 seems to be good, should be a config parameter
    constLidarOdomNoise_ = gtsam::noiseModel::Isotropic::Sigma(6, config_.lidar_correction_noise); // meter
    noiseModelBetweenBias_ = (gtsam::Vector(6)
            << config_.imu_acc_bias_noise,config_.imu_acc_bias_noise,config_.imu_acc_bias_noise,config_.imu_gyr_bias_noise,config_.imu_gyr_bias_noise,config_.imu_gyr_bias_noise)
            .finished();
    imuIntegrator_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(p, prior_imu_bias);
    if(config_.fixed_lag_smoother_batch_or_incremental == "batch"){
        batchFixedLagSmoother_ = std::make_shared<gtsam::BatchFixedLagSmoother>(config_.fixed_lag);
    }else if(config_.fixed_lag_smoother_batch_or_incremental == "incremental"){
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.0; // Set the relin threshold to zero such that the batch estimate is recovered
        parameters.relinearizeSkip = 1; // Relinearize every time  
        isam2FixedLagSmoother_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(config_.fixed_lag, parameters);
        // ref: /home/ysugano/code_review/gtsam_4-3/examples/FixedLagSmootherExample.cpp
    }else{
        RCLCPP_INFO_STREAM(get_logger(), "Smoother type is not defined. No smoother was created.");
    }

    biasPrevKf_ = gtsam::imuBias::ConstantBias(); // initialize bias, which is updated later in performFixedLagSmoothing()
    biasCurrKf_ = gtsam::imuBias::ConstantBias(); // initialize bias, which is updated later in performFixedLagSmoothing()
    isFGInitialized_ = false;

    // [Note]
    // gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2,1e-2,1e-2,1e-2,1e-2,1e-2).finished())
    //  -> 1e-2 becomes standard deviations, the diagonal of the square root covariance matrix
    //     , where the covariance matrix is a diagonal matrix which is:
    //    i.e., [(1e-2)^2         0        0        0        0        0]
    //          [       0  (1e-2)^2        0        0        0        0]
    //          [       0         0 (1e-2)^2        0        0        0]
    //          [       0         0        0 (1e-2)^2        0        0]
    //          [       0         0        0        0 (1e-2)^2        0]
    //          [       0         0        0        0        0 (1e-2)^2]
    // gtsam::noiseModel::Isotropic::Sigmas -> all the diagonal elements are the same

    // ref: https://gtsam.org/doxygen/4.0.0/a03227.html

    // std::cout << __FUNCTION__ << __LINE__ << std::endl;
    // // std::cout << "priorPoseNoise_" << std::endl;
    // priorPoseNoise_->print("priorPoseNoise_");
    // // std::cout << "priorVelNoise_" << std::endl;
    // priorVelNoise_->print("priorVelNoise_");
    // // std::cout << "priorBiasNoise_" << std::endl;
    // priorBiasNoise_->print("priorBiasNoise_");
    // // std::cout << "constLidarOdomNoise_" << std::endl;
    // constLidarOdomNoise_->print("constLidarOdomNoise_");

    // Eigen::MatrixXd covMatrix = constLidarOdomNoise_->covariance();
    // std::cout << "constLidarOdomNoise_.covariance():" << std::endl;
    // std::cout << covMatrix << std::endl;

    stateCurrKf_initGuess_lo_log_ = gtsam::NavState();
    biasCurrKf_initGuess_lo_log_ = gtsam::imuBias::ConstantBias();
    stateCurrKf_initGuess_imu_log_ = gtsam::NavState();
    biasCurrKf_initGuess_imu_log_ = gtsam::imuBias::ConstantBias();

    // initialize process flag
    isProcessing_.store(false);  // std::atomic_bool isProcessing_; // for process flag avoiding racing in multi-thread
    
    // set timer logger
    timeLogger_.setLogger(get_logger());
    // set state logger
    stateLogger_.setLogger(get_logger());
    stateLogger_.createRecord("Lidar");
    stateLogger_.createRecord("Imu");
    imuRawLogger_.setLogger(get_logger());
    // set factor graph logger
    fgStateLogger_.setLogger(get_logger());
    fgStateLogger_.setCfgParams(
        config_.lidar_correction_noise,
        config_.imu_acc_noise,
        config_.imu_gyr_noise,
        config_.imu_acc_bias_noise,
        config_.imu_gyr_bias_noise,
        config_.imu_gravity,
        config_.fixed_lag,
        config_.use_lo_prior_factor_wo_between_factor,
        config_.factor_graph_init_guess_source
    );
    ineqConstLogger_.setLogger(get_logger());
    twistLogger_.setLogger(get_logger());
    dualLogger_.setLogger(get_logger());

    // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    lm_lambda_ = 0.0;

    // initialize voxel map
    voxelMap_.Initialize(config_.voxel_size, config_.voxel_max_distance, config_.max_points_per_voxel);

    // check num. of threads available for parallel threading
    RCLCPP_INFO_STREAM(this->get_logger(), "Max threads OpenMP may use: " << omp_get_max_threads());
    RCLCPP_INFO_STREAM(this->get_logger(), "Number of processors: " << omp_get_num_procs());

    if(!config_.turn_off_pcl_conversion_error_printing) pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}

// [Synchronization Policy]
// {
//     // imu: .....
//     // lidar scan beginning and ending: |
//     // We always try to process lidar-imu measurement (Keyframe) as
//     //  {.|.....|} measurement process chunk synchronized

//     // e.g.,
//     //  raw measurement: ........|......|......|......|
//     //  -- after synchronization --
//     //      1st process:       {.|......}
//     //      2nd process:              {.|......}
//     //      3rd process:                     {.|......}
    
//     // case 1
//     //     ........|.......|....
//     //  =>       {.|.......|}....   (discard earlier imus)

//     // case 2
//     //      |    ..|.......|....
//     //  =>  |      |              -> first iteration (the first process is only based on lidar)
//     //  =>       {.|.......|}     -> second iteration (from the second process, standard sync goes)

//     // case 3
//     //     .|......|.......|....   (everything works fine. just go with standard sync)
//     //    {.|......|}.......|....   

//     // case 4
//     //    {.|.    .|}.......|....   (bad imu measurement but we have to live with such case. just go with standard sync)

//     // Also need to add lidar/imu guard in fixed lag smoothing


//     // 1. see if there is more than one imu meas before timeCurrScanBeg_
//     // 2. if yes -> case 1 -> (discard earlier imus) and go standard sync onward
//     // 3. if no -> go lidar only process
//     // 4. afterward it's always the same:
//     //      1. check if earlier imu exists before the oldest scan time, 
//     //              yes: discard imus except the one right before the scan -> standard sync process
//     //               no: go to lidar only process
// }

bool Frontend::synchronizeMeasurements()
{   
    mtxCloud_.lock();
    if (config_.point_cloud_msg_timestamp == "scan_end_time"){
        timeCurrScanEnd_ = rclcpp::Time(cloudMsgBuffer_.front().header.stamp).seconds();
    }else if(config_.point_cloud_msg_timestamp == "scan_beginning_time"){
        bool isNextCloudAvailable = (cloudMsgBuffer_.size() > 1);
        if(!isNextCloudAvailable){
             RCLCPP_INFO_STREAM(get_logger(), "The first scan end time is not available, skip the process." );
            return false;
        }
        double timeNextScanBeg = rclcpp::Time(cloudMsgBuffer_[1].header.stamp).seconds();
        timeCurrScanEnd_ = timeNextScanBeg;
    }
    else{
        RCLCPP_INFO_STREAM(get_logger(), "point_cloud_msg_timestamp parameter is not given, skip the synchronization.");
        return false;
    }

    // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
    
    convertCloudMsgToPcl(cloudMsgBuffer_.front(), cloudKfWindow_);

    // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
    
    if(debug_print_point_cloud_msg_header_timestamp){
        RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
        RCLCPP_INFO_STREAM(get_logger(), "cloudMsgBuffer_.front().header.stamp: " << std::fixed << rclcpp::Time(cloudMsgBuffer_.front().header.stamp).seconds());
        RCLCPP_INFO_STREAM(get_logger(), "cloudMsgBuffer_[1].header.stamp: " << std::fixed << rclcpp::Time(cloudMsgBuffer_[1].header.stamp).seconds());
    }

    mtxCloud_.unlock();

    if (debug_print_scan_time_sync){
        RCLCPP_INFO_STREAM(get_logger(), "timePrev2ScanEnd_: " << std::fixed << timePrev2ScanEnd_);
        RCLCPP_INFO_STREAM(get_logger(), " timePrevScanEnd_: " << std::fixed << timePrevScanEnd_);
        RCLCPP_INFO_STREAM(get_logger(), " timeCurrScanBeg_: " << std::fixed << timeCurrScanBeg_);
        RCLCPP_INFO_STREAM(get_logger(), " timeCurrScanEnd_: " << std::fixed << timeCurrScanEnd_);
    }

    // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // [Lidar Only Process Case in LO]
    // if the system is just LO (factor graph is off), return true and just do lidar only process
    if(!config_.turn_on_factor_graph){
        mtxCloud_.lock();
        cloudMsgBuffer_.pop_front();
        mtxCloud_.unlock();
        return true;
    }


    // [Lidar Only Process Case in LIO]
    // if there is no imu measurement earlier than the scan beginning time, return true and do lidar only process
    // get the earliest imu measurement left in the buffer
    mtxImu_.lock();
    auto firstImu = imuMsgBuffer_.front();
    mtxImu_.unlock();
    double firstTimeImu = rclcpp::Time(firstImu.header.stamp).seconds();
    if (timeCurrScanBeg_ < firstTimeImu){
        mtxCloud_.lock();
        cloudMsgBuffer_.pop_front();
        mtxCloud_.unlock();
        return true; // imuPoseTimeLine_ is empty at this point, so the process goes with lidar only
    }

    // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // [Skip the Process (wait for more measurement) Case]
    // if there is no imu measurement later than the scan end time, the current scan is not fully covered with imu measurement yet.
    // so, return false and skip the current process, waiting for imu measurement.
    mtxImu_.lock();
    auto lastImu = imuMsgBuffer_.back();
    mtxImu_.unlock();
    double lastTimeImu = rclcpp::Time(lastImu.header.stamp).seconds();
    if (lastTimeImu < timeCurrScanEnd_){
        return false; // skip the current process
    }

    // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
    
    // Now that the current scan time range is fully covered with imu measurements.
    // But there is a chance there are too many imu measurements before the scan begin time.
    // So, we discard too early imu measurements:
    
    // [Standard Synchronization Case]
    // discard all the imu data that are too early 
    //  ......|.....|...  => .|.....|...
    for(;;){
        // take the second imu data
        mtxImu_.lock();
        auto secondImu = imuMsgBuffer_[1];      // this is safe since lastTimeImu < timeCurrScanEnd_ above
        mtxImu_.unlock();
        double secondImuTime = rclcpp::Time(secondImu.header.stamp).seconds();

        if (secondImuTime < timeCurrScanBeg_){
            mtxImu_.lock();
            imuMsgBuffer_.pop_front();
            mtxImu_.unlock();
        }
        else break;
    }

    // Just for debugging
    // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
    // RCLCPP_INFO_STREAM(get_logger(), "imuMsgBuffer_.size(): " << imuMsgBuffer_.size());
    // for(auto imu: imuMsgBuffer_){
    //     auto time = rclcpp::Time(imu.header.stamp).seconds();
    //     RCLCPP_INFO_STREAM(get_logger(), "timeImu: " << std::fixed << time);
    // }
    // RCLCPP_INFO_STREAM(get_logger(), "lastTimeImu: " << std::fixed << lastTimeImu);
    // RCLCPP_INFO_STREAM(get_logger(), "timeCurrScanEnd_: " << std::fixed << timeCurrScanEnd_);
    

    // store appropriate imu measurement data into a process container 
    //  .|.....|...  => {.|.....}|...
    for(;;){

        // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
        
        // take the second imu data
        mtxImu_.lock();
        auto secondImu = imuMsgBuffer_[1];      // this is safe since lastTimeImu < timeCurrScanEnd_ above
        mtxImu_.unlock();

        // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

        double secondImuTime = rclcpp::Time(secondImu.header.stamp).seconds();
    
        // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
    
        // if (timeCurrScanEnd_ < secondImuTime){   
        if (timeCurrScanEnd_ <= secondImuTime){   
            mtxImu_.lock();
            imuMsgKfWindow_.push_back(imuMsgBuffer_.front());
            mtxImu_.unlock();
            // imuMsgBuffer_.pop_front();   // leave the last imu in imuMsgKfWindow_ for the next process
            break;
        }

        // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

        mtxImu_.lock();
        imuMsgKfWindow_.push_back(imuMsgBuffer_.front());
        imuMsgBuffer_.pop_front();
        mtxImu_.unlock();

        // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
    }

    // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
    
    // return true as successful flag of synchronization
    mtxCloud_.lock();
    cloudMsgBuffer_.pop_front();
    mtxCloud_.unlock();
    return true;
}

void Frontend::convertCloudMsgToPcl(const sensor_msgs::msg::PointCloud2& msgIn, pcl::PointCloud<PointType>::Ptr cloudOut)
{
    if (config_.sensor == SensorType::VELODYNE){
        // check the incoming pointcloud msg contains 'time' field (timestamp for each pointcloud)
        for (const auto& f : msgIn.fields){
            if (f.name == "time") pointsTimestampAvailable_ = true;
        }

        pcl::PointCloud<VelodynePointXYZIRT>::Ptr cloudIn;
        cloudIn.reset(new pcl::PointCloud<VelodynePointXYZIRT>());
        pcl::fromROSMsg(msgIn, *cloudIn);

        cloudOut->points.resize(cloudIn->size());
        for (size_t i=0; i<cloudIn->size(); i++){
            auto &ptIn = cloudIn->points[i];
            auto &ptOut = cloudOut->points[i];
            ptOut.x = ptIn.x;
            ptOut.y = ptIn.y;
            ptOut.z = ptIn.z;
            ptOut.intensity = ptIn.intensity;
            ptOut.curvature = ptIn.time;  // !! curvature contains time[sec]
            // discard ring information
        }
    }else if (config_.sensor == SensorType::OUSTER){
        // check the incoming pointcloud msg contains 'time' field (timestamp for each pointcloud)
        for (const auto& f : msgIn.fields){
            if (f.name == "t") pointsTimestampAvailable_ = true;
        }

        pcl::PointCloud<OusterPointXYZIRT>::Ptr cloudIn;
        cloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());

        pcl::fromROSMsg(msgIn, *cloudIn);

        cloudOut->points.resize(cloudIn->size());
        for (size_t i=0; i<cloudIn->size(); i++){
            auto &ptIn = cloudIn->points[i];
            auto &ptOut = cloudOut->points[i];
            ptOut.x = ptIn.x;
            ptOut.y = ptIn.y;
            ptOut.z = ptIn.z;
            ptOut.intensity = ptIn.intensity;
            ptOut.curvature = ptIn.t * 1e-9f;  // !! curvature contains time[sec]
            // discard ring information
            // RCLCPP_ERROR_STREAM(get_logger(), "ptIn.x: " << int(config_.sensor));
        }
    }else{
        RCLCPP_ERROR_STREAM(get_logger(), "Unknown sensor type: " << int(config_.sensor));
        rclcpp::shutdown();
    }

    if(!pointsTimestampAvailable_){
        // RCLCPP_ERROR_STREAM(get_logger(), "Point cloud msg doesn't contain point time stamp.");
        // RCLCPP_ERROR_STREAM(get_logger(), " -> Point cloud deskewing would be skipped.");
    }
}

void Frontend::initializeImu()
{
    // only if setting the param 'init_imu_init_fastlio2', initialize imu gravity vector and covariance
    if (init_imu_init_fastlio2 == true){
        size_t N=1;
        Eigen::Vector3d acc, gyr;

        for (size_t i=0; i<imuMsgKfWindow_.size(); i++){
            sensor_msgs::msg::Imu& imu = imuMsgKfWindow_[i];
            acc = Eigen::Vector3d(
                imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
            gyr = Eigen::Vector3d(
                imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);

            meanAccInit_ += (acc - meanAccInit_) / N;
            meanGyrInit_ += (gyr - meanGyrInit_) / N;

            // covariance calculation (if needed)
            // cov_acc = cov_acc * (N - 1.0) / N + (acc - mean_acc).cwiseProduct(acc - mean_acc) * (N - 1.0) / (N * N);
            // cov_gyr = cov_gyr * (N - 1.0) / N + (gyr - mean_gyr).cwiseProduct(gyr - mean_gyr) * (N - 1.0) / (N * N);
            N++;
        }

        gravInit_ = (-meanAccInit_ / meanAccInit_.norm() * config_.imu_gravity);
        
        RCLCPP_INFO_STREAM(get_logger(), "meanAccInit_ = " << meanAccInit_);
        RCLCPP_INFO_STREAM(get_logger(), "meanGyrInit_ = " << meanGyrInit_);
        RCLCPP_INFO_STREAM(get_logger(), "gravInit_ = " << gravInit_);
    }
}

void Frontend::propagateImu()
{
    // perform propagate Imu only after the state is initialized since it needs correct velocity
    gtsam::NavState state = statePrevKf_;
    gtsam::imuBias::ConstantBias bias = biasPrevKf_;

    if (imuMsgKfWindow_.empty()){
        RCLCPP_ERROR_STREAM(this->get_logger(), "Keyframe window imu measurements is none. Skip propagateImu().");
        return;
    }

    double timeFirstImu = rclcpp::Time(imuMsgKfWindow_[0].header.stamp).seconds();
    imuPoseTimeline_.insert(std::make_pair(timeFirstImu, state));

    // for covariance propagation via imu
    // get the corrent state covariance
    gtsam::Matrix9 P0 = gtsam::Matrix9::Identity();
    // Q = batchFixedLagSmoother_.marginalCovariance(X(key_));
    if (config_.ekf_use_const_prior_cov){
        // Q = fill all the diagonal elements with some predefined const.
        P0 = P0 * config_.ekf_const_diagonal_elem_prior_cov;
    }else{
        // add marginal variable as the member variable
        // extract marginalized covariance of the latest state
        P0 = stateCovPrevKf_;
    }
    gtsam::InvariantEKF<gtsam::NavState> ekf(statePrevKf_, P0);
    gtsam::Matrix9 P = P0;


    for (size_t i=0; i<imuMsgKfWindow_.size()-1; i++){
        // reset imuPropagator_ for the next two consecutive imu measurements integration
        imuPropagator_->resetIntegrationAndSetBias(bias);

        sensor_msgs::msg::Imu& imuPrev = imuMsgKfWindow_[i];
        sensor_msgs::msg::Imu& imuNext = imuMsgKfWindow_[i+1];

        double tPrev = rclcpp::Time(imuPrev.header.stamp).seconds();
        double tNext = rclcpp::Time(imuNext.header.stamp).seconds();

        // collect the imu data input in imu preintegration in GTSAM  
        gtsam::Vector3 vecAcc = gtsam::Vector3(
            imuPrev.linear_acceleration.x, imuPrev.linear_acceleration.y, imuPrev.linear_acceleration.z);
        gtsam::Vector3 vecGyr = gtsam::Vector3(
            imuPrev.angular_velocity.x, imuPrev.angular_velocity.y, imuPrev.angular_velocity.z);

        // // mid-point integration
        // gtsam::Vector3 vecAcc = gtsam::Vector3(
        //     (imuPrev.linear_acceleration.x + imuNext.linear_acceleration.x)/2, 
        //     (imuPrev.linear_acceleration.y + imuNext.linear_acceleration.y)/2,
        //     (imuPrev.linear_acceleration.z + imuNext.linear_acceleration.z)/2
        // );
        // gtsam::Vector3 vecGyr = gtsam::Vector3(
        //     (imuPrev.angular_velocity.x + imuNext.angular_velocity.x)/2, 
        //     (imuPrev.angular_velocity.y + imuNext.angular_velocity.y)/2, 
        //     (imuPrev.angular_velocity.z + imuNext.angular_velocity.z)/2
        // );

        double dt = tNext - tPrev;

        // if the param is set, calibrate linear acceleration based on the gravity as Fast-lio does
        if (imu_isotropic_scale_calibration_to_G){
            vecAcc = vecAcc * config_.imu_gravity / meanAccInit_.norm();
        }

        imuPropagator_->integrateMeasurement(vecAcc, vecGyr, dt);
        imuIntegrator_->integrateMeasurement(vecAcc, vecGyr, dt);

        state = imuPropagator_->predict(state, bias);
        imuPoseTimeline_.insert(std::make_pair(tNext, state));

        // for debugging
        if (debug_print_imu_forward_propagation_state){
            RCLCPP_INFO_STREAM(get_logger(), "vecAcc: " << vecAcc[0] << ", " << vecAcc[1] << ", " << vecAcc[2]);
            RCLCPP_INFO_STREAM(get_logger(), "vecGyr: " << vecGyr[0] << ", " << vecGyr[1] << ", " << vecGyr[2]);
            RCLCPP_INFO_STREAM(get_logger(), "dt: " << dt);
            RCLCPP_INFO_STREAM(get_logger(), "state: " << state);
            RCLCPP_INFO_STREAM(get_logger(), "bias: " << bias);
        }
        // use gtsam::NavState::update() -> for forward integration: https://gtsam.org/doxygen/4.0.0/a03507.html#a5494db1f41c8a61acc2d63c32b9adc31

        if (config_.ekf_imu_covariance_prop_on){
            // get covariance matrix with the current key from the factor graph
            // gtsam::Matrix9 Q = gtsam::Matrix9::Identity();
            // // Q = batchFixedLagSmoother_.marginalCovariance(X(key_));
            // if (config_.ekf_use_const_prior_cov){
            //     // Q = fill all the diagonal elements with some predefined const.
            //     Q = Q * config_.ekf_const_diagonal_elem_prior_cov;
            // }else{
            //     // add marginal variable as the member variable
            //     // extract marginalized covariance of the latest state
            //     Q = stateCovPrevKf_;
            // }
            
            // RCLCPP_INFO_STREAM(get_logger(), "Before update covariance:");
            // RCLCPP_INFO_STREAM(get_logger(), P);

            // get noise matrix
            gtsam::Matrix9 Q = imuPropagator_->preintMeasCov();
            // preintMeasCov() -> COVARIANCE OF: [PreintROTATION PreintPOSITION PreintVELOCITY].

            // prpagate imu
            gtsam::Vector9 xi;
            xi << vecGyr, gtsam::Vector3::Zero(), vecAcc;
            ekf.predict(xi, dt, Q);

            // RCLCPP_INFO_STREAM(get_logger(), "After update covariance:");
            // RCLCPP_INFO_STREAM(get_logger(), P);

            // stateCovPredCurrKf_imu_ = ekf.covariance();
            P = ekf.covariance();
        }
        // ref: /home/ysugano/code_review/gtsam_4-3/examples/IEKF_NavstateExample.cpp
        // ref: /home/ysugano/code_review/gtsam_4-3/gtsam/navigation/InvariantEKF.h

        imuRawLogger_.recordImuRaw(timeCurrScanBeg_, tPrev, tNext, dt, vecAcc, vecGyr);
    }

    stateCovPredCurrKf_imu_ = P;

    // for debugging
    if(debug_print_imu_pose_timeline){
        RCLCPP_INFO_STREAM(get_logger(), "Print imuPoseTimeline_: ");
        RCLCPP_INFO_STREAM(get_logger(), "imuPoseTimeline_.size(): " << imuPoseTimeline_.size());
        for (const auto& [key, value] : imuPoseTimeline_){
            RCLCPP_INFO_STREAM(get_logger(), "time:"  << std::fixed << key << ", imu state: " << value);
        }
        RCLCPP_INFO_STREAM(get_logger(), "Print imuPoseTimeline_ -------- END ");
    }

    if(debug_print_fg_imuIntegrator_right_after_imu_propagation){
        std::cout << __FUNCTION__ << __LINE__ << std::endl;
        // std::cout << "imuPoseTimeline_.size"
        const gtsam::PreintegratedImuMeasurements &preintImu = dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(*imuIntegrator_);
        preintImu.print("debug_print_fg_imuIntegrator_right_after_imu_propagation");
    }
}

void Frontend::transformPointCloudInImuFrame()
{
    for (auto &point: cloudKfWindow_->points){
        Eigen::Vector3d t_l_pt(point.x, point.y, point.z);
        Eigen::Vector3d t_b_pt;
        t_b_pt = q_IMU_LIDAR * t_l_pt + t_IMU_LIDAR;

        point.x = t_b_pt.x();
        point.y = t_b_pt.y();
        point.z = t_b_pt.z();
    }
}

// void Frontend::makeInitialGuess()
void Frontend::makeInitialGuessLO()
{
    Eigen::Quaterniond q_bKfWdBeg_bKfWdEnd;
    Eigen::Vector3d t_bKfWdBeg_bKfWdEnd;
    
    // if enable the initial guess for LO based on imu, set a transformation
    if (config_.initial_guess_source == "constant velocity" || imuPoseTimeline_.empty()){
        // take the previous relative motion
        q_bKfWdBeg_bKfWdEnd = q_bPrev2Kf_bPrevKf_;
        t_bKfWdBeg_bKfWdEnd = t_bPrev2Kf_bPrevKf_;
    }else if (config_.initial_guess_source == "imu"){
        Eigen::Quaterniond q_w_bKfWindowBeg;
        Eigen::Vector3d t_w_bKfWindowBeg;
        getEigenFromGtsam(imuPoseTimeline_.begin()->second, q_w_bKfWindowBeg, t_w_bKfWindowBeg);

        Eigen::Quaterniond q_w_bKfWindowEnd_;
        Eigen::Vector3d t_w_bKfWindowEnd_;
        getEigenFromGtsam(std::prev(imuPoseTimeline_.end())->second, q_w_bKfWindowEnd_, t_w_bKfWindowEnd_);
        
        q_bKfWdBeg_bKfWdEnd = (q_w_bKfWindowBeg.inverse() * q_w_bKfWindowEnd_).normalized();
        t_bKfWdBeg_bKfWdEnd = q_w_bKfWindowBeg.inverse() * (t_w_bKfWindowEnd_ - t_w_bKfWindowBeg);
        // t_w_bKfWindowEnd_ - t_w_bKfWindowBeg : translation of beg->end in world frame
        // q_w_bKfWindowBeg.inverse() * {above} = q_bKfWindowBeg_w * {above}
        //                                      = translation of beg->end in beg frame (OK)
    }else{
        q_bKfWdBeg_bKfWdEnd = Eigen::Quaterniond::Identity();
        t_bKfWdBeg_bKfWdEnd = Eigen::Vector3d::Zero();
    }

    // // if enable the initial guess for LO based on imu, set a transformation
    // if (config_.initial_guess_source == "imu" && !imuPoseTimeline_.empty()){
    //     Eigen::Quaterniond q_w_bKfWindowBeg;
    //     Eigen::Vector3d t_w_bKfWindowBeg;
    //     getEigenFromGtsam(imuPoseTimeline_.begin()->second, q_w_bKfWindowBeg, t_w_bKfWindowBeg);

    //     Eigen::Quaterniond q_w_bKfWindowEnd_;
    //     Eigen::Vector3d t_w_bKfWindowEnd_;
    //     getEigenFromGtsam(std::prev(imuPoseTimeline_.end())->second, q_w_bKfWindowEnd_, t_w_bKfWindowEnd_);
        
    //     q_bKfWdBeg_bKfWdEnd = (q_w_bKfWindowBeg.inverse() * q_w_bKfWindowEnd_).normalized();
    //     t_bKfWdBeg_bKfWdEnd = q_w_bKfWindowBeg.inverse() * (t_w_bKfWindowEnd_ - t_w_bKfWindowBeg);
    //     // t_w_bKfWindowEnd_ - t_w_bKfWindowBeg : translation of beg->end in world frame
    //     // q_w_bKfWindowBeg.inverse() * {above} = q_bKfWindowBeg_w * {above}
    //     //                                      = translation of beg->end in beg frame (OK)
    // }else if(config_.initial_guess_source == "constant velocity"){ 
    //     // take the previous relative motion
    //     q_bKfWdBeg_bKfWdEnd = q_bPrev2Kf_bPrevKf_;
    //     t_bKfWdBeg_bKfWdEnd = t_bPrev2Kf_bPrevKf_;
    // }else{
    //     q_bKfWdBeg_bKfWdEnd = Eigen::Quaterniond::Identity();
    //     t_bKfWdBeg_bKfWdEnd = Eigen::Vector3d::Zero();
    // }

    q_bPrevKf_bCurrKf_initGuess_ = q_bKfWdBeg_bKfWdEnd;
    t_bPrevKf_bCurrKf_initGuess_ = t_bKfWdBeg_bKfWdEnd;

    // for debugging
    if(debug_print_initial_guess_lo){
        RCLCPP_INFO_STREAM(get_logger(), "t_w_bPrevKf_: " << t_w_bPrevKf_.transpose());
        RCLCPP_INFO_STREAM(get_logger(), "q_w_bPrevKf_: " << q_w_bPrevKf_.coeffs().transpose());
        RCLCPP_INFO_STREAM(get_logger(), "t_bPrevKf_bCurrKf_initGuess_: " << t_bPrevKf_bCurrKf_initGuess_.transpose());
        RCLCPP_INFO_STREAM(get_logger(), "q_bPrevKf_bCurrKf_initGuess_: " << q_bPrevKf_bCurrKf_initGuess_.coeffs().transpose());
    }
}

void Frontend::deskewPointCloud()
{   
    Eigen::Quaterniond q_w_bKfWindowEnd_;
    Eigen::Vector3d t_w_bKfWindowEnd_;

    // if imu prediction is NOT availble or motion compensation "constant velocity" is set,
    //  AND also the previous velocity is available, then deskew point cloud based on the constant velocity model
    if (pointsTimestampAvailable_
        && (imuPoseTimeline_.empty() || config_.motion_compensation_source == "constant velocity") 
        && (!t_bPrev2Kf_bPrevKf_.isApprox(Eigen::Vector3d::Zero(), 1e-9) && !q_bPrev2Kf_bPrevKf_.isApprox(Eigen::Quaterniond::Identity(), 1e-9))){ // [TODO] velocity should be drawn from NavState statePrevKf_
        RCLCPP_INFO_STREAM(get_logger(), "Deskew point cloud based on constant velocity model.");
        // RCLCPP_INFO_STREAM(get_logger(), "t_bPrev2Kf_bPrevKf_: " << t_bPrev2Kf_bPrevKf_.transpose());
        // RCLCPP_INFO_STREAM(get_logger(), "q_bPrev2Kf_bPrevKf_: " << q_bPrev2Kf_bPrevKf_);

        // follows kiss-icp way
        const double min_time = timeCurrScanBeg_ + cloudKfWindow_->points.front().curvature;
        const double max_time = timeCurrScanBeg_ + cloudKfWindow_->points.back().curvature;
        if (max_time - min_time < TIME_EPS)
            RCLCPP_INFO_STREAM(get_logger(), "Timestamp for the first and end point are the same. Point deskewing would fail.");

        const Sophus::SE3d relative_motion(q_bPrev2Kf_bPrevKf_.toRotationMatrix(), t_bPrev2Kf_bPrevKf_);
        const auto &xi = relative_motion.log();

        for (const auto &point: cloudKfWindow_->points){
            Eigen::Vector3d bPtTime_pt(point.x, point.y, point.z);
            double pointTime = timeCurrScanBeg_ + point.curvature; // !! curvature contains time[sec] !!
            const auto stamp = (pointTime - min_time) / (max_time - min_time);
            auto T_bKfWindowEnd_bPtTime = Sophus::SE3d::exp((stamp - 1.0) * xi);
        
            Eigen::Vector3d bKfWindowEnd_pt;
            bKfWindowEnd_pt = T_bKfWindowEnd_bPtTime * bPtTime_pt;

            // for debugging
            if(debug_print_deskew_pointcloud){
                RCLCPP_INFO_STREAM(get_logger(), "START DESKEWING - pt: ( " << point.x << ", " << point.y << ", " << point.z << " )");
                RCLCPP_INFO_STREAM(get_logger(), "t_bKfWindowEnd_bPtTime = " << T_bKfWindowEnd_bPtTime.translation().transpose());
                RCLCPP_INFO_STREAM(get_logger(), "q_bKfWindowEnd_bPtTime = " << Eigen::Quaterniond(T_bKfWindowEnd_bPtTime.rotationMatrix()));
                RCLCPP_INFO_STREAM(get_logger(), "bKfWindowEnd_pt = " << bKfWindowEnd_pt.transpose());
                RCLCPP_INFO_STREAM(get_logger(), "END DESKEWING   - pt: ( " << bKfWindowEnd_pt.x() << ", " << bKfWindowEnd_pt.y() << ", " << bKfWindowEnd_pt.z() << " )");
            }

            PointType ptDeskewed;
            ptDeskewed.x = bKfWindowEnd_pt.x();
            ptDeskewed.y = bKfWindowEnd_pt.y();
            ptDeskewed.z = bKfWindowEnd_pt.z();

            if(config_.enable_min_range_filter){
                const double ptDisSq = ptDeskewed.x*ptDeskewed.x + ptDeskewed.y*ptDeskewed.y + ptDeskewed.z*ptDeskewed.z;
                const double minThrDisSq = config_.lidar_min_range*config_.lidar_min_range;
                if(ptDisSq > minThrDisSq){
                    cloudScanCurr_->points.push_back(ptDeskewed);
                }
            }else{
                cloudScanCurr_->points.push_back(ptDeskewed);
            }
            // eventually get away from pcl::pointcloud cloudScanCurr_ and move to std::vector<Eigen:Vector3d> for the points container
        }
    }
    // if imu prediction IS availble or motion compensation "imu" is set,
    //  then deskew point cloud based on the imu prediction
    else if(pointsTimestampAvailable_ && config_.motion_compensation_source == "imu" && !imuPoseTimeline_.empty()){
        getEigenFromGtsam(std::prev(imuPoseTimeline_.end())->second, q_w_bKfWindowEnd_, t_w_bKfWindowEnd_);
        RCLCPP_INFO_STREAM(get_logger(), "Deskew point cloud based on imu forward propagation.");

        if(debug_print_deskew_pointcloud)
            std::cout << "std::prev(imuPoseTimeline_.end()) -> second = " << std::prev(imuPoseTimeline_.end())->second << std::endl;

        // [TODO] can be multi-threaded by replacing push_back
        for (auto &point: cloudKfWindow_->points){
            double pointTime = timeCurrScanBeg_ + point.curvature; // !! curvature contains time[sec] !!

            // get an imu pose interpolated based on the point time
            Eigen::Quaterniond q_w_bPtTime;
            Eigen::Vector3d t_w_bPtTime;
            getImuPoseAtPointMeasurementTime(pointTime, q_w_bPtTime, t_w_bPtTime);

            Eigen::Quaterniond q_bKfWindowEnd_bPtTime;
            Eigen::Vector3d t_bKfWindowEnd_bPtTime;
            q_bKfWindowEnd_bPtTime = (q_w_bKfWindowEnd_.inverse() * q_w_bPtTime).normalized();
            t_bKfWindowEnd_bPtTime = q_w_bKfWindowEnd_.inverse() * (t_w_bPtTime - t_w_bKfWindowEnd_);

            Eigen::Vector3d t_bPtTime_pt(point.x, point.y, point.z);
            Eigen::Vector3d bKfWindowEnd_pt;
            bKfWindowEnd_pt = q_bKfWindowEnd_bPtTime * t_bPtTime_pt + t_bKfWindowEnd_bPtTime;

            // for debugging
            if(debug_print_deskew_pointcloud){
                RCLCPP_INFO_STREAM(get_logger(), "START DESKEWING - pt: ( " << point.x << ", " << point.y << ", " << point.z << " )");
                RCLCPP_INFO_STREAM(get_logger(), "t_w_bPtTime = " << t_w_bPtTime.transpose());
                RCLCPP_INFO_STREAM(get_logger(), "q_w_bPtTime = " << q_w_bPtTime.coeffs().transpose());
                RCLCPP_INFO_STREAM(get_logger(), "t_w_bKfWindowEnd_ = " << t_w_bKfWindowEnd_.transpose());
                RCLCPP_INFO_STREAM(get_logger(), "q_w_bKfWindowEnd_ = " << q_w_bKfWindowEnd_.coeffs().transpose());
                RCLCPP_INFO_STREAM(get_logger(), "t_bKfWindowEnd_bPtTime = " << t_bKfWindowEnd_bPtTime.transpose());
                RCLCPP_INFO_STREAM(get_logger(), "q_bKfWindowEnd_bPtTime = " << q_bKfWindowEnd_bPtTime.coeffs().transpose());
                RCLCPP_INFO_STREAM(get_logger(), "bKfWindowEnd_pt = " << bKfWindowEnd_pt.transpose());
                RCLCPP_INFO_STREAM(get_logger(), "END DESKEWING   - pt: ( " << bKfWindowEnd_pt.x() << ", " << bKfWindowEnd_pt.y() << ", " << bKfWindowEnd_pt.z() << " )");
            }

            PointType ptDeskewed;
            ptDeskewed.x = bKfWindowEnd_pt.x();
            ptDeskewed.y = bKfWindowEnd_pt.y();
            ptDeskewed.z = bKfWindowEnd_pt.z();
            ptDeskewed.curvature = point.curvature;
            ptDeskewed.intensity = point.intensity;

            if(config_.enable_min_range_filter){
                const double ptDisSq = ptDeskewed.x*ptDeskewed.x + ptDeskewed.y*ptDeskewed.y + ptDeskewed.z*ptDeskewed.z;
                const double minThrDisSq = config_.lidar_min_range*config_.lidar_min_range;
                if(ptDisSq > minThrDisSq){
                    cloudScanCurr_->points.push_back(ptDeskewed);
                }
            }else{
                cloudScanCurr_->points.push_back(ptDeskewed);
            }
            // eventually get away from pcl::pointcloud cloudScanCurr_ and move to std::vector<Eigen:Vector3d> for the points container
        }
    }else{
        RCLCPP_INFO_STREAM(get_logger(), "Motion Compensation is not well-defined. Skip point cloud deskewing.");
        RCLCPP_INFO_STREAM(get_logger(), " -> Just process the incoming point cloud without deskewing.");
        for (auto &point: cloudKfWindow_->points){
            if(config_.enable_min_range_filter){
                const double ptDisSq = point.x*point.x + point.y*point.y + point.z*point.z;
                const double minThrDisSq = config_.lidar_min_range*config_.lidar_min_range;
                if(ptDisSq > minThrDisSq){
                    cloudScanCurr_->points.push_back(point);
                }
            }else{
                cloudScanCurr_->points.push_back(point);
            }
            // eventually get away from pcl::pointcloud cloudScanCurr_ and move to std::vector<Eigen:Vector3d> for the points container
        }
    }
}

void Frontend::getImuPoseAtPointMeasurementTime(double pointTime, Eigen::Quaterniond& qOut, Eigen::Vector3d& tOut)
{
    // get iterator for imu time-pose pair right after the point measurement time
    auto itPoseImuAftPtT = imuPoseTimeline_.upper_bound(pointTime); // iterator for std::map imuPoseTimeline_
    // get iterator for imu time-pose pair right before the point measurement time
    auto itPoseImuBefPtT = std::prev(itPoseImuAftPtT); // iterator for std::map imuPoseTimeline_

    double timeRatio = (pointTime - itPoseImuBefPtT->first) / (itPoseImuAftPtT->first - itPoseImuBefPtT->first);

    Eigen::Quaterniond q_w_bAftPtTime;
    Eigen::Vector3d t_w_bAftPtTime;
    getEigenFromGtsam(itPoseImuAftPtT->second, q_w_bAftPtTime, t_w_bAftPtTime);

    Eigen::Quaterniond q_w_bBefPtTime;
    Eigen::Vector3d t_w_bBefPtTime;
    getEigenFromGtsam(itPoseImuBefPtT->second, q_w_bBefPtTime, t_w_bBefPtTime);

    Eigen::Quaterniond q_w_bInterpolated;
    Eigen::Vector3d t_w_bInterpolated;
    q_w_bInterpolated = q_w_bBefPtTime.slerp(timeRatio, q_w_bAftPtTime).normalized();
    t_w_bInterpolated = (1-timeRatio) * t_w_bBefPtTime + timeRatio * t_w_bAftPtTime;
    qOut = q_w_bInterpolated;
    tOut = t_w_bInterpolated;

    // for debugging
    if (debug_print_get_imu_pose_at_measurement_time){
        RCLCPP_INFO_STREAM(get_logger(), "pointTime:     " << pointTime);
        RCLCPP_INFO_STREAM(get_logger(), "PoseImuBefPtT: " << itPoseImuBefPtT->first);
        RCLCPP_INFO_STREAM(get_logger(), "PoseImuAftPtT: " << itPoseImuAftPtT->first);
        RCLCPP_INFO_STREAM(get_logger(), "timeRatio:     " << timeRatio);
        RCLCPP_INFO_STREAM(get_logger(), "t_w_bAftPtTime: " << t_w_bAftPtTime.transpose());
        RCLCPP_INFO_STREAM(get_logger(), "q_w_bAftPtTime: " << q_w_bAftPtTime.coeffs().transpose());
        RCLCPP_INFO_STREAM(get_logger(), "t_w_bBefPtTime: " << t_w_bBefPtTime.transpose());
        RCLCPP_INFO_STREAM(get_logger(), "q_w_bBefPtTime: " << q_w_bBefPtTime.coeffs().transpose());
        RCLCPP_INFO_STREAM(get_logger(), "t_w_bInterpolated: " << t_w_bInterpolated.transpose());
        RCLCPP_INFO_STREAM(get_logger(), "q_w_bInterpolated: " << q_w_bInterpolated.coeffs().transpose());
    }
}

// old initializeCloudMap()
// void Frontend::initializeCloudMap()
// {
// // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

//     RCLCPP_INFO_STREAM(this->get_logger(), "Initializing a global cloud map.");
//     RCLCPP_INFO_STREAM(this->get_logger(), "cloudScanCurr_->points.size() = " << cloudScanCurr_->points.size());

//     pcl::PointCloud<PointType>::Ptr cloudInW;
//     cloudInW.reset(new pcl::PointCloud<PointType>());
//     // convert point cloud to the world frame 
//     for (auto& pt: cloudScanCurr_->points){
//         PointType ptInW;
//         transformPointToWorldFrame(pt, ptInW);
//         cloudInW->points.push_back(ptInW);
//     }

//     if(config_.use_voxel_map){
//         // TODO //
//         // set cloudInW to voxelMap_  
//         // should we downsample the cloud?
//         // frame_downsample = VoxelDownsample(cloudInW, config_.voxel_size);
//         // voxelMap_.Update(frame_downsample, new_pose);
//         std::vector<Eigen::Vector3d> cloudVecEig;
//         for (auto& pt: cloudInW->points){
//             Eigen::Vector3d ptEig(pt.x, pt.y, pt.z);
//             cloudVecEig.emplace_back(ptEig);
//         }

//         std::vector<Eigen::Vector3d> cloudVecEigDs;

//         // voxel downsample on 
//         if (config_.turn_on_voxel_downsample){
//             // cloudVecEigDs = VoxelDownsample(cloudVecEig, config_.voxel_size * 0.5);
//             cloudVecEigDs = VoxelDownsample(cloudVecEig, config_.voxel_size * config_.voxel_downsample_resolution_mapping_to_icpsource);
//         }else{ // voxel downsample off
//             cloudVecEigDs = cloudVecEig;
//         }

//         // update the map
//         if (debug_print_num_point_in_voxel_map){
//             RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__ << " before update: voxelMap.size() = " << voxelMap_.Pointcloud().size());
//             RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__ << "     added cloudVecEigDs.size() = " << cloudVecEigDs.size());
//         }
//         voxelMap_.Update(cloudVecEigDs, t_w_bCurrKf_);

//         if (debug_print_num_point_in_voxel_map){
//             RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__ << " after update: voxelMap.size() = " << voxelMap_.Pointcloud().size());
//         }

//         if(debug_dump_log_file_voxel_ids)
//             voxelMap_.DumpVoxelCorrdinates(timeCurrScanBeg_);
//     }else{
//         // We should downsample point cloud here
//         cloudFramesMapGlobal_.push_back(cloudInW);
//     }
// }

bool Frontend::initializeCloudMap(){
    // when initializing the global cloud map
    //  0. Check at least two point cloud msgs are available to safely extract the current scan end time
    //  1. update time timeCurrScanEnd to the end time of the first scan -> this becomes timePrevScanEnd at next round
    //  2. take the earliest point cloud from buffer and remove it from the buffer
    //  3. just put it into the map without transforming or deskewing it
    //      -> the first cloud should be at the world origin -> no need for transforming
    //      -> for the first process, velocity is not available so we can not deskew point cloud

    // 0. Check at least two point cloud msgs are available to safely extract the current scan end time
    if(config_.point_cloud_msg_timestamp == "scan_beginning_time"){
        mtxCloud_.lock();
        bool isNextCloudAvailable = (cloudMsgBuffer_.size() > 1);
        mtxCloud_.unlock();
        // [TODO] This probebly should be only in the case of config_.point_cloud_msg_timestamp == "scan_beginning_time". need to be fixed
        if(!isNextCloudAvailable){
            RCLCPP_INFO_STREAM(get_logger(), "The first scan end time is not available, skip the process." );
            return false;
        }
    }

    //  1. update time timeCurrScanEnd to the end time of the first scan -> this becomes timePrevScanEnd at next round
    mtxCloud_.lock();

    if (config_.point_cloud_msg_timestamp == "scan_end_time"){
        timeCurrScanEnd_ = rclcpp::Time(cloudMsgBuffer_.front().header.stamp).seconds();
    }else if(config_.point_cloud_msg_timestamp == "scan_beginning_time"){
        timeCurrScanBeg_ = rclcpp::Time(cloudMsgBuffer_.front().header.stamp).seconds();
        double timeNextScanBeg = rclcpp::Time(cloudMsgBuffer_[1].header.stamp).seconds();
        timeCurrScanEnd_ = timeNextScanBeg;
    }
    else{
        RCLCPP_INFO_STREAM(get_logger(), "point_cloud_msg_timestamp parameter is not given, skip the map initialization.");
        return false;
    }
    // RCLCPP_INFO_STREAM(get_logger(), "timeCurrScanBeg_: "  << std::fixed << timeCurrScanBeg_ );
    // RCLCPP_INFO_STREAM(get_logger(), "timeCurrScanEnd_: "  << std::fixed << timeCurrScanEnd_ );

    // 2. take the earliest point cloud from buffer and remove it from the buffer
    pcl::PointCloud<PointType>::Ptr cloud;
    cloud.reset(new pcl::PointCloud<PointType>());
    convertCloudMsgToPcl(cloudMsgBuffer_.front(), cloud); // [TODO] this can be quicker to chech just the last point cloud time (cloudKfWindow_->points.back().curvature)
    cloudMsgBuffer_.pop_front();
    mtxCloud_.unlock();
    
    // set the initial cloud to map
    if(config_.use_voxel_map){
        std::vector<Eigen::Vector3d> cloudVecEig;
        for (auto& pt: cloud->points){
            Eigen::Vector3d ptEig(pt.x, pt.y, pt.z);
            cloudVecEig.emplace_back(ptEig);
        }

        std::vector<Eigen::Vector3d> cloudVecEigDs;

        // voxel downsample on 
        if (config_.turn_on_voxel_downsample){
            // cloudVecEigDs = VoxelDownsample(cloudVecEig, config_.voxel_size * 0.5);
            cloudVecEigDs = VoxelDownsample(cloudVecEig, config_.voxel_size * config_.voxel_downsample_resolution_mapping_to_icpsource);
        }else{ // voxel downsample off
            cloudVecEigDs = cloudVecEig;
        }

        // update the map
        if (debug_print_num_point_in_voxel_map){
            RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__ << " before update: voxelMap.size() = " << voxelMap_.Pointcloud().size());
            RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__ << "     added cloudVecEigDs.size() = " << cloudVecEigDs.size());
        }
        voxelMap_.Update(cloudVecEigDs, t_w_bCurrKf_);

        if (debug_print_num_point_in_voxel_map){
            RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__ << " after update: voxelMap.size() = " << voxelMap_.Pointcloud().size());
        }

        if(debug_dump_log_file_voxel_ids)
            // voxelMap_.DumpVoxelCorrdinates(timeCurrScanBeg_);
            voxelMap_.DumpVoxelCorrdinates(timeCurrScanEnd_);
    }else{
        // We should downsample point cloud here
        cloudFramesMapGlobal_.push_back(cloud);
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Initializing a global cloud map.");
    RCLCPP_INFO_STREAM(this->get_logger(), "cloud->points.size() = " << cloud->points.size());
    return true;
}

void Frontend::transformPointToWorldFrame(const PointType&  pi, PointType& po) 
{
    // for debugging
    if (debug_print_get_imu_pose_at_measurement_time){
        RCLCPP_INFO_STREAM(this->get_logger(), "Before transformation---");
        RCLCPP_INFO_STREAM(this->get_logger(), "pi:");
        RCLCPP_INFO_STREAM(this->get_logger(), "  x: " << pi.x);
        RCLCPP_INFO_STREAM(this->get_logger(), "  y: " << pi.y);
        RCLCPP_INFO_STREAM(this->get_logger(), "  z: " << pi.z);
        RCLCPP_INFO_STREAM(this->get_logger(), "po:");
        RCLCPP_INFO_STREAM(this->get_logger(), "  x: " << po.x);
        RCLCPP_INFO_STREAM(this->get_logger(), "  y: " << po.y);
        RCLCPP_INFO_STREAM(this->get_logger(), "  z: " << po.z);

        RCLCPP_INFO_STREAM(this->get_logger(), "q_w_bCurrKf_ = " << q_w_bCurrKf_.coeffs().transpose());
        RCLCPP_INFO_STREAM(this->get_logger(), "t_w_bCurrKf_ = " << t_w_bCurrKf_.transpose());
    }

    Eigen::Vector3d ptIn(pi.x, pi.y, pi.z);
    Eigen::Vector3d ptOut = q_w_bCurrKf_ * ptIn + t_w_bCurrKf_;

    po.x = ptOut.x();
    po.y = ptOut.y();
    po.z = ptOut.z();
    po.intensity = pi.intensity;
    po.curvature = pi.curvature;

    // for debugging
    if (debug_print_get_imu_pose_at_measurement_time){
        RCLCPP_INFO_STREAM(this->get_logger(), "After transformation---");
        RCLCPP_INFO_STREAM(this->get_logger(), "pi:");
        RCLCPP_INFO_STREAM(this->get_logger(), "  x: " << pi.x);
        RCLCPP_INFO_STREAM(this->get_logger(), "  y: " << pi.y);
        RCLCPP_INFO_STREAM(this->get_logger(), "  z: " << pi.z);
        RCLCPP_INFO_STREAM(this->get_logger(), "po:");
        RCLCPP_INFO_STREAM(this->get_logger(), "  x: " << po.x);
        RCLCPP_INFO_STREAM(this->get_logger(), "  y: " << po.y);
        RCLCPP_INFO_STREAM(this->get_logger(), "  z: " << po.z);
    }
}

// void Frontend::setInitialPose()
void Frontend::setInitialPoseLO()
{
    // q_w_bPrevKf_.normalize();
    
    // Eigen::Vector3d t_w_bCurrKf_initGuess = q_w_bPrevKf_ * t_bPrevKf_bCurrKf_initGuess_ + t_w_bPrevKf_;
    // Eigen::Quaterniond q_w_bCurrKf_initGuess = (q_w_bPrevKf_ * q_bPrevKf_bCurrKf_initGuess_).normalized();

    // // for debugging
    // if(debug_print_initial_guess_lo){
    //     RCLCPP_INFO_STREAM(this->get_logger(), "t_w_bCurrKf_initGuess: " << t_w_bCurrKf_initGuess.transpose());
    //     RCLCPP_INFO_STREAM(this->get_logger(), "q_w_bCurrKf_initGuess: " << q_w_bCurrKf_initGuess.coeffs().transpose());
    // }

    // q_w_bCurrKf_ = q_w_bCurrKf_initGuess;
    // t_w_bCurrKf_ = t_w_bCurrKf_initGuess;
}

void Frontend::buildLocalMap() 
{
    cloudMapLocal_->clear();

    if(config_.use_voxel_map){
        // the local map is exactly voxelMap_, so we don't have to do anything here (It's already build in the process)
        // in solveLeastSquares(), we need to use voxelMap_ for nn search
    }else{
        const size_t nCloudInMap = cloudFramesMapGlobal_.size();
        // const size_t maxCloudToTake = 20;
        const size_t maxCloudToTake = config_.max_cloud_frame_for_local_map;
        const size_t nCloudToTake = std::min(nCloudInMap, maxCloudToTake);

        if(!config_.build_local_map_from_all_global_map){ // if extracting all the points to build a local map in the global map
            for (size_t i=0; i<nCloudToTake; ++i) {
                *cloudMapLocal_ += *cloudFramesMapGlobal_[(nCloudInMap-1) - i]; // idx: (nCloudInMap-1), (nCloudInMap-1)-1, ...
            }
        }else{
            for (size_t i=0; i<nCloudInMap; ++i){  // if extracting some of the recent points to build a local map in the global map
                *cloudMapLocal_ += *cloudFramesMapGlobal_[i]; // idx: (nCloudInMap-1), (nCloudInMap-1)-1, ...
            } 
        }

        // for debugging
        if (debug_print_cloud_map_size){
            RCLCPP_WARN_STREAM(get_logger(), "cloudFramesMapGlobal_.size() = " << cloudFramesMapGlobal_.size());
            RCLCPP_WARN_STREAM(get_logger(), "cloudMapLocal_->points.size() = " << cloudMapLocal_->points.size());
        }
    }
}

void Frontend::downsampleCloud() 
{
    // for debugging
    if (debug_print_num_downsampled_point)
        RCLCPP_WARN_STREAM(get_logger(), "before downsample num.point = " << cloudScanCurr_->points.size());

    if(config_.use_voxel_map){
        // TODO //
        // We don't have to build cloudMapLocalDs_ since it's exactly voxelMap_ which is already downsampled
        // We have to update cloudScanCurrDs_ based on VoxelDownsample(cloudScanCurr_, config_.voxel_size)

        // convert the cloudScanCurr_ to std::vector<Eigen::Vector3d>
        std::vector<Eigen::Vector3d> cloudScanCurr_vecEigen;
        for (auto& pt: cloudScanCurr_->points){
            Eigen::Vector3d ptEig(pt.x, pt.y, pt.z);
            cloudScanCurr_vecEigen.emplace_back(ptEig);
        }

        // voxel downsample on 
        if(config_.turn_on_voxel_downsample){
            // cloudScanCurrDs_vecEigen_ = VoxelDownsample(cloudScanCurr_vecEigen, config_.voxel_size * 0.5);
            // following kiss-icp parameter
            cloudScanCurrDsToMap_vecEigen_ = VoxelDownsample(cloudScanCurr_vecEigen, config_.voxel_size * config_.voxel_downsample_resolution_raw_to_mapping);
            cloudScanCurrDs_vecEigen_ = VoxelDownsample(cloudScanCurrDsToMap_vecEigen_, config_.voxel_size * config_.voxel_downsample_resolution_mapping_to_icpsource);
            
            // check the number
            if (debug_print_num_downsampled_point)
                RCLCPP_WARN_STREAM(get_logger(), "cloudScanCurrDsToMap_vecEigen_ num.point = " << cloudScanCurrDsToMap_vecEigen_.size());
        }else{ // voxel downsample off
            cloudScanCurrDs_vecEigen_ = cloudScanCurr_vecEigen; 
        }

        // keep a debugging code
        // // JUST for debugging: add the current scan to the voxel map before NN search to see NN search works well
        // for (auto& ptEig: cloudScanCurrDs_vecEigen_)
        // {
        //     Eigen::Vector3d t_w_pt = q_w_bCurrKf_ * ptEig + t_w_bCurrKf_;
        //     cloudScanCurrInWorld_vecEigen_.push_back(t_w_pt);
        // }
        // voxelMap_.Update(cloudScanCurrInWorld_vecEigen_, t_w_bCurrKf_);
        // cloudScanCurrInWorld_vecEigen_.clear();
        // // JUST for debugging

        // convert std::vector<Eigen::Vector3d> back to cloudScanCurrDs_
        cloudScanCurrDs_->clear();
        for (auto& ptEig: cloudScanCurrDs_vecEigen_){
            PointType pt;
            pt.x = ptEig.x();
            pt.y = ptEig.y();
            pt.z = ptEig.z();
            cloudScanCurrDs_->points.emplace_back(pt);
        }
    }else{ // naive pcl::pointcloud map ver.
        downsizeFilterMapLocal_.setInputCloud(cloudMapLocal_);
        cloudMapLocalDs_->clear();
        downsizeFilterMapLocal_.filter(*cloudMapLocalDs_);
        // std::swap(cloudMapLocalDs_, cloudMapLocal_); // what if we don't downsample localCloudMap??
        // -> the computation for downsampleCloud() drastically decreased

        // for debugging
        if(debug_print_first_point_in_current_scan){
            RCLCPP_INFO_STREAM(this->get_logger(), __FUNCTION__ << __LINE__);
            RCLCPP_INFO_STREAM(this->get_logger(), "  cloudScanCurr_->points[0].x: " << cloudScanCurr_->points[0].x);
            RCLCPP_INFO_STREAM(this->get_logger(), "  cloudScanCurr_->points[0].y: " << cloudScanCurr_->points[0].y);
            RCLCPP_INFO_STREAM(this->get_logger(), "  cloudScanCurr_->points[0].z: " << cloudScanCurr_->points[0].z);
        }

        downsizeFilterScanCurr_.setInputCloud(cloudScanCurr_);
        cloudScanCurrDs_->clear();
        downsizeFilterScanCurr_.filter(*cloudScanCurrDs_);
    }

    // for debugging
    if (debug_print_num_downsampled_point)
        RCLCPP_WARN_STREAM(get_logger(), "after downsample num.point = " << cloudScanCurrDs_->points.size());

    // for debugging
    if(debug_print_first_point_in_current_scan){
        RCLCPP_INFO_STREAM(this->get_logger(), __FUNCTION__ << __LINE__);
        RCLCPP_INFO_STREAM(this->get_logger(), "  cloudScanCurrDs_->points[0].x: " << cloudScanCurrDs_->points[0].x);
        RCLCPP_INFO_STREAM(this->get_logger(), "  cloudScanCurrDs_->points[0].y: " << cloudScanCurrDs_->points[0].y);
        RCLCPP_INFO_STREAM(this->get_logger(), "  cloudScanCurrDs_->points[0].z: " << cloudScanCurrDs_->points[0].z);
    }
}

void Frontend::solveLeastSquares_Ceres_LeftMultiplying()
{
    if(!config_.use_voxel_map){
        // set the local map point cloud to the kd_tree to nearest neighbor search
        timeLogger_.start("kdTreeMapLocal_->setInputCloud", __FUNCTION__, __LINE__);
        kdTreeMapLocal_->setInputCloud(cloudMapLocalDs_);
        timeLogger_.stop("kdTreeMapLocal_->setInputCloud", __FUNCTION__, __LINE__);
    }

    // Set initial pose based on the initial guess
    q_w_bPrevKf_.normalize();
    // Eigen::Vector3d t_w_bCurrKf_initGuess = q_w_bPrevKf_ * t_bPrevKf_bCurrKf_initGuess_ + t_w_bPrevKf_;
    // Eigen::Quaterniond q_w_bCurrKf_initGuess = (q_w_bPrevKf_ * q_bPrevKf_bCurrKf_initGuess_).normalized();
    t_w_bCurrKf_ = q_w_bPrevKf_ * t_bPrevKf_bCurrKf_initGuess_ + t_w_bPrevKf_;
    q_w_bCurrKf_ = (q_w_bPrevKf_ * q_bPrevKf_bCurrKf_initGuess_).normalized();

    // create pose parameters variables to optimize in Ceres solver    
    // pose representation: [quaternion: w, x, y, z | transition: x, y, z]
    // this parameters are iteratively optimized in Ceres
    // we have to give pointers of the pose parameters to Ceres
    // note that these parameters must represent an absolute pose (T_w_b: transformation of the robot w.r.t. the world) due to the design of Ceres solver
    //  because the jacobian is left-jacobian so the pose increment should bd wrt world frame = absolute pose
    double icpPoseParam[7] = 
    {
        q_w_bCurrKf_.w(),
        q_w_bCurrKf_.x(),
        q_w_bCurrKf_.y(),
        q_w_bCurrKf_.z(),
        t_w_bCurrKf_.x(),
        t_w_bCurrKf_.y(),
        t_w_bCurrKf_.z()
    };

    // ICP iteration (point-to-plane)
    for (int iter_cnt = 0; iter_cnt < config_.icp_iteration_num; iter_cnt++) {

        // moved to here to keep the last residual parameters for Hessian analysis
        pointScanCurrInBForResidual_.clear();
        planeNormalForResidual_.clear();
        planeDistFromOriginForResidual_.clear();

        // define a loss function with some kernel
        ceres::LossFunction *lossFunction = new ceres::HuberLoss(0.1); // the Huber kernel is the same as liliom

        // define rotation parameterization: we parameterize rotation as quarternion by using built-in parameterization in Ceres
        // if you use variables that live on manifolds(Lie-Groups) you have to define local parameterizations to tell Ceres how to manipulate those variables
        // translation components live on a common vector space so they don't need parameterization, but rotation components do.
        ceres::LocalParameterization *quatParameterization = new ceres::QuaternionParameterization();

        // create a problem object in Ceres
        ceres::Problem problem;

        // add a pointer to the rotation variables to optimize, with the parameterization
        problem.AddParameterBlock(icpPoseParam, 4, quatParameterization);
        // void Problem::AddParameterBlock(double *values, int size, Manifold *manifold)
        //  -> icpPoseParam,4,quatParameterization = pointer to icpPoseParam[0] and 4 succeeding components in the array with manifold parameterization
        //                                         = rotation component in icpPoseParam (icpPoseParam[0],[1],[2],[3])

        // add a pointer to the translation variables to optimize 
        problem.AddParameterBlock(icpPoseParam + 4, 3);
        // void Problem::AddParameterBlock(double *values, int size)
        //  -> icpPoseParam+4,3 = pointer to icpPoseParam[4] and 3 succeeding components in the array 
        //                      = translation component in icpPoseParam (icpPoseParam[4],[5],[6])

        timeLogger_.start("preparePointPlaneResidual", __FUNCTION__, __LINE__);
        // compute necessary values for point-to-plane residual computation
        // this is a praparation for the objective function in the least squares, later added to the problem by AddResidualBlock()
        preparePointPlaneResidual();
        // compute:
        // x:point position in world
        // n:plane normal in world
        // d:the distance from the world origin to the plane -> plane offset (scaler)
        timeLogger_.stop("preparePointPlaneResidual", __FUNCTION__, __LINE__);

        // for debugging
        if (debug_print_num_residuals) {
            // std::cout << "num. of pts nn found = " << numPtsNnFound_ << std::endl;
            // std::cout << "num. of residual points = " << numResidual_ << std::endl;
            RCLCPP_INFO_STREAM(get_logger(), "num. of pts nn found = " << numPtsNnFound_);
            RCLCPP_INFO_STREAM(get_logger(), "num. of residual points = " << numResidual_);
        }

        // loop over all the residuals and add them to the objective funtion, forming the entire objective function in the least squares problem
        for (int i = 0; i < numResidual_; ++i) {
            // set x:the point position
            Eigen::Vector3d currentPt = pointScanCurrInBForResidual_[i]; // Eigen::Vector3d ver.
            // here currentPt is a measured point in the body frame and the transformation from body to world is done in LidarPlaneNormIncreFactor
            // note that we have to give the point position in BODY to allow Ceres to examine the loss change w.r.t. the transformation increment in LidarPlaneNormIncreFactor
            // We always give point coordinate in Body to the optimization solver since point B->W transformation is done based on the transformation at the current iteration in non-linear optimization for solving the current icp iteration
            //      inner loop: non-linear optimization -> update Jacobian, residual, pose increment at each iteration
            //      outer loop: icp iteration -> update NN points, plane normal, plane offset
            
            // set n:the normal
            Eigen::Vector3d norm = planeNormalForResidual_[i];
            // set d:distance from the world origin to the plane(=norm inverse)
            double normInverse = planeDistFromOriginForResidual_[i]; // Eigen::Vector3d ver.
            // set cost function (residual) based on x,n,d
            // point-to-plane residual (=nx+d) will be computed in the cost function (LidarPlaneNormIncreFactor)
            // one point corresponds to one residual(cost function) 
            // Jacobian is to be computed in ceres::AutoDiff in LidarPlaneNormIncreFactor, so we don't define analytical Jacobian in this code
            ceres::CostFunction *costFunction = LidarPlaneNormIncreFactor_LeftMultiplying::Create(currentPt, norm, normInverse);
            problem.AddResidualBlock(costFunction, lossFunction, icpPoseParam, icpPoseParam + 4);
            // -> icpPoseParam   = parameters of rotation component in icpPoseParam (icpPoseParam[0],[1],[2],[3])
            // -> icpPoseParam+4 = parameters of translation component in icpPoseParam (icpPoseParam[4],[5],[6])
            // ResidualBlockId Problem::AddResidualBlock(CostFunction *cost_function, LossFunction *loss_function, const std::vector<double*> parameter_blocks)
        }

        // set some solver parameters
        ceres::Solver::Options solverOptions;
        solverOptions.linear_solver_type = ceres::DENSE_QR;
        solverOptions.max_num_iterations = config_.max_iterations;
        solverOptions.max_solver_time_in_seconds = config_.max_solver_time_in_seconds;
        solverOptions.minimizer_progress_to_stdout = false;
        solverOptions.check_gradients = false;
        solverOptions.gradient_check_relative_precision = 1e-2;

        // create a solution container
        ceres::Solver::Summary summary;

        timeLogger_.start("ceres::Solve", __FUNCTION__, __LINE__);
        // solve the least squares for the current iteration in ICP
        ceres::Solve(solverOptions, &problem, &summary);
        timeLogger_.stop("ceres::Solve", __FUNCTION__, __LINE__);

        // make sure w in rotation quaternion is positive (carried over from liliom)
        // this might not be necesary but keep it just in case
        if(icpPoseParam[0] < 0) {
            Eigen::Quaterniond tmpQ(icpPoseParam[0],
                    icpPoseParam[1],
                    icpPoseParam[2],
                    icpPoseParam[3]);
            tmpQ = unifyQuaternion(tmpQ);
            icpPoseParam[0] = tmpQ.w();
            icpPoseParam[1] = tmpQ.x();
            icpPoseParam[2] = tmpQ.y();
            icpPoseParam[3] = tmpQ.z();
        }

        // moved to the top to keep the last residual parameters for Hessian analysis
        // pointScanCurrInBForResidual_.clear();
        // planeNormalForResidual_.clear();
        // planeDistFromOriginForResidual_.clear();

        // update the pose variables to the latest optimized ones
        // icpPoseParam[] is what is being optimized in the Ceres process above
        // we need to update the pose (q_w_bCurrKf_, t_w_bCurrKf_) to re-compute the nearest neighbor points and planes in preparePointPlaneResidual() at each iteration
        // , so we have to update the pose variables(q_w_bCurrKf_, t_w_bCurrKf_) at each ICP iteration.
        q_w_bCurrKf_ = Eigen::Quaterniond(
                            icpPoseParam[0],
                            icpPoseParam[1],
                            icpPoseParam[2],
                            icpPoseParam[3]);
        t_w_bCurrKf_ = Eigen::Vector3d(
                            icpPoseParam[4],
                            icpPoseParam[5],
                            icpPoseParam[6]);
    }
    // end of the point-to-plane icp
}

void Frontend::solveLeastSquares_Ceres_RightMultiplying()
{
    if(!config_.use_voxel_map){
        // set the local map point cloud to the kd_tree to nearest neighbor search
        timeLogger_.start("kdTreeMapLocal_->setInputCloud", __FUNCTION__, __LINE__);
        kdTreeMapLocal_->setInputCloud(cloudMapLocalDs_);
        timeLogger_.stop("kdTreeMapLocal_->setInputCloud", __FUNCTION__, __LINE__);
    }

    // Set initial pose based on the initial guess
    q_w_bPrevKf_.normalize();
    // Eigen::Vector3d t_w_bCurrKf_initGuess = q_w_bPrevKf_ * t_bPrevKf_bCurrKf_initGuess_ + t_w_bPrevKf_;
    // Eigen::Quaterniond q_w_bCurrKf_initGuess = (q_w_bPrevKf_ * q_bPrevKf_bCurrKf_initGuess_).normalized();
    t_w_bCurrKf_ = q_w_bPrevKf_ * t_bPrevKf_bCurrKf_initGuess_ + t_w_bPrevKf_;
    q_w_bCurrKf_ = (q_w_bPrevKf_ * q_bPrevKf_bCurrKf_initGuess_).normalized();

    RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // create pose parameters variables to optimize in Ceres solver    
    // pose representation: [quaternion: w, x, y, z | transition: x, y, z]
    // this parameters are iteratively optimized in Ceres
    // we have to give pointers of the pose parameters to Ceres
    // note that these parameters must represent an absolute pose (T_w_b: transformation of the robot w.r.t. the world) due to the design of Ceres solver
    //  because the jacobian is left-jacobian so the pose increment should bd wrt world frame = absolute pose
    // double icpPoseParam[7] = 
    // {
    //     q_w_bCurrKf_.w(),
    //     q_w_bCurrKf_.x(),
    //     q_w_bCurrKf_.y(),
    //     q_w_bCurrKf_.z(),
    //     t_w_bCurrKf_.x(),
    //     t_w_bCurrKf_.y(),
    //     t_w_bCurrKf_.z()
    // };

    // Eigen::Vector3d t_lifting_point = t_w_bCurrKf_;
    // Eigen::Quaterniond q_lifting_point = q_w_bCurrKf_;

    // // starts from zero increment
    // // this is pose increment
    // double icpPoseParam[7] = 
    // {
    //     1.0,
    //     0.0,
    //     0.0,
    //     0.0,
    //     0.0,
    //     0.0,
    //     0.0
    // };

    // [TODO] initial guess is already incremented to q_w_bCurrKf_, t_w_bCurrKf_ in setInitialPoseLO()
    //        but it should be incremented here as initialization of icpPoseParam[]
    //        So, icpPoseParam[] should start with the initial guess pose -> this is more strainghtforward

    // ICP iteration (point-to-plane)
    for (int iter_cnt = 0; iter_cnt < config_.icp_iteration_num; iter_cnt++) {

        Eigen::Vector3d t_lifting_point = t_w_bCurrKf_;
        Eigen::Quaterniond q_lifting_point = q_w_bCurrKf_;

        // starts from zero increment
        // this is pose increment
        double icpPoseParam[7] = 
        {
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        };

        // moved to here to keep the last residual parameters for Hessian analysis
        pointScanCurrInBForResidual_.clear();
        planeNormalForResidual_.clear();
        planeDistFromOriginForResidual_.clear();

        // define a loss function with some kernel
        ceres::LossFunction *lossFunction = new ceres::HuberLoss(0.1); // the Huber kernel is the same as liliom

        // define rotation parameterization: we parameterize rotation as quarternion by using built-in parameterization in Ceres
        // if you use variables that live on manifolds(Lie-Groups) you have to define local parameterizations to tell Ceres how to manipulate those variables
        // translation components live on a common vector space so they don't need parameterization, but rotation components do.
        ceres::LocalParameterization *quatParameterization = new ceres::QuaternionParameterization();

        // create a problem object in Ceres
        ceres::Problem problem;

        // add a pointer to the rotation variables to optimize, with the parameterization
        problem.AddParameterBlock(icpPoseParam, 4, quatParameterization);
        // void Problem::AddParameterBlock(double *values, int size, Manifold *manifold)
        //  -> icpPoseParam,4,quatParameterization = pointer to icpPoseParam[0] and 4 succeeding components in the array with manifold parameterization
        //                                         = rotation component in icpPoseParam (icpPoseParam[0],[1],[2],[3])

        // add a pointer to the translation variables to optimize 
        problem.AddParameterBlock(icpPoseParam + 4, 3);
        // void Problem::AddParameterBlock(double *values, int size)
        //  -> icpPoseParam+4,3 = pointer to icpPoseParam[4] and 3 succeeding components in the array 
        //                      = translation component in icpPoseParam (icpPoseParam[4],[5],[6])

        timeLogger_.start("preparePointPlaneResidual", __FUNCTION__, __LINE__);
        // compute necessary values for point-to-plane residual computation
        // this is a praparation for the objective function in the least squares, later added to the problem by AddResidualBlock()
        preparePointPlaneResidual();
        // compute:
        // x:point position in world
        // n:plane normal in world
        // d:the distance from the world origin to the plane -> plane offset (scaler)
        timeLogger_.stop("preparePointPlaneResidual", __FUNCTION__, __LINE__);

        // for debugging
        if (debug_print_num_residuals) {
            // std::cout << "num. of pts nn found = " << numPtsNnFound_ << std::endl;
            // std::cout << "num. of residual points = " << numResidual_ << std::endl;
            RCLCPP_INFO_STREAM(get_logger(), "num. of pts nn found = " << numPtsNnFound_);
            RCLCPP_INFO_STREAM(get_logger(), "num. of residual points = " << numResidual_);
        }

        // loop over all the residuals and add them to the objective funtion, forming the entire objective function in the least squares problem
        for (int i = 0; i < numResidual_; ++i) {
            // set x:the point position
            Eigen::Vector3d currentPt = pointScanCurrInBForResidual_[i]; // Eigen::Vector3d ver.
            // here currentPt is a measured point in the body frame and the transformation from body to world is done in LidarPlaneNormIncreFactor
            // note that we have to give the point position in BODY to allow Ceres to examine the loss change w.r.t. the transformation increment in LidarPlaneNormIncreFactor
            // We always give point coordinate in Body to the optimization solver since point B->W transformation is done based on the transformation at the current iteration in non-linear optimization for solving the current icp iteration
            //      inner loop: non-linear optimization -> update Jacobian, residual, pose increment at each iteration
            //      outer loop: icp iteration -> update NN points, plane normal, plane offset
            
            // set n:the normal
            Eigen::Vector3d norm = planeNormalForResidual_[i];
            // set d:distance from the world origin to the plane(=norm inverse)
            double normInverse = planeDistFromOriginForResidual_[i]; // Eigen::Vector3d ver.
            // set cost function (residual) based on x,n,d
            // point-to-plane residual (=nx+d) will be computed in the cost function (LidarPlaneNormIncreFactor)
            // one point corresponds to one residual(cost function) 
            // Jacobian is to be computed in ceres::AutoDiff in LidarPlaneNormIncreFactor, so we don't define analytical Jacobian in this code
            // ceres::CostFunction *costFunction = LidarPlaneNormIncreFactor::Create(currentPt, norm, normInverse);
            ceres::CostFunction *costFunction = LidarPlaneNormIncreFactor_RightMultiplying::Create(currentPt, norm, normInverse, q_lifting_point, t_lifting_point);
            problem.AddResidualBlock(costFunction, lossFunction, icpPoseParam, icpPoseParam + 4);
            // -> icpPoseParam   = parameters of rotation component in icpPoseParam (icpPoseParam[0],[1],[2],[3])
            // -> icpPoseParam+4 = parameters of translation component in icpPoseParam (icpPoseParam[4],[5],[6])
            // ResidualBlockId Problem::AddResidualBlock(CostFunction *cost_function, LossFunction *loss_function, const std::vector<double*> parameter_blocks)
        }

        // set some solver parameters
        ceres::Solver::Options solverOptions;
        solverOptions.linear_solver_type = ceres::DENSE_QR;
        solverOptions.max_num_iterations = config_.max_iterations;
        solverOptions.max_solver_time_in_seconds = config_.max_solver_time_in_seconds;
        solverOptions.minimizer_progress_to_stdout = false;
        solverOptions.check_gradients = false;
        solverOptions.gradient_check_relative_precision = 1e-2;

        // create a solution container
        ceres::Solver::Summary summary;

        timeLogger_.start("ceres::Solve", __FUNCTION__, __LINE__);
        // solve the least squares for the current iteration in ICP
        ceres::Solve(solverOptions, &problem, &summary);
        timeLogger_.stop("ceres::Solve", __FUNCTION__, __LINE__);

        // make sure w in rotation quaternion is positive (carried over from liliom)
        // this might not be necesary but keep it just in case
        if(icpPoseParam[0] < 0) {
            Eigen::Quaterniond tmpQ(icpPoseParam[0],
                    icpPoseParam[1],
                    icpPoseParam[2],
                    icpPoseParam[3]);
            tmpQ = unifyQuaternion(tmpQ);
            icpPoseParam[0] = tmpQ.w();
            icpPoseParam[1] = tmpQ.x();
            icpPoseParam[2] = tmpQ.y();
            icpPoseParam[3] = tmpQ.z();
        }

        // moved to the top to keep the last residual parameters for Hessian analysis
        // pointScanCurrInBForResidual_.clear();
        // planeNormalForResidual_.clear();
        // planeDistFromOriginForResidual_.clear();

        // update the pose variables to the latest optimized ones
        // icpPoseParam[] is what is being optimized in the Ceres process above
        // we need to update the pose (q_w_bCurrKf_, t_w_bCurrKf_) to re-compute the nearest neighbor points and planes in preparePointPlaneResidual() at each iteration
        // , so we have to update the pose variables(q_w_bCurrKf_, t_w_bCurrKf_) at each ICP iteration.
        // q_w_bCurrKf_ = Eigen::Quaterniond(
        //                     icpPoseParam[0],
        //                     icpPoseParam[1],
        //                     icpPoseParam[2],
        //                     icpPoseParam[3]);
        // t_w_bCurrKf_ = Eigen::Vector3d(
        //                     icpPoseParam[4],
        //                     icpPoseParam[5],
        //                     icpPoseParam[6]);

        // extract pose increment 
        Eigen::Quaterniond q_increment = Eigen::Quaterniond(
                                                icpPoseParam[0],
                                                icpPoseParam[1],
                                                icpPoseParam[2],
                                                icpPoseParam[3]);
        q_increment.normalized();
        Eigen::Vector3d t_increment = Eigen::Vector3d(
                                                icpPoseParam[4],
                                                icpPoseParam[5],
                                                icpPoseParam[6]);
        
        // update the current pose for the next icp iteration
        t_w_bCurrKf_ = q_w_bCurrKf_ * t_increment + t_w_bCurrKf_;
        q_w_bCurrKf_ = (q_w_bCurrKf_ * q_increment).normalized();
    }
    // end of the point-to-plane icp
}

void Frontend::preparePointPlaneResidual(){
    numPtsNnFound_ = 0;
    numResidual_ = 0;

    const size_t num_pts_curr = cloudScanCurrDs_->points.size();
    pointScanCurrInBForResidual_.resize(num_pts_curr);
    planeNormalForResidual_.resize(num_pts_curr);
    planeDistFromOriginForResidual_.resize(num_pts_curr);
    std::vector<bool> valid(num_pts_curr, false);

// #pragma omp parallel for num_threads(8) schedule(static)
#pragma omp parallel for
    for (size_t i = 0; i < num_pts_curr; ++i) {
        PointType point_curr_scan_inw;    // should be point_curr_scan
        PointType point_curr_scan_inb = cloudScanCurrDs_->points[i];

        transformPointToWorldFrame(point_curr_scan_inb, point_curr_scan_inw);

        // for debugging
        if(debug_print_point_plane_residual_preparation){
            RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
            RCLCPP_INFO_STREAM(get_logger(), "point_curr_scan_inw:");
            RCLCPP_INFO_STREAM(get_logger(), "  x: " << point_curr_scan_inw.x);
            RCLCPP_INFO_STREAM(get_logger(), "  y: " << point_curr_scan_inw.y);
            RCLCPP_INFO_STREAM(get_logger(), "  z: " << point_curr_scan_inw.z);
            RCLCPP_INFO_STREAM(get_logger(), "point_curr_scan_inb:");
            RCLCPP_INFO_STREAM(get_logger(), "  x: " << point_curr_scan_inb.x);
            RCLCPP_INFO_STREAM(get_logger(), "  y: " << point_curr_scan_inb.y);
            RCLCPP_INFO_STREAM(get_logger(), "  z: " << point_curr_scan_inb.z);
        }

        std::vector<int> neighbor_ids;
        std::vector<float> neighbor_dists;
        std::vector<Eigen::Vector3d> neighbors;

        if(config_.use_voxel_map){
            Eigen::Vector3d query(point_curr_scan_inw.x, point_curr_scan_inw.y, point_curr_scan_inw.z);
            voxelMap_.GetClosestNNeighbors(query, 5, neighbors, neighbor_dists, config_.threshold_voxel_nn_search_radius);

            // [Threshold] num. of neighborhoods found
            if (neighbors.size() != 5 || neighbor_dists.size() != 5) continue; 
        }else{
            kdTreeMapLocal_->nearestKSearch(point_curr_scan_inw, 5, neighbor_ids, neighbor_dists); // PCL nearestKSearch typically returns squared distances?

            for (size_t j=0; j<neighbor_ids.size(); j++){
                Eigen::Vector3d pt(
                    cloudMapLocalDs_->points[neighbor_ids[j]].x, 
                    cloudMapLocalDs_->points[neighbor_ids[j]].y, 
                    cloudMapLocalDs_->points[neighbor_ids[j]].z);
                neighbors.push_back(pt);
            }
        }
        
#pragma omp atomic
        numPtsNnFound_++;   // just for debugging

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = - Eigen::Matrix<double, 5, 1>::Ones();
 
        // [Threshold] Point-to-Plane distance
        if (neighbor_dists[4] < 1.0) {
            PointType center;       // not used currently

            for (int j = 0; j < 5; ++j) {
                matA0(j, 0) = neighbors[j].x();
                matA0(j, 1) = neighbors[j].y();
                matA0(j, 2) = neighbors[j].z();
                
                // for debugging
                if(debug_print_point_plane_residual_preparation){
                    RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
                    RCLCPP_INFO_STREAM(get_logger(), "NN search result ----");
                    RCLCPP_INFO_STREAM(get_logger(), "point_search_voxel:");
                    RCLCPP_INFO_STREAM(get_logger(), "  x: " << neighbors[j].x());
                    RCLCPP_INFO_STREAM(get_logger(), "  y: " << neighbors[j].y());
                    RCLCPP_INFO_STREAM(get_logger(), "  z: " << neighbors[j].z());
                }
            }

            // solve linear system to extract a plane normal vector (this is also a least squares)
            // get the norm of the plane using linear solver based on QR composition
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0); //  = v: v = d . n
            double normInverse = 1 / norm.norm(); // = d:distance from the plane to the origin
            norm.normalize(); // get the unit norm // = n

            // Compute the centroid of the plane
            center.x = matA0.col(0).sum() / 5.0;
            center.y = matA0.col(1).sum() / 5.0;
            center.z = matA0.col(2).sum() / 5.0;
            // Actually these are not used

            // [Threshold] Planarity: make sure that the plane is fit well
            // check the distance between the 5 map points and the plane
            bool planeValid = true;
            for (int j = 0; j < 5; ++j) {
                if (fabs(norm.x() * neighbors[j].x() +
                         norm.y() * neighbors[j].y() +
                         norm.z() * neighbors[j].z() + normInverse) > 0.06) {
                    planeValid = false;
                    break;
                }
            }

            // if it's validated to be a plane,
            if (planeValid) {
                // compute point-plane distance (n.x + d)
                float pd = norm.x() * point_curr_scan_inw.x 
                         + norm.y() * point_curr_scan_inw.y 
                         + norm.z() * point_curr_scan_inw.z 
                         + normInverse;
                // compute a weight for a point based on the point-plane distance
                float weight = 1 - 0.9 * fabs(pd) / sqrt(sqrt(point_curr_scan_inw.x * point_curr_scan_inw.x + point_curr_scan_inw.y * point_curr_scan_inw.y + point_curr_scan_inw.z * point_curr_scan_inw.z));

                // if the weight exceeds the threshold, store the points and normal into the residual container
                // This is just for computing weighted normal, plane-origin distance, which is used for residual computation in LidarPlaneNormIncreFactor cost function or QP function
                //  in the cost function, the measured points are transfromed to the world coordinate, so points should be in the body coordinate here.
                // we adopt a weight calculation of either fast-lio or lili-om way (weight as threshold for residual, not weight for least squares robust kernel)
                if(config_.use_fastlio_point_plane_residual_param){
                    if(weight > 0.9){ // fast-lio 
                        Eigen::Vector3d point_inb(point_curr_scan_inb.x, point_curr_scan_inb.y, point_curr_scan_inb.z);
                        Eigen::Vector3d normal(norm.x(), norm.y(), norm.z());
                        pointScanCurrInBForResidual_[i] = point_inb;
                        planeNormalForResidual_[i] = normal;
                        planeDistFromOriginForResidual_[i] = normInverse;
                        valid[i] = true;

                        // for debugging
                        if(debug_print_point_plane_residual_preparation)
                            RCLCPP_INFO_STREAM(get_logger(), "direct point-to-plane distance (pd): " << pd);
                    }
                }else{ // This part would no longer be necessary
                    if(weight > 0.4){ // lili-om 
                        Eigen::Vector3d point_inb(point_curr_scan_inb.x, point_curr_scan_inb.y, point_curr_scan_inb.z);
                        Eigen::Vector3d normal(norm.x(), norm.y(), norm.z());
                        pointScanCurrInBForResidual_[i] = point_inb;
                        planeDistFromOriginForResidual_[i] = normInverse;
                        valid[i] = true;

                        // for debugging
                        if(debug_print_point_plane_residual_preparation)
                            RCLCPP_INFO_STREAM(get_logger(), "direct point-to-plane distance (pd): " << pd);
                    }
                }

                // the quantities are stored:
                // x:point position -> surf_current_pts
                // n:plane normal(weighted) -> normal.xyz in surf_normal
                // d:the distance from the world origin to the plane -> normal.intensity in surf_normal
                //  ,which are used for point-to-plane residual computation later
            }
        }
    }

    // fit the size of the vector to that only for non-zero values
    std::vector<Eigen::Vector3d> pts_inb_compact;
    std::vector<Eigen::Vector3d> normals_compact;
    std::vector<double> dists_compact;
    pts_inb_compact.reserve(num_pts_curr);
    normals_compact.reserve(num_pts_curr);
    dists_compact.reserve(num_pts_curr);
    for (std::size_t i = 0; i < num_pts_curr; ++i) {
        if (valid[i]) { 
            pts_inb_compact.push_back(pointScanCurrInBForResidual_[i]);
            normals_compact.push_back(planeNormalForResidual_[i]); 
            dists_compact.push_back(planeDistFromOriginForResidual_[i]); 
        }
    }
    pointScanCurrInBForResidual_.swap(pts_inb_compact);
    planeNormalForResidual_.swap(normals_compact);
    planeDistFromOriginForResidual_.swap(dists_compact);
    numResidual_ = static_cast<int>(planeNormalForResidual_.size());
}


void Frontend::computeCovarianceLO()
{
    Eigen::MatrixXd A(numResidual_, 6); // stacked Jacobian
    Eigen::MatrixXd b(numResidual_, 1); // residual vectorn
    Eigen::MatrixXd b_for_kernel(numResidual_, 1); // residual vector to be used for kernel weight computation

    // loop over all the residuals and add them to the objective funtion, forming the entire objective function in the least squares problem
    for (int i = 0; i < numResidual_; ++i) 
    {
        const Eigen::Vector3d bp = pointScanCurrInBForResidual_[i];
        const Eigen::Vector3d wp = q_w_bCurrKf_ * bp + t_w_bCurrKf_; // should be updated at each iteration based on the current pose update
        const Eigen::Vector3d wn = planeNormalForResidual_[i];  // plane normal (does not change depending on the current body pose)
        const double d = planeDistFromOriginForResidual_[i];    // plane offset (does not change depending on the current body pose)

        // Jacobian Computation
        if(config_.pose_increment_multiplication == "right")
        {
            // Right Jacobian
            Eigen::Matrix3d Rwb = q_w_bCurrKf_.toRotationMatrix();
            Eigen::Matrix3d bp_hat = getSkewSymMatrix(bp);
            // row i of A: [ -wn^T @ Rwb @ [bp]x  wn^T @ Rwb ]
            A.row(i).head<3>() = -wn.transpose() * Rwb * bp_hat;
            A.row(i).tail<3>() = wn.transpose() * Rwb;
            // residual b = - (wn^T @ wp + d)
            b(i) = - (wn.transpose() * wp + d);
            b_for_kernel(i) = b(i);             // for kernel weighting for robustifying the cost function, which is to be used later
            // following a conventional || A x - b' || notation
            // = || A x - (- (wn^T @ wp + d)) || 
            // = || A x + (wn^T @ wp + d) ||  
            // = || A x + b0 || where b0 = wn.transpose() * wp(0) + d
        }
        else if (config_.pose_increment_multiplication == "left")
        {
            // Left Jacobian
            // row i of A: [ (p × n)^T  n^T ]
            A.row(i).head<3>() = wp.cross(wn).transpose();
            A.row(i).tail<3>() = wn.transpose();
            // residual b = - (wn^T @ wp + d)
            b(i) = - (wn.transpose() * wp + d);
            b_for_kernel(i) = b(i);
        }
        else
        {
            RCLCPP_INFO_STREAM(get_logger(), "Left or Right Jacobian is not defined. Jacobian is set to zero.");
        }

        // ----- Sanity check for Jacobian (analytical Jacobian vs finite difference Jacobian) ----- 
        // // auto residual = [&](const Eigen::Vector3d& w, const Eigen::Vector3d& t){
        // //   return wn.dot( (Eigen::AngleAxisd(w.norm(), (w.norm()>1e-12)? w.normalized():Eigen::Vector3d::UnitX()).toRotationMatrix() * bp) + t ) + d;
        // // };
        // Eigen::Vector3d w0 = Eigen::Vector3d::Zero();
        // Eigen::Vector3d t0 = Eigen::Vector3d::Zero();

        // // Your analytical J at (w0,t0): A_i = [ -n^T R [b]x , n^T R ] with R from current pose
        // Eigen::RowVector<double,6> Ai;
        // Ai.head<3>() = -wn.transpose() * Rwb * bp_hat;
        // Ai.tail<3>() =  wn.transpose() * Rwb;

        // // Finite-diff directional check
        // Eigen::Matrix<double,6,1> v; v.setRandom(); v.normalize();
        // double eps = 1e-8;
        // Eigen::Vector3d dw = v.head<3>() * eps;
        // Eigen::Vector3d dt = v.tail<3>() * eps;

        // double r0 = wn.dot(Rwb*bp + t_w_bCurrKf_) + d;
        // Eigen::Matrix3d R1 = Rwb * Eigen::AngleAxisd(dw.norm(), (dw.norm()>1e-12)? dw.normalized():Eigen::Vector3d::UnitX()).toRotationMatrix();
        // double r1 = wn.dot(R1*bp + (t_w_bCurrKf_ + Rwb*dt)) + d;

        // double fd = (r1 - r0) / eps;
        // double an = Ai * v;
        // // std::cout << "dir-deriv num=" << fd << "  analytic=" << an << "  diff=" << std::abs(fd-an) << "\n";
        // RCLCPP_INFO_STREAM(get_logger(), "dir-deriv num=" << fd << "  analytic=" << an << "  diff=" << std::abs(fd-an));
        // ----- Sanity check for Jacobian (analytical Jacobian vs finite difference Jacobian) ----- 
    }
    
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> D; // Jacobian Scaling Matrix
    Eigen::VectorXd dj;  // scaling factor for each column
    if (config_.turn_on_jacobian_column_scaling_qp_active_set)
    {
        // bounds/constraints scaled on the step x 
        // Eigen::VectorXd lb_x, ub_x;        // direct bounds on x
        // Eigen::MatrixXd Acon;              // general constraint matrix on x 
        // Eigen::VectorXd Alb_x, Aub_x;

        // 1) Build column-scaling matrix D so that columns of A_s = A * D have norm ~1 ---
        const int n = A.cols();
        Eigen::VectorXd col_norms(n);
        for (int j = 0; j < n; ++j)
        {
            col_norms(j) = A.col(j).norm();
        }
        const double eps   = 1e-12;    // avoid division by zero
        const double dmin  = 1e-5;     // clamp factors to avoid extreme scaling
        const double dmax  = 1e+5;
        // Eigen::VectorXd dj = (col_norms.array() > eps).select(1.0 / col_norms.array(), 1.0); // scaling factor for each column
        dj = (col_norms.array() > eps).select(1.0 / col_norms.array(), 1.0); // scaling factor for each column, eps: caring for numerical stability
        dj = dj.cwiseMax(dmin).cwiseMin(dmax);
        // construct a column-scaling matrix D, where diagonal elements are dj
        D = dj.asDiagonal();

        // 2) Scaled Jacobian and transform bounds/constraints ---
        // column scaling, matrix multiplication
        Eigen::MatrixXd A_s = A * D;           // columns ~ unit-norm
        // // If you have simple bounds on the step x, convert to z: z = D^{-1} x
        // Eigen::VectorXd lb_z, ub_z;
        // if (lb_x.size() == n && ub_x.size() == n) {
        // // Dinv is just 1/dj
        // Eigen::VectorXd d_inv = dj.cwiseInverse();
        // lb_z = lb_x.cwiseProduct(d_inv);
        // ub_z = ub_x.cwiseProduct(d_inv);
        // }
        // // If you have general linear constraints on x: Acon * x in [Alb_x, Aub_x]
        // // convert to z: (Acon * D) * z in [Alb_x, Aub_x]
        // Eigen::MatrixXd Acon_z;
        // if (Acon.size() > 0) Acon_z = Acon * D;

        // 3) Build QP in z: min 1/2 z^T H_s z + h_s^T z ---
        //  done in the following scripts

        // Now that se have ||Ax - b||2 = ||(AD)(D^-1x) - b||2 = ||(A_s)z - b||2 
        //  (A_s) =    AD   : column-wise scaled Jacobian
        //    z   = (D^-1x) : scaled state vector (to be mapped back to x after solving QP)

        // update A to As (column-wise scaled A)
        A = A_s; 
    }
    
    // build the QP formulation
    // min(xQP): 1/2 xQP.T @ HQP @ xQP + hQP.T @ xQP
    // s.t.    : lb  <=   xQP   <= ub
    //           Alb <= A @ xQP <= Aub
    // min(x_qp_iter): 1/2 x_qp_iter.T @ HQP @ x_qp_iter + hQP.T @ x_qp_iter
    // s.t.    : lb  <=   x_qp_iter   <= ub
    //           Alb <= A @ x_qp_iter <= Aub

    Eigen::Matrix<double,6,6> HQP = Eigen::Matrix<double,6,6>::Zero();  // Hessian in QP
    Eigen::Matrix<double,6,1> hQP = Eigen::Matrix<double,6,1>::Zero();  // h matrix in QP

    // compute the weights first based on the robust kernel
    Eigen::VectorXd w;  
    if(config_.turn_on_robust_kernel_qp_active_set)
    {
        // if(config_.turn_on_mad_based_scaling_for_kernel_weights_qp_active_set)
        // {
        double s = 0.0; // default scaling just as an option manually tune it
        double c = 0.0; // default range just as an option manually tune it
        computeWeightsFromResiduals(b_for_kernel, w, s, config_.robust_kernel_qp_active_set, c, true);
    }
    else
    {
        w = Eigen::VectorXd::Ones(b_for_kernel.size());
    }

    if (debug_print_qp_active_set_matrices)
    {
        RCLCPP_INFO_STREAM(get_logger(), "w: ");
        RCLCPP_INFO_STREAM(get_logger(), w.transpose());
    }

    // Eigen::MatrixXd W = w.asDiagonal(); // This is super expensive, keep this just in case
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> W = w.asDiagonal();   // fast

    if(config_.qp_active_set_Hessian_computation == "matrix_multiplication")
    {
        // cleaner but slow way
        HQP = 2 * A.transpose() * W * A;
        hQP = - 2 * A.transpose() * W * b;
    }
    else if(config_.qp_active_set_Hessian_computation == "for_loop")
    {
        // fast way
        for (int i = 0; i < numResidual_; ++i) 
        {
            double wi = w(i);
            Eigen::Matrix<double,1,6> ai = A.row(i);
            HQP.noalias() += 2.0 * wi * (ai.transpose() * ai);
            hQP.noalias() += - 2.0 * wi * ai.transpose() * b(i);
        }
    }
    else
    {
        RCLCPP_INFO_STREAM(get_logger(), "Hessian Computation is not defined well. Set Hessian to zero.");
    }

    // We should not add any damping term
    // if(config_.turn_on_levenberg_marquardt_qp_active_set)
    // {
    //     const double lambda = config_.levenberg_marquardt_lambda_qp_active_set;
    //     if(config_.turn_on_levenberg_marquardt_qp_active_set_marquardt_damping)
    //     {
    //         HQP += 2.0 * lambda * (HQP.diagonal().asDiagonal()); // // Marquardt-type damping, "2.0 *" is for conversion to QP formulation (* 1/2 afterward)
    //     }
    //     else
    //     {
    //         HQP += 2.0 * lambda * Eigen::MatrixXd::Identity(6,6); // "2.0 *" is for conversion to QP formulation (* 1/2 afterward)
    //     }
    // }

    // update the lidar odometry measurement covariance
    lidarOdomNoise_ = gtsam::noiseModel::Gaussian::Information(HQP);
    icpHessianLatest_log_ = HQP;
}

// void Frontend::degeneracyAnalysisLO()
// {

// }

void Frontend::updatePoseLO()
{
    // right-hand side update
    q_bPrevKf_bCurrKf_lo_ = q_w_bPrevKf_.inverse() * q_w_bCurrKf_;
    q_bPrevKf_bCurrKf_lo_.normalize();
    t_bPrevKf_bCurrKf_lo_ = q_w_bPrevKf_.inverse() * (t_w_bCurrKf_ - t_w_bPrevKf_);

    // for debugging
    if(debug_print_relative_transform_lo){
        RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
        RCLCPP_INFO_STREAM(get_logger(), "timeCurrScanBeg_: " << std::fixed << timeCurrScanBeg_);
        RCLCPP_INFO_STREAM(get_logger(), "t_bPrevKf_bCurrKf_lo_: " << t_bPrevKf_bCurrKf_lo_.transpose());
        RCLCPP_INFO_STREAM(get_logger(), "q_bPrevKf_bCurrKf_lo_: " << q_bPrevKf_bCurrKf_lo_.coeffs().transpose());
        RCLCPP_INFO_STREAM(get_logger(), "t_w_bCurrKf_: " << t_w_bCurrKf_.transpose());
        RCLCPP_INFO_STREAM(get_logger(), "q_w_bCurrKf_: " << q_w_bCurrKf_.coeffs().transpose());
    }
}

void Frontend::run()
{
// RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__ << " : " << "t= " << timeCurrScanBeg_);

    // check if any measurement data available
    mtxImu_.lock();
    mtxCloud_.lock();
    if(cloudMsgBuffer_.empty() || imuMsgBuffer_.empty())
    {
        mtxImu_.unlock();
        mtxCloud_.unlock();
        return;
    }
    mtxImu_.unlock();
    mtxCloud_.unlock();

    // Check the process status in a thread-safe way. If it's processing, skip the process.
    // if set this main process as timer callback, we need to make sure there is not race condition (default: set timer callback)
    if (config_.set_main_process_timer)
    {
        bool expectedProcessStatus = false;
        if(!isProcessing_.compare_exchange_strong(expectedProcessStatus, true))
            return;
    }
    timeLogger_.start("run", __FUNCTION__, __LINE__);

    if (debug_print_lines_in_run) RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // ----- initialization -----
    if(!isCloudMapInitialized_)
    {
        timeLogger_.start("initializeCloudMap", __FUNCTION__, __LINE__);
    
        isCloudMapInitialized_ = initializeCloudMap();
        if(isCloudMapInitialized_) updateState();
        clearProcess();
        isProcessing_.store(false);

        timeLogger_.stop("initializeCloudMap", __FUNCTION__, __LINE__);

        timeLogger_.stop("run", __FUNCTION__, __LINE__);
        return;
    }

    if (debug_print_lines_in_run) RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // ----- pre-process ----- 
    // Data Synchronization
    timeLogger_.start("synchronizeMeasurements", __FUNCTION__, __LINE__);
    bool isSyncSuccess = synchronizeMeasurements();
    if(isSyncSuccess && !imuMsgKfWindow_.empty()){
        if(config_.print_sync_status) RCLCPP_INFO_STREAM(get_logger(), "Lidar-Imu frames were time-synchronized well.");
        // for debugging
        if(debug_print_scan_imu_time_sync){
            RCLCPP_INFO_STREAM(get_logger(), "  Imu Data Beginning: " << std::fixed << rclcpp::Time(imuMsgKfWindow_.front().header.stamp).seconds());
            RCLCPP_INFO_STREAM(get_logger(), "Lidar Scan Beginning: " << std::fixed << timeCurrScanBeg_);
            RCLCPP_INFO_STREAM(get_logger(), "     Imu Data Ending: " << std::fixed << rclcpp::Time(imuMsgKfWindow_.back().header.stamp).seconds());
            RCLCPP_INFO_STREAM(get_logger(), "   Lidar Scan Ending: " << std::fixed << timeCurrScanEnd_);
        }
    }else if(isSyncSuccess && imuMsgKfWindow_.empty()){
        if(config_.print_sync_status) RCLCPP_INFO_STREAM(get_logger(), "Earliest Imu measurement doesn't catch up with the lidar scan beginning time. Process only lidar measurement.");
        isCurrFrameLidarOnlyEstimation_ = true;
    }else{    // synchronization failure, need to wait more imu for the current lidar frame
        // RCLCPP_INFO_STREAM(get_logger(), "Sufficient Imu measurement hasn't come yet.");
        // RCLCPP_INFO_STREAM(get_logger(), " -> Skip a process and wait for imu measurements");
        // RCLCPP_INFO_STREAM(get_logger(), " -> to perform a complete LIO at the next process");
        clearProcess();
        isProcessing_.store(false);
        timeLogger_.stop("synchronizeMeasurements", __FUNCTION__,__LINE__);
        timeLogger_.stop("run", __FUNCTION__, __LINE__);
        return;
    }
    timeLogger_.stop("synchronizeMeasurements", __FUNCTION__,__LINE__);

    if (debug_print_lines_in_run) RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // Initialize Imu
    if(!isImuInitialized_){
        timeLogger_.start("initializeImu", __FUNCTION__, __LINE__);
        initializeImu();
        isImuInitialized_ = true;   // should be isImuInitialized_ = initializeImu()
        timeLogger_.stop("initializeImu", __FUNCTION__, __LINE__);
    }

    if (debug_print_lines_in_run) RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // Initialize Factor Graph
    if(config_.turn_on_factor_graph){
        if (!isFGInitialized_ && isStateInitialized_) {     // if FG is not initialized and a reference velocity is available
            timeLogger_.start("initializeFG", __FUNCTION__, __LINE__);
            isFGInitialized_ = initializeFG();
            timeLogger_.stop("initializeFG", __FUNCTION__, __LINE__);
        }
        // RCLCPP_INFO_STREAM(get_logger(), "isStateInitialized_: " << isStateInitialized_);
    }

    if (debug_print_lines_in_run) RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // Propagate Imu
    if(isStateInitialized_){
        timeLogger_.start("propagateImu", __FUNCTION__, __LINE__);
        propagateImu();     // if imu covariance prediction is on, run left-invariant kalman filter here.
        timeLogger_.stop("propagateImu", __FUNCTION__, __LINE__);
    }

    if (debug_print_lines_in_run) RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // Transform Point Cloud to Imu Frame
    timeLogger_.start("transformPointCloudInImuFrame", __FUNCTION__, __LINE__);
    transformPointCloudInImuFrame();
    timeLogger_.stop("transformPointCloudInImuFrame", __FUNCTION__, __LINE__);

    if (debug_print_lines_in_run) RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // Compute an initial guess transformation for Lidar Odometry ICP
    timeLogger_.start("makeInitialGuessLO", __FUNCTION__, __LINE__);
    makeInitialGuessLO();
    timeLogger_.stop("makeInitialGuessLO", __FUNCTION__, __LINE__);
    // update t_bPrevKf_bCurrKf_initGuess_ and q_bPrevKf_bCurrKf_initGuess_

    if (debug_print_lines_in_run) RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    timeLogger_.start("deskewPointCloud", __FUNCTION__, __LINE__);
    deskewPointCloud();
    timeLogger_.stop("deskewPointCloud", __FUNCTION__, __LINE__);

    if (debug_print_lines_in_run) RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // Compute an initial guess Pose in World for Lidar Odometry ICP
    // timeLogger_.start("setInitialPoseLO", __FUNCTION__, __LINE__);
    // setInitialPoseLO();
    // timeLogger_.stop("setInitialPoseLO", __FUNCTION__, __LINE__);
    // compute t_w_bCurrKf_initGuess, q_w_bCurrKf_initGuess
    // update t_w_bCurrKf_ and q_w_bCurrKf_

    if (debug_print_lines_in_run) RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    timeLogger_.start("buildLocalMap", __FUNCTION__, __LINE__);
    buildLocalMap();
    timeLogger_.stop("buildLocalMap", __FUNCTION__, __LINE__);
    
    if (debug_print_lines_in_run) RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    timeLogger_.start("downsampleCloud", __FUNCTION__, __LINE__);
    downsampleCloud();
    timeLogger_.stop("downsampleCloud", __FUNCTION__, __LINE__);

    if (debug_print_lines_in_run) RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    if(config_.inequality_constraints_solver == "Ceres_left_multiplying"){
        timeLogger_.start("solveLeastSquares_Ceres_LeftMultiplying", __FUNCTION__, __LINE__);
        solveLeastSquares_Ceres_LeftMultiplying();    // unconstrained least squares
        timeLogger_.stop("solveLeastSquares_Ceres_LeftMultiplying", __FUNCTION__, __LINE__);
    }else if(config_.inequality_constraints_solver == "Ceres_right_multiplying"){
        timeLogger_.start("solveLeastSquares_Ceres_RightMultiplying", __FUNCTION__, __LINE__);
        solveLeastSquares_Ceres_RightMultiplying();    // unconstrained least squares
        timeLogger_.stop("solveLeastSquares_Ceres_RightMultiplying", __FUNCTION__, __LINE__);
    }else if(config_.inequality_constraints_solver == "qpmad"){
        timeLogger_.start("solveLeastSquares_InequalityConstraints_ActiveSet", __FUNCTION__, __LINE__);
        solveLeastSquares_InequalityConstraints_ActiveSet();
        timeLogger_.stop("solveLeastSquares_InequalityConstraints_ActiveSet", __FUNCTION__, __LINE__);
    }else if(config_.inequality_constraints_solver == "LBFGSpp"){
        timeLogger_.start("solveLeastSquares_InequalityConstraints_BFGS", __FUNCTION__, __LINE__);
        solveLeastSquares_InequalityConstraints_BFGS(); 
        timeLogger_.stop("solveLeastSquares_InequalityConstraints_BFGS", __FUNCTION__, __LINE__);
    }else if(config_.inequality_constraints_solver == "ifopt"){
        timeLogger_.start("solveLeastSquares_InequalityConstraints_IFOPT", __FUNCTION__, __LINE__);
        solveLeastSquares_InequalityConstraints_IFOPT(); 
        timeLogger_.stop("solveLeastSquares_InequalityConstraints_IFOPT", __FUNCTION__, __LINE__);
    }else{
        RCLCPP_INFO_STREAM(get_logger(), "The least squares solver is not specified well. Skip ICP for LO.");
    }

    // if(config_.turn_on_qp_ineq_constraints_active_set){
    //     timeLogger_.start("solveLeastSquares_InequalityConstraints_ActiveSet", __FUNCTION__, __LINE__);
    //     solveLeastSquares_InequalityConstraints_ActiveSet();
    //     timeLogger_.stop("solveLeastSquares_InequalityConstraints_ActiveSet", __FUNCTION__, __LINE__);
    // }else if(config_.bfgs_on_inequality_constraint){
    //     timeLogger_.start("solveLeastSquares_InequalityConstraints_BFGS", __FUNCTION__, __LINE__);
    //     solveLeastSquares_InequalityConstraints_BFGS(); 
    //     timeLogger_.stop("solveLeastSquares_InequalityConstraints_BFGS", __FUNCTION__, __LINE__);
    // }else if(config_.ifopt_on_inequality_constraint){
    //     timeLogger_.start("solveLeastSquares_InequalityConstraints_IFOPT", __FUNCTION__, __LINE__);
    //     solveLeastSquares_InequalityConstraints_IFOPT(); 
    //     timeLogger_.stop("solveLeastSquares_InequalityConstraints_IFOPT", __FUNCTION__, __LINE__);
    // }else{
    //     timeLogger_.start("solveLeastSquares", __FUNCTION__, __LINE__);
    //     solveLeastSquares();    // unconstrained least squares
    //     timeLogger_.stop("solveLeastSquares", __FUNCTION__, __LINE__);
    // }
    // update t_w_bCurrKf_ and q_w_bCurrKf_

    if (debug_print_lines_in_run) RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    if(!config_.fix_lidar_odom_covariance){
        timeLogger_.start("computeCovarianceLO", __FUNCTION__, __LINE__);
        computeCovarianceLO();
        timeLogger_.stop("computeCovarianceLO", __FUNCTION__, __LINE__);
    }

    // timeLogger_.start("degeneracyAnalysisLO", __FUNCTION__, __LINE__);
    // degeneracyAnalysisLO();
    // timeLogger_.stop("degeneracyAnalysisLO", __FUNCTION__, __LINE__);

    if (debug_print_lines_in_run) RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // Update LO relative transformation resulted from the current Lidar Odometry ICP
    timeLogger_.start("updatePoseLO", __FUNCTION__, __LINE__);
    updatePoseLO();
    timeLogger_.stop("updatePoseLO", __FUNCTION__, __LINE__);
    // update t_bPrevKf_bCurrKf_lo_ and q_bPrevKf_bCurrKf_lo_
    //  tu plug relative LO to the factor graph

    if (debug_print_lines_in_run) RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // Compute Fixed-Lag Smoothing
    if(config_.turn_on_factor_graph){
        bool isImuPreintegrationReady = (imuIntegrator_->deltaTij() > TIME_EPS);
        if(isImuPreintegrationReady && isFGInitialized_){ 
            performFixedLagSmoothing();
        }else{  // if Imu Preintegration is not ready, skip factor graph optimization and perform LO only in the current process
            RCLCPP_INFO_STREAM(get_logger(), "Imu preintegrator is not ready or state is not initialized.");
            // RCLCPP_INFO_STREAM(get_logger(), "isImuPreintegrationReady: " << isImuPreintegrationReady);
            // RCLCPP_INFO_STREAM(get_logger(), "isFGInitialized_: " << isFGInitialized_);
            RCLCPP_INFO_STREAM(get_logger(), " ->  Skip fixed lag smoothing and reset factor graph and perform LO only for the current process.");
            isFGInitialized_ = false;   // It's necessary to re-initialize the factor graph with the latest updated state
        }
    }

    if (debug_print_lines_in_run) RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    //  ----- post-process ----- 
    timeLogger_.start("transformPointCloudInWorldFrame", __FUNCTION__, __LINE__);
    transformPointCloudInWorldFrame();
    timeLogger_.stop("transformPointCloudInWorldFrame", __FUNCTION__, __LINE__);

    if (debug_print_lines_in_run) RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // Publish ROS2 message for visualization
    publishOdometry();
    publishCloud();
    publishTf();

    if (debug_print_lines_in_run) RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // Add the current cloud to the map
    timeLogger_.start("updateCloudMap", __FUNCTION__, __LINE__);
    updateCloudMap();
    timeLogger_.stop("updateCloudMap", __FUNCTION__, __LINE__);

    if (debug_print_lines_in_run) RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // Update state variables and clear the current process
    updateState();

    if (debug_print_lines_in_run) RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    clearProcess();

    // if set this main process as timer callback, we need to make sure there is not race condition
    if (config_.set_main_process_timer)
        isProcessing_.store(false);

    timeLogger_.stop("run", __FUNCTION__, __LINE__);
}

void Frontend::imuHandler(const sensor_msgs::msg::Imu::SharedPtr msgIn)
{
    // store incomming imu data
    mtxImu_.lock();
    sensor_msgs::msg::Imu imu=*msgIn;
    imuMsgBuffer_.push_back(imu);
    mtxImu_.unlock();
}

void Frontend::cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msgIn)
{
    // store incomming point cloud data
    mtxCloud_.lock();
    sensor_msgs::msg::PointCloud2 cloud=*msgIn;
    cloudMsgBuffer_.push_back(cloud);
    mtxCloud_.unlock();

    // if not call the main process (run()) in timer, call run() every time receiving point cloud msg
    if (!config_.set_main_process_timer){
        run();
    }
}

void Frontend::transformPointCloudInWorldFrame()
{
    if(debug_print_published_pose){
        RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
        RCLCPP_INFO_STREAM(get_logger(), "t_w_bCurrKf_: " << t_w_bCurrKf_.transpose());
        RCLCPP_INFO_STREAM(get_logger(), "q_w_bCurrKf_: " << q_w_bCurrKf_.coeffs().transpose());
    }

    // Only add downsampled cloud to map
    if(config_.use_voxel_map){
        for (auto& ptEig: cloudScanCurrDsToMap_vecEigen_){
            // Eigen::Vector3d t_b_pt(point.x, point.y, point.z);
            Eigen::Vector3d t_w_pt = q_w_bCurrKf_ * ptEig + t_w_bCurrKf_;

            cloudScanCurrInWorld_vecEigen_.push_back(t_w_pt);

            PointType ptInW;
            ptInW.x = t_w_pt.x();
            ptInW.y = t_w_pt.y();
            ptInW.z = t_w_pt.z();
            cloudCurrentScanInWorld_->points.push_back(ptInW);
        }
    }else{
        for (auto point: cloudScanCurrDs_->points){
            Eigen::Vector3d t_b_pt(point.x, point.y, point.z);
            Eigen::Vector3d t_w_pt = q_w_bCurrKf_ * t_b_pt + t_w_bCurrKf_;

            PointType ptInW;
            ptInW.x = t_w_pt.x();
            ptInW.y = t_w_pt.y();
            ptInW.z = t_w_pt.z();
            cloudCurrentScanInWorld_->points.push_back(ptInW);
        }
    }
}

void Frontend::updateCloudMap() 
{
    if(config_.use_voxel_map){
        // for debugging
        if (debug_print_num_point_in_voxel_map){
            RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__ << " before update: voxelMap.size() = " << voxelMap_.Pointcloud().size());
            RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__ << "             added cloud.size() = " << cloudScanCurrInWorld_vecEigen_.size());
        }
        voxelMap_.Update(cloudScanCurrInWorld_vecEigen_, t_w_bCurrKf_);
        // voxelMap_.CheckVoxelIds(); // for debugging 

        // for debugging
        if (debug_print_num_point_in_voxel_map){
            RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__ << " after update: voxelMap.size() = " << voxelMap_.Pointcloud().size());
        }
    
        // for debugging
        if(debug_dump_log_file_voxel_ids)
            voxelMap_.DumpVoxelCorrdinates(timeCurrScanBeg_);
    }else{
        pcl::PointCloud<PointType>::Ptr cloud;
        cloud.reset(new pcl::PointCloud<PointType>());
        // swap (=move) point cloud for fast computation
        std::swap(cloudCurrentScanInWorld_, cloud);

        cloudFramesMapGlobal_.push_back(cloud);
    }
}

void Frontend::publishOdometry()
{
    if(debug_print_published_pose)
    {
        RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);\
        RCLCPP_INFO_STREAM(get_logger(), "t_w_bCurrKf_: " << t_w_bCurrKf_.transpose());
        RCLCPP_INFO_STREAM(get_logger(), "q_w_bCurrKf_: " << q_w_bCurrKf_.coeffs().transpose());
    }

    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = ODOM_FRAME;
    odom.header.stamp.sec = (int32_t)timeCurrScanEnd_;
    odom.header.stamp.nanosec = (uint32_t)((timeCurrScanEnd_ - (int32_t)timeCurrScanEnd_) * 1e+9f);
    odom.child_frame_id = BASE_FRAME;

    odom.pose.pose.position.x = t_w_bCurrKf_.x();
    odom.pose.pose.position.y = t_w_bCurrKf_.y();
    odom.pose.pose.position.z = t_w_bCurrKf_.z();
    odom.pose.pose.orientation.w = q_w_bCurrKf_.w();
    odom.pose.pose.orientation.x = q_w_bCurrKf_.x();
    odom.pose.pose.orientation.y = q_w_bCurrKf_.y();
    odom.pose.pose.orientation.z = q_w_bCurrKf_.z();

    pubOdom_->publish(odom);
}

void Frontend::publishCloud() 
{
    sensor_msgs::msg::PointCloud2 cloud;
    pcl::toROSMsg(*cloudCurrentScanInWorld_, cloud);
    cloud.header.stamp.sec = (int32_t)timeCurrScanEnd_;
    cloud.header.stamp.nanosec = (uint32_t)((timeCurrScanEnd_ - (int32_t)timeCurrScanEnd_) * 1e+9f);
    cloud.header.frame_id = MAP_FRAME;
    // The point cloud transformation is only IMU->ODOM
    // It should be theoretically IMU->BASE->ODOM->MAP
    // But in this lo, IMU=BASE->ODOM=MAP so only IMU->ODOM transformation is okay

    pubCloud_->publish(cloud);
}

// This function is just for ROS2 Rviz visualization (not important)
void Frontend::publishTf()
{
    if(debug_print_published_pose)
    {
        RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);\
        RCLCPP_INFO_STREAM(get_logger(), "t_w_bCurrKf_: " << t_w_bCurrKf_.transpose());
        RCLCPP_INFO_STREAM(get_logger(), "q_w_bCurrKf_: " << q_w_bCurrKf_.coeffs().transpose());
    }

    // Publish ODOM->BASE tf
    // This should take into account MAP->ODOM to make this more general
    geometry_msgs::msg::TransformStamped tf_odom_base;
    // tf_odom_base.header.stamp = get_clock()->now();    // keyframe time (sim time)
    tf_odom_base.header.stamp.sec = (int32_t)timeCurrScanEnd_;
    tf_odom_base.header.stamp.nanosec = (uint32_t)((timeCurrScanEnd_ - (int32_t)timeCurrScanEnd_) * 1e+9f);
    tf_odom_base.header.frame_id = ODOM_FRAME;
    tf_odom_base.child_frame_id = BASE_FRAME; // BASE_FRAME = IMU_FRAME in this livo
    tf_odom_base.transform.translation.x = t_w_bCurrKf_.x();
    tf_odom_base.transform.translation.y = t_w_bCurrKf_.y();
    tf_odom_base.transform.translation.z = t_w_bCurrKf_.z();
    // tf_odom_base.transform.rotation = tf2::toMsg(q_w_bCurrKf_.normalized());
    tf_odom_base.transform.rotation.w = q_w_bCurrKf_.w();
    tf_odom_base.transform.rotation.x = q_w_bCurrKf_.x();
    tf_odom_base.transform.rotation.y = q_w_bCurrKf_.y();
    tf_odom_base.transform.rotation.z = q_w_bCurrKf_.z();
    // tfBroadcaster_->sendTransform(tf_odom_base);
    // t_w_bCurrKf_ and q_w_bCurrKf_ are only ODOM->IMU by definition in the code
    // It should be theoretically ODOM->BASE (because this should represent odometry)
    // But in this livo, MAP=ODOM->BASE=IMU so only IMU->ODOM = BASE->ODOM transformation
    // Skipping unnecessary computation but keeping clarity

    // should be: in the future
    // Eigen::isometry3d tCur
    // geometry_msgs::msg::TransformStamped ts;
    // tf2::convert(tCur, ts);

    // Publish MAP->ODOM tf (constant)
    geometry_msgs::msg::TransformStamped tf_map_odom;
    // tf_map_odom.header.stamp = get_clock()->now();
    tf_map_odom.header.stamp.sec = (int32_t)timeCurrScanEnd_;
    tf_map_odom.header.stamp.nanosec = (uint32_t)((timeCurrScanEnd_ - (int32_t)timeCurrScanEnd_) * 1e+9f);
    tf_map_odom.header.frame_id = MAP_FRAME;
    tf_map_odom.child_frame_id = ODOM_FRAME;
    tf_map_odom.transform.translation.x = t_MAP_ODOM.x();
    tf_map_odom.transform.translation.y = t_MAP_ODOM.y();
    tf_map_odom.transform.translation.z = t_MAP_ODOM.z();
    // tf_map_odom.transform.rotation = tf2::toMsg(q_MAP_ODOM.normalized());
    tf_map_odom.transform.rotation.w = q_MAP_ODOM.w();
    tf_map_odom.transform.rotation.x = q_MAP_ODOM.x();
    tf_map_odom.transform.rotation.y = q_MAP_ODOM.y();
    tf_map_odom.transform.rotation.z = q_MAP_ODOM.z();
    // tfBroadcaster_->sendTransform(tf_map_odom);

    tfBroadcaster_->sendTransform(std::vector{tf_map_odom, tf_odom_base});
    // MAP->ODOM transformation is always assumed to be constant
}

// This function is just for ROS2 Rviz visualization (not important)
void Frontend::publishStaticTf()
{
    // Publish BASE->IMU tf (constant)
    geometry_msgs::msg::TransformStamped tf_base_imu;
    tf_base_imu.header.stamp = get_clock()->now();
    tf_base_imu.header.frame_id = BASE_FRAME;
    tf_base_imu.child_frame_id = IMU_FRAME;
    tf_base_imu.transform.translation.x = t_BASE_IMU.x();
    tf_base_imu.transform.translation.y = t_BASE_IMU.y();
    tf_base_imu.transform.translation.z = t_BASE_IMU.z();
    // tf_base_imu.transform.rotation = tf2::toMsg(q_BASE_IMU);
    tf_base_imu.transform.rotation.w = q_BASE_IMU.w();
    tf_base_imu.transform.rotation.x = q_BASE_IMU.x();
    tf_base_imu.transform.rotation.y = q_BASE_IMU.y();
    tf_base_imu.transform.rotation.z = q_BASE_IMU.z();
    // tfBroadcaster_->sendTransform(tfMsg);
    // BASE->IMU transformation is always assumed to be constant

    // Publish BASE->LIDAR tf (constant)
    geometry_msgs::msg::TransformStamped tf_base_lidar;
    tf_base_lidar.header.stamp = get_clock()->now();
    Eigen::Vector3d t_base_lidar = t_BASE_IMU + q_BASE_IMU * t_IMU_LIDAR;
    Eigen::Quaterniond q_base_lidar = (q_BASE_IMU * q_IMU_LIDAR).normalized();
    tf_base_lidar.header.frame_id = BASE_FRAME;
    tf_base_lidar.child_frame_id = LIDAR_FRAME;
    tf_base_lidar.transform.translation.x = t_base_lidar.x();
    tf_base_lidar.transform.translation.y = t_base_lidar.y();
    tf_base_lidar.transform.translation.z = t_base_lidar.z();
    // tf_base_lidar.transform.rotation = tf2::toMsg(q_base_lidar);
    tf_base_lidar.transform.rotation.w = q_base_lidar.w();
    tf_base_lidar.transform.rotation.x = q_base_lidar.x();
    tf_base_lidar.transform.rotation.y = q_base_lidar.y();
    tf_base_lidar.transform.rotation.z = q_base_lidar.z();
    // tfBroadcaster_->sendTransform(tfMsg);

    tfStaticBroadcaster_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{tf_base_imu, tf_base_lidar});
}

void Frontend::clearProcess()
{
    imuMsgKfWindow_.clear();
    cloudKfWindow_->clear();
    imuPoseTimeline_.clear();

    cloudScanCurr_->clear();
    cloudScanCurrDs_->clear();
    cloudScanCurrDs_vecEigen_.clear();
    cloudScanCurrDsToMap_vecEigen_.clear();
    cloudCurrentScanInWorld_->clear();
    cloudScanCurrInWorld_vecEigen_.clear();

    pointsTimestampAvailable_ = false;
    isCurrFrameLidarOnlyEstimation_ = false;
}

void Frontend::updateState() 
{
    // for sanity check for imu and lidar relative pose estimation
    // logRelativePoseEstimateEachSensor();

    // Compute the current relative pose for the following computation 
    const Eigen::Quaterniond q_bPrevKf_bCurrKf = (q_w_bPrevKf_.inverse() * q_w_bCurrKf_).normalized();
    const Eigen::Vector3d t_bPrevKf_bCurrKf = q_w_bPrevKf_.inverse() * (t_w_bCurrKf_ - t_w_bPrevKf_);

    // Update stateCurrKf_ when not initialized yet or running lidar only estimation
    // Compute the velocity to update velocity in stateCurrKf_
    Eigen::Vector3d bP_v_bPrevKf_bCurrKf = Eigen::Vector3d::Zero();
    Eigen::Vector3d w_v_bPrevKf_bCurrKf = Eigen::Vector3d::Zero();
    if(timeCurrScanEnd_ - timePrevScanEnd_ > TIME_EPS){
        bP_v_bPrevKf_bCurrKf = (t_bPrevKf_bCurrKf / (timeCurrScanEnd_ - timePrevScanEnd_));
        w_v_bPrevKf_bCurrKf = q_w_bPrevKf_ * bP_v_bPrevKf_bCurrKf;
    }
    if(!isStateInitialized_){
        if(stateCurrKf_.equals(gtsam::NavState()) && (!w_v_bPrevKf_bCurrKf.isApprox(Eigen::Vector3d::Zero(), 1e-9))){
            getGtsamFromEigen(q_w_bCurrKf_, t_w_bCurrKf_, w_v_bPrevKf_bCurrKf, stateCurrKf_);
            isStateInitialized_ = true;
        }
    }else if(isCurrFrameLidarOnlyEstimation_){
        getGtsamFromEigen(q_w_bCurrKf_, t_w_bCurrKf_, w_v_bPrevKf_bCurrKf, stateCurrKf_);
    }

    logFactorGraphState();

    // Update time sequence variables
    timePrev2ScanEnd_ = timePrevScanEnd_;  // just to printing and debugging
    timePrevScanEnd_ = timeCurrScanEnd_;
    timeCurrScanBeg_ = timePrevScanEnd_;
    // timeCurrScanEnd_ is to be updated in synchronizeMeasurements()

    // Update the past reference trajectory to be used in LO initial guess and deskewing with constant velocity
    q_bPrev2Kf_bPrevKf_ = q_bPrevKf_bCurrKf;
    t_bPrev2Kf_bPrevKf_ = t_bPrevKf_bCurrKf;

    // Updata the pose variables
    q_w_bPrevKf_ = q_w_bCurrKf_;
    t_w_bPrevKf_ = t_w_bCurrKf_;

    // Update GTSAM state variables
    statePrevKf_ = stateCurrKf_;
    biasPrevKf_ = biasCurrKf_;
    stateCovPrevKf_ = stateCovCurrKf_;

    // Update the imu preintegrator
    imuIntegrator_->resetIntegrationAndSetBias(biasCurrKf_);
}

// void Frontend::initializeFG() 
bool Frontend::initializeFG() 
{
    if (statePrevKf_.equals(gtsam::NavState())){
        RCLCPP_INFO(this->get_logger(), "State variable is not initialized. Skip the factor graph initialization.");
        return false;
    }

    resetSmoother();

    key_ = 0;

    // Set a prior velocity factor to the first state
    // graphFactors_.addPrior(X(key_), posePrevKf_, priorPoseNoise_);
    graphFactors_.addPrior(X(key_), statePrevKf_.pose(), priorPoseNoise_);

    // Set a prior velocity factor to the first state
    graphFactors_.addPrior(V(key_), statePrevKf_.velocity(), priorVelNoise_);

    // Set a prior bias factor to the first state
    // graphFactors_.addPrior(B(key_), biasPrevKf_, priorBiasNoise_);
    graphFactors_.addPrior(B(key_), biasPrevKf_, priorBiasNoise_);

    // Set initial guess values for the first state
    graphValues_.insert(X(key_), statePrevKf_.pose());
    graphValues_.insert(V(key_), statePrevKf_.velocity());
    graphValues_.insert(B(key_), biasPrevKf_);

    // Set key timestamps for fixed-lag smoothing
    keyTimestamps_[X(key_)] = timePrevScanEnd_;
    keyTimestamps_[V(key_)] = timePrevScanEnd_;
    keyTimestamps_[B(key_)] = timePrevScanEnd_;
    // could be done in the following way
    // for (auto const &val: graphValues_)
    // {
    //     keyTimestamps_[val.key] = initTime; // following gtsam_4-3/examples/FixedLagSmootherExample.cpp
    // }

    // Update the optimizer once
    // batchFixedLagSmoother_->update(graphFactors_, graphValues_, keyTimestamps_);

    // isam2FixedLagSmoother_->update(graphFactors_, graphValues_, keyTimestamps_);
    // for(size_t i = 1; i < 2; ++i) { // Optionally perform multiple iSAM2 iterations
    //     isam2FixedLagSmoother_->update();
    // }
    if(config_.fixed_lag_smoother_batch_or_incremental == "batch"){
        batchFixedLagSmoother_->update(graphFactors_, graphValues_, keyTimestamps_);
    }else if(config_.fixed_lag_smoother_batch_or_incremental == "incremental"){
        isam2FixedLagSmoother_->update(graphFactors_, graphValues_, keyTimestamps_);
        for(size_t i = 1; i < 2; ++i) { // Optionally perform multiple iSAM2 iterations
            isam2FixedLagSmoother_->update();
        }
    }
    // ref: /home/ysugano/code_review/gtsam_4-3/examples/FixedLagSmootherExample.cpp

    // // just to check if marginals() works
    // gtsam::Values result;
    // if(config_.fixed_lag_smoother_batch_or_incremental == "batch"){
    //     result = batchFixedLagSmoother_->calculateEstimate();
    // }else if(config_.fixed_lag_smoother_batch_or_incremental == "incremental"){
    //     result = isam2FixedLagSmoother_->calculateEstimate();
    // }  
    // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
    // gtsam::Marginals marginals(graphFactors_, result, gtsam::Marginals::QR);
    // marginals.print();
    // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
    // // stateCovCurrKf_ = marginals.marginalCovariance(X(key_));


    graphFactors_.resize(0);
    graphValues_.clear();
    keyTimestamps_.clear();

    imuIntegrator_->resetIntegrationAndSetBias(biasPrevKf_);

    key_ = 1;

    return true;
}


void Frontend::resetSmoother() 
{
    // If it's an incremental smoother, we need the followings:
    // gtsam::ISAM2Params optParameters;
    // optParameters.relinearizeThreshold = 0.1;
    // optParameters.relinearizeSkip = 1;
    // optimizer = gtsam::ISAM2(optParameters);

    // batchFixedLagSmoother_ = std::make_shared<gtsam::BatchFixedLagSmoother>(config_.fixed_lag);
        
    // ISAM2Params parameters;
    // parameters.relinearizeThreshold = 0.0; // Set the relin threshold to zero such that the batch estimate is recovered
    // parameters.relinearizeSkip = 1; // Relinearize every time  
    // isam2FixedLagSmoother_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(config_.fixed_lag, parameters);

    if(config_.fixed_lag_smoother_batch_or_incremental == "batch"){
        batchFixedLagSmoother_ = std::make_shared<gtsam::BatchFixedLagSmoother>(config_.fixed_lag);
    }else if(config_.fixed_lag_smoother_batch_or_incremental == "incremental"){
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.0; // Set the relin threshold to zero such that the batch estimate is recovered
        parameters.relinearizeSkip = 1; // Relinearize every time  
        isam2FixedLagSmoother_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(config_.fixed_lag, parameters);
    }    

    gtsam::NonlinearFactorGraph newGraphFactors;
    graphFactors_ = newGraphFactors;

    gtsam::Values newGraphValues;
    graphValues_ = newGraphValues;

    // refresh the member variable gtsam::NonlinearFactorGraph fixedLagValues
    gtsam::FixedLagSmoother::KeyTimestampMap newGraphTimestamps;
    keyTimestamps_ = newGraphTimestamps;
}


void Frontend::performFixedLagSmoothing() 
{
    // Note for the correct process
    // 1. velocity set (statePrevKf set) based on LO
    // 2. Initialize FG
    // 2. propagate Imu -> imupreintegrator set
    // 3. perform Fixed lag Smoothing

    getGtsamFromEigen(q_bPrevKf_bCurrKf_lo_, t_bPrevKf_bCurrKf_lo_, T_bPrevKf_bCurrKf_lo_);

    if(debug_print_keyframe_id) RCLCPP_INFO_STREAM(get_logger(), "Keyframe ID: " << key_);
    if(debug_print_lo_relative_pose_fg) RCLCPP_INFO_STREAM(get_logger(), "T_bPrevKf_bCurrKf_lo_: " << T_bPrevKf_bCurrKf_lo_); // print T_bPrevKf_bCurrKf_lo_

    // Add imu factor
    const gtsam::PreintegratedImuMeasurements &preintImu = dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(*imuIntegrator_);
    gtsam::ImuFactor imuFactor(X(key_-1), V(key_-1), X(key_), V(key_), B(key_-1), preintImu);
    graphFactors_.add(imuFactor);
    if(debug_print_PreintegratedImuMeasurements_fg) preintImu.print("debug_print_PreintegratedImuMeasurements_fg");
    if(debug_print_imu_factor_fg) imuFactor.print("imuFactor: ");
    imuPreintCovariance_log_ = imuIntegrator_->preintMeasCov(); // for logging

    // Add imu bias between factor
    gtsam::noiseModel::Diagonal::shared_ptr imuBiasNoise = gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegrator_->deltaTij()) * noiseModelBetweenBias_);
    auto imuBiasFactor = gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key_-1), B(key_), gtsam::imuBias::ConstantBias(), imuBiasNoise);
    // auto imuBiasFactor = gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
    //         B(key_-1), B(key_), gtsam::imuBias::ConstantBias(),
    //         gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegrator_->deltaTij()) * noiseModelBetweenBias_));
    graphFactors_.add(imuBiasFactor); // imu bias factor just adds covariance with zero-mean between the previous bias and the current bias
    if(debug_print_imu_bias_factor_fg) imuBiasFactor.print("imuBiasFactor: ");
    imuBiasCovariance_log_ = imuBiasNoise->covariance(); // for logging

    // Add LO between factor (or Prior factor for debugging)
    if(!config_.use_lo_prior_factor_wo_between_factor){
        gtsam::BetweenFactor<gtsam::Pose3> lidarOdomFactor;
        // use lo between factor
        if(config_.fix_lidar_odom_covariance){
            // gtsam::BetweenFactor<gtsam::Pose3> lidarOdomFactor(X(key_-1), X(key_), T_bPrevKf_bCurrKf_lo_, constLidarOdomNoise_); // this correctionNoise should be based on ICP score
            lidarOdomFactor = gtsam::BetweenFactor<gtsam::Pose3>(X(key_-1), X(key_), T_bPrevKf_bCurrKf_lo_, constLidarOdomNoise_);
        } else {
            lidarOdomFactor = gtsam::BetweenFactor<gtsam::Pose3>(X(key_-1), X(key_), T_bPrevKf_bCurrKf_lo_, lidarOdomNoise_);
            // gtsam::BetweenFactor<gtsam::Pose3> lidarOdomFactor(X(key_-1), X(key_), T_bPrevKf_bCurrKf_lo_, lidarOdomNoise_);
        }
        graphFactors_.add(lidarOdomFactor);
        if(debug_print_lo_factor_fg) lidarOdomFactor.print("lidarOdomFactor: ");
    }else{
        // use lo prior factor
        // gtsam::Pose3 poseCurrLO;
        // getGtsamFromEigen(q_w_bCurrKf_, t_w_bCurrKf_, poseCurrLO);
        // gtsam::PriorFactor<gtsam::Pose3> lidarOdomFactor(X(key_), poseCurrLO ,constLidarOdomNoise_); // this correctionNoise should be based on ICP score
        gtsam::PriorFactor<gtsam::Pose3> lidarOdomFactor;
        getGtsamFromEigen(q_w_bCurrKf_, t_w_bCurrKf_, T_w_bCurrKf_lo_);
        if(config_.fix_lidar_odom_covariance){
            // gtsam::PriorFactor<gtsam::Pose3> lidarOdomFactor(X(key_), T_w_bCurrKf_lo_ ,constLidarOdomNoise_); // this correctionNoise should be based on ICP score
            lidarOdomFactor = gtsam::PriorFactor<gtsam::Pose3>(X(key_), T_w_bCurrKf_lo_ ,constLidarOdomNoise_);
        } else {
            // gtsam::PriorFactor<gtsam::Pose3> lidarOdomFactor(X(key_), T_w_bCurrKf_lo_, lidarOdomNoise_);
            lidarOdomFactor = gtsam::PriorFactor<gtsam::Pose3>(X(key_), T_w_bCurrKf_lo_, lidarOdomNoise_);
        }
        graphFactors_.add(lidarOdomFactor);
        if(debug_print_lo_factor_fg) lidarOdomFactor.print("lidarOdomFactor: ");

        RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
        RCLCPP_INFO_STREAM(get_logger(), "T_w_bCurrKf_lo_: " << T_w_bCurrKf_lo_);
    }
RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
    RCLCPP_INFO_STREAM(get_logger(), "icpHessianLatest_log_: " << icpHessianLatest_log_);
    RCLCPP_INFO_STREAM(get_logger(), "lidarOdomNoise_: " << lidarOdomNoise_);

    if(config_.fix_lidar_odom_covariance){
        lidarOdomCovariance_log_ = constLidarOdomNoise_->covariance(); // for logging
    } else {
RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
        lidarOdomCovariance_log_ = lidarOdomNoise_->covariance(); // for logging
    }

RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // NEED TO CHANGE
    // Now we compute both lidar and imu -based init guess for the current keyframe for debugging and analysis
    // But eventually we need to compute only one of them to save computation
    // init guess based on lidar
    gtsam::Pose3 poseCurrLO;
    getGtsamFromEigen(q_w_bCurrKf_, t_w_bCurrKf_, poseCurrLO);
    Eigen::Vector3d bP_v_bPrevKf_bCurrKf_lo = t_bPrevKf_bCurrKf_lo_ / (timeCurrScanEnd_ - timePrevScanEnd_);
    Eigen::Vector3d w_v_bPrevKf_bCurrKf_lo = q_w_bPrevKf_ * bP_v_bPrevKf_bCurrKf_lo;
    gtsam::Velocity3 velCurrLO = w_v_bPrevKf_bCurrKf_lo; // this should be in world frame
    stateCurrKf_initGuess_lo_log_ = gtsam::NavState(poseCurrLO, velCurrLO);
    biasCurrKf_initGuess_lo_log_ = biasPrevKf_;
    // init guess based on imu (also compute imu propagated relative pose for debugging)
    gtsam::NavState stateCurrKfProp;
    if(imuIntegrator_->deltaTij() > TIME_EPS){
        stateCurrKfProp = imuIntegrator_->predict(statePrevKf_, biasPrevKf_);
        stateCurrKf_initGuess_imu_log_ = stateCurrKfProp;
        biasCurrKf_initGuess_imu_log_ = biasPrevKf_;
        Eigen::Vector3d t_w_bCurrKf_imu;
        Eigen::Quaterniond q_w_bCurrKf_imu;
        getEigenFromGtsam(stateCurrKfProp, q_w_bCurrKf_imu, t_w_bCurrKf_imu);
        Eigen::Quaterniond q_bPrevKf_bCurrKf_imu = (q_w_bPrevKf_.inverse() * q_w_bCurrKf_imu).normalized();
        Eigen::Vector3d t_bPrevKf_bCurrKf_imu = q_w_bPrevKf_.inverse() * (t_w_bCurrKf_imu - t_w_bPrevKf_);
        getGtsamFromEigen(q_bPrevKf_bCurrKf_imu, t_bPrevKf_bCurrKf_imu, T_bPrevKf_bCurrKf_imu_log_);
    }

RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // Set initial values for the new keyframe state
    if (config_.factor_graph_init_guess_source == "lidar"){
        // gtsam::Pose3 poseCurrLO;
        // getGtsamFromEigen(q_w_bCurrKf_, t_w_bCurrKf_, poseCurrLO);
        graphValues_.insert(X(key_), poseCurrLO);

        // Eigen::Vector3d bP_v_bPrevKf_bCurrKf_lo = t_bPrevKf_bCurrKf_lo_ / (timeCurrScanEnd_ - timePrevScanEnd_);
        // Eigen::Vector3d w_v_bPrevKf_bCurrKf_lo = q_w_bPrevKf_ * bP_v_bPrevKf_bCurrKf_lo;
        // gtsam::Velocity3 velCurrLO = w_v_bPrevKf_bCurrKf_lo; // this should be in world frame
        graphValues_.insert(V(key_), velCurrLO);

    }else if(config_.factor_graph_init_guess_source == "imu" && imuIntegrator_->deltaTij() > TIME_EPS){
        // gtsam::NavState stateCurrKfProp = imuIntegrator_->predict(statePrevKf_, biasPrevKf_);
        graphValues_.insert(X(key_), stateCurrKfProp.pose());
        graphValues_.insert(V(key_), stateCurrKfProp.velocity());

        if(debug_print_imu_prop_state_fg) RCLCPP_INFO_STREAM(get_logger(), "imu propagated current state: " << stateCurrKfProp);
    }else 
        RCLCPP_INFO_STREAM(get_logger(), "config_.factor_graph_init_guess_source is not properly set. Skip setting init guess for the current KF state in factor graph.");
    
    graphValues_.insert(B(key_), biasPrevKf_);

RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // Set the measured time for each key(X,V,B), following gtsam_4-3/examples/FixedLagSmootherExample.cpp
    keyTimestamps_[X(key_)] = timeCurrScanEnd_;  // it should align with the scan ending time 
    keyTimestamps_[V(key_)] = timeCurrScanEnd_;
    keyTimestamps_[B(key_)] = timeCurrScanEnd_;

    if(debug_print_num_factors_values){
        // Print some intermediate statistics: from IncrementalFixedLagSmootherExample.cpp
        // std::cout << "Before update - Graph has " << batchFixedLagSmoother_->getFactors().size() << " factors, " << batchFixedLagSmoother_->getFactors().nrFactors() << " nr factors." << std::endl;
        // std::cout << "New factors: " << graphFactors_.size() << ", New values: " << graphFactors_.size() << std::endl;

        // std::cout << "Before update - Graph has " << isam2FixedLagSmoother_->getFactors().size() << " factors, " << isam2FixedLagSmoother_->getFactors().nrFactors() << " nr factors." << std::endl;
        // std::cout << "New factors: " << graphFactors_.size() << ", New values: " << graphFactors_.size() << std::endl;

        if(config_.fixed_lag_smoother_batch_or_incremental == "batch"){
            std::cout << "Before update - Graph has " << batchFixedLagSmoother_->getFactors().size() << " factors, " << batchFixedLagSmoother_->getFactors().nrFactors() << " nr factors." << std::endl;
            std::cout << "New factors: " << graphFactors_.size() << ", New values: " << graphFactors_.size() << std::endl;
        }else if(config_.fixed_lag_smoother_batch_or_incremental == "incremental"){
            std::cout << "Before update - Graph has " << isam2FixedLagSmoother_->getFactors().size() << " factors, " << isam2FixedLagSmoother_->getFactors().nrFactors() << " nr factors." << std::endl;
            std::cout << "New factors: " << graphFactors_.size() << ", New values: " << graphFactors_.size() << std::endl;
        }
    }

RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // Solve the optimization
    bool success = false;
    try {
        // batchFixedLagSmoother_->update(graphFactors_, graphValues_, keyTimestamps_);     // following gtsam_4-3/examples/FixedLagSmootherExample.cpp
        // isam2FixedLagSmoother_->update(graphFactors_, graphValues_, keyTimestamps_);
        // for(size_t i = 1; i < 2; ++i) { // Optionally perform multiple iSAM2 iterations
        //     isam2FixedLagSmoother_->update();
        // }

        if(config_.fixed_lag_smoother_batch_or_incremental == "batch"){
            batchFixedLagSmoother_->update(graphFactors_, graphValues_, keyTimestamps_);     // following gtsam_4-3/examples/FixedLagSmootherExample.cpp
        }else if(config_.fixed_lag_smoother_batch_or_incremental == "incremental"){
            isam2FixedLagSmoother_->update(graphFactors_, graphValues_, keyTimestamps_);
            for(size_t i = 1; i < 2; ++i) { // Optionally perform multiple iSAM2 iterations
                isam2FixedLagSmoother_->update();
            }
        }      

        // any parameters for fixedLagSmoother?
        success = true;
    } catch (const gtsam::IndeterminantLinearSystemException &) {
        success = false;
        RCLCPP_WARN(this->get_logger(), "Update failed in the fixed lag smoother.");
    }    

    if(debug_print_num_factors_values){
        // you may not get expected results if you use the gtsam version lower than 4.3
        // std::cout << "After update - Graph has " << batchFixedLagSmoother_->getFactors().size()
        //         << " factors, " << batchFixedLagSmoother_->getFactors().nrFactors() << " nr factors." << std::endl;
        //         // size_t 	nrFactors () const -> return the number of non-null factors

        // std::cout << "After update - Graph has " << isam2FixedLagSmoother_->getFactors().size()
        //         << " factors, " << isam2FixedLagSmoother_->getFactors().nrFactors() << " nr factors." << std::endl;
        //         // size_t 	nrFactors () const -> return the number of non-null factors

        if(config_.fixed_lag_smoother_batch_or_incremental == "batch"){
            std::cout << "After update - Graph has " << batchFixedLagSmoother_->getFactors().size()
                    << " factors, " << batchFixedLagSmoother_->getFactors().nrFactors() << " nr factors." << std::endl;
                    // size_t 	nrFactors () const -> return the number of non-null factors
        }else if(config_.fixed_lag_smoother_batch_or_incremental == "incremental"){
            std::cout << "After update - Graph has " << isam2FixedLagSmoother_->getFactors().size()
                    << " factors, " << isam2FixedLagSmoother_->getFactors().nrFactors() << " nr factors." << std::endl;
                    // size_t 	nrFactors () const -> return the number of non-null factors
        }  
    }

RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

    // // Clear the factors and values
    // graphFactors_.resize(0);
    // graphValues_.clear();
    // keyTimestamps_.clear();

    // Update the previous state
    if (success) {
        // gtsam::Values result = batchFixedLagSmoother_->calculateEstimate();
        // // If only a single variable is needed, it is faster to call calculateEstimate(const KEY&).
        // gtsam::Values result = isam2FixedLagSmoother_->calculateEstimate();

        gtsam::Values result;
        if(config_.fixed_lag_smoother_batch_or_incremental == "batch"){
            result = batchFixedLagSmoother_->calculateEstimate();
        }else if(config_.fixed_lag_smoother_batch_or_incremental == "incremental"){
            result = isam2FixedLagSmoother_->calculateEstimate();
        }  

        // RCLCPP_INFO(this->get_logger(), "fixedLagKey:  %d", fixedLagKey);
        // RCLCPP_INFO(this->get_logger(), "result.size():  %ld", result.size());

        // poseCurrKf_ = result.at<gtsam::Pose3>(X(key_));
        // velCurrKf_ = result.at<gtsam::Velocity3>(V(key_));
        auto posCurrKf = result.at<gtsam::Pose3>(X(key_));
        auto velCurrKf = result.at<gtsam::Velocity3>(V(key_));
        biasCurrKf_ = result.at<gtsam::imuBias::ConstantBias>(B(key_));
        stateCurrKf_ = gtsam::NavState(posCurrKf, velCurrKf);

        RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
        result.print();
        RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
        graphFactors_.print();

        gtsam::NonlinearFactorGraph fullGraph;
        if (config_.fixed_lag_smoother_batch_or_incremental == "batch") {
            fullGraph = batchFixedLagSmoother_->getFactors();
        } else {
            fullGraph = isam2FixedLagSmoother_->getFactors();
        }

        // Should be in this way
        // For now, state covariance does not work so well for some reason.
        // We can try:
        //  - ImuFactor2
        //  - CombinedImuFactor
        //  - tuning LM parameters
        //  - tuning fixed_lag range
        //  But none of them have anything to do with graphFactors_ and result
        //  So, there would be something wrong with factors and results themselves
        RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
        // update the latest state covariance
        gtsam::Marginals marginals(fullGraph, result);
        // gtsam::Marginals marginals(graphFactors_, result, gtsam::Marginals::QR);
        RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
        stateCovCurrKf_.topLeftCorner<6,6>() = marginals.marginalCovariance(X(key_));
        stateCovCurrKf_.bottomRightCorner<3,3>() = marginals.marginalCovariance(V(key_));

        // stateCovCurrKf_ = marginals.marginalCovariance(X(key_));
        // ref: /home/ysugano/code_review/gtsam_4-3/examples/OdometryExample.cpp

        // So for now, just set the current state covariance as the initial state covariance and move on
        // stateCovCurrKf_.topLeftCorner<6,6>() = priorPoseNoise_->covariance();
        // stateCovCurrKf_.bottomRightCorner<3,3>() = priorVelNoise_->covariance();

        // imuIntegrator_->resetIntegrationAndSetBias(biasCurrKf_);

RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

        // update the current state estimate(q_w_bCurrKf_, t_w_bCurrKf_) with FG estimation
        getEigenFromGtsam(stateCurrKf_, q_w_bCurrKf_, t_w_bCurrKf_);

        // check the current pose, velocity, bias
        if(debug_print_estimated_state_fg){
            RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
            // RCLCPP_INFO_STREAM(get_logger(), "poseCurrKf_: " << poseCurrKf_);
            // RCLCPP_INFO_STREAM(get_logger(), "velCurrKf_: " << velCurrKf_);
            RCLCPP_INFO_STREAM(get_logger(), "stateCurrKf_.pose(): " << stateCurrKf_.pose());
            RCLCPP_INFO_STREAM(get_logger(), "stateCurrKf_.velocity(): " << stateCurrKf_.velocity());
            RCLCPP_INFO_STREAM(get_logger(), "biasCurrKf_: " << biasCurrKf_);
            RCLCPP_INFO_STREAM(get_logger(), "stateCurrKf_: " << stateCurrKf_);
            RCLCPP_INFO_STREAM(get_logger(), "t_w_bCurrKf_: " << t_w_bCurrKf_.transpose());
            RCLCPP_INFO_STREAM(get_logger(), "q_w_bCurrKf_: " << q_w_bCurrKf_.coeffs().transpose());
        }

        if(debug_print_key_timestamp_in_window){
            RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
            // RCLCPP_INFO_STREAM(get_logger(), "poseCurrKf_: " << poseCurrKf_);
            RCLCPP_INFO_STREAM(get_logger(), "stateCurrKf_.pose(): " << stateCurrKf_.pose());
            // for(const gtsam::FixedLagSmoother::KeyTimestampMap::value_type& key_timestamp: batchFixedLagSmoother_->timestamps()) {
            //     std::cout << std::setprecision(std::numeric_limits<double>::max_digits10) << "    Key: " << key_timestamp.first << "  Time: " << key_timestamp.second << std::endl;  // smootherBatch.timestamps() -> key_timestamp
            // }

            // for(const gtsam::FixedLagSmoother::KeyTimestampMap::value_type& key_timestamp: isam2FixedLagSmoother_->timestamps()) {
            //     std::cout << std::setprecision(std::numeric_limits<double>::max_digits10) << "    Key: " << key_timestamp.first << "  Time: " << key_timestamp.second << std::endl;  // smootherBatch.timestamps() -> key_timestamp
            // }
            if(config_.fixed_lag_smoother_batch_or_incremental == "batch"){
                for(const gtsam::FixedLagSmoother::KeyTimestampMap::value_type& key_timestamp: batchFixedLagSmoother_->timestamps()) {
                    std::cout << std::setprecision(std::numeric_limits<double>::max_digits10) << "    Key: " << key_timestamp.first << "  Time: " << key_timestamp.second << std::endl;  // smootherBatch.timestamps() -> key_timestamp
                }
            }else if(config_.fixed_lag_smoother_batch_or_incremental == "incremental"){
                for(const gtsam::FixedLagSmoother::KeyTimestampMap::value_type& key_timestamp: isam2FixedLagSmoother_->timestamps()) {
                    std::cout << std::setprecision(std::numeric_limits<double>::max_digits10) << "    Key: " << key_timestamp.first << "  Time: " << key_timestamp.second << std::endl;  // smootherBatch.timestamps() -> key_timestamp
                }
            }
        }
    }else{
        key_ = 0; // reset the key
        isFGInitialized_ = false; // need to initialize it again??
        RCLCPP_INFO_STREAM(get_logger(), "Smoothing failed. Reset factor graph");
        return;
    }

    // Clear the factors and values
    graphFactors_.resize(0);
    graphValues_.clear();
    keyTimestamps_.clear();

    key_++;
}

void Frontend::logRelativePoseEstimateEachSensor()
{
    stateLogger_.recordState("Lidar", timeCurrScanBeg_, q_bPrevKf_bCurrKf_lo_, t_bPrevKf_bCurrKf_lo_);
    // stateLogger_.recordState("Lidar", timeCurrScanEnd_, q_bPrevKf_bCurrKf_lo_, t_bPrevKf_bCurrKf_lo_);

    // gtsam::NavState identity;
    // gtsam::NavState imuPropState = imuIntegrator_->predict(identity, biasPrevKf_);
    gtsam::NavState imuPropStateCurrKf = imuIntegrator_->predict(statePrevKf_, biasPrevKf_);
    // Also need to convert it to a realtive pose in W
    Eigen::Vector3d t_w_bCurrKf_imu;
    Eigen::Quaterniond q_w_bCurrKf_imu;
    getEigenFromGtsam(imuPropStateCurrKf, q_w_bCurrKf_imu, t_w_bCurrKf_imu);
    Eigen::Quaterniond q_bPrevKf_bCurrKf_imu = (q_w_bPrevKf_.inverse() * q_w_bCurrKf_imu).normalized();
    Eigen::Vector3d t_bPrevKf_bCurrKf_imu = q_w_bPrevKf_.inverse() * (t_w_bCurrKf_imu - t_w_bPrevKf_);
 
    stateLogger_.recordState("Imu", timeCurrScanBeg_, q_bPrevKf_bCurrKf_imu, t_bPrevKf_bCurrKf_imu);
}

void Frontend::logFactorGraphState()
{
    if (!isStateInitialized_) return;

    double keyframeTimestamp = 0.0;
    // if(!batchFixedLagSmoother_->timestamps().empty()){
    //     keyframeTimestamp = std::prev(batchFixedLagSmoother_->timestamps().end())->second;
    //     RCLCPP_INFO_STREAM(get_logger(), "batchFixedLagSmoother_->timestamps().begin()->first: " << batchFixedLagSmoother_->timestamps().begin()->first);
    //     RCLCPP_INFO_STREAM(get_logger(), "std::prev(batchFixedLagSmoother_->timestamps().end())->first: " << std::prev(batchFixedLagSmoother_->timestamps().end())->first);
    // }

    // if(!isam2FixedLagSmoother_->timestamps().empty()){
    //     keyframeTimestamp = std::prev(isam2FixedLagSmoother_->timestamps().end())->second;
    //     RCLCPP_INFO_STREAM(get_logger(), "isam2FixedLagSmoother_->timestamps().begin()->first: " << isam2FixedLagSmoother_->timestamps().begin()->first);
    //     RCLCPP_INFO_STREAM(get_logger(), "std::prev(isam2FixedLagSmoother_->timestamps().end())->first: " << std::prev(isam2FixedLagSmoother_->timestamps().end())->first);
    // }

    if(config_.fixed_lag_smoother_batch_or_incremental == "batch"){
        if(!batchFixedLagSmoother_->timestamps().empty()){
            keyframeTimestamp = std::prev(batchFixedLagSmoother_->timestamps().end())->second;
            RCLCPP_INFO_STREAM(get_logger(), "batchFixedLagSmoother_->timestamps().begin()->first: " << batchFixedLagSmoother_->timestamps().begin()->first);
            RCLCPP_INFO_STREAM(get_logger(), "std::prev(batchFixedLagSmoother_->timestamps().end())->first: " << std::prev(batchFixedLagSmoother_->timestamps().end())->first);
        }
    }else if(config_.fixed_lag_smoother_batch_or_incremental == "incremental"){
        if(!isam2FixedLagSmoother_->timestamps().empty()){
            keyframeTimestamp = std::prev(isam2FixedLagSmoother_->timestamps().end())->second;
            RCLCPP_INFO_STREAM(get_logger(), "isam2FixedLagSmoother_->timestamps().begin()->first: " << isam2FixedLagSmoother_->timestamps().begin()->first);
            RCLCPP_INFO_STREAM(get_logger(), "std::prev(isam2FixedLagSmoother_->timestamps().end())->first: " << std::prev(isam2FixedLagSmoother_->timestamps().end())->first);
        }
    }

    double timeImuBeg = 0.0;
    double timeImuEnd = 0.0;
    if (!imuPoseTimeline_.empty()){
        timeImuBeg = imuPoseTimeline_.begin()->first;
        timeImuEnd = std::prev(imuPoseTimeline_.end())->first;
    }

    // Eigen::MatrixXd imuPreintCovariance = Eigen::MatrixXd::Identity(9, 9);
    // Eigen::MatrixXd imuBiasCovariance = Eigen::MatrixXd::Identity(6, 6);
    // if(imuIntegrator_->deltaTij() > TIME_EPS){
    //     imuPreintCovariance = imuIntegrator_->preintMeasCov();
    //     imuBiasCovariance = *gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegrator_->deltaTij()) * noiseModelBetweenBias_);
    // }

    size_t numKeyframes = 0;
    // numKeyframes = batchFixedLagSmoother_->timestamps().size();
    // numKeyframes = isam2FixedLagSmoother_->timestamps().size();

    if(config_.fixed_lag_smoother_batch_or_incremental == "batch"){
        numKeyframes = batchFixedLagSmoother_->timestamps().size();
    }else if(config_.fixed_lag_smoother_batch_or_incremental == "incremental"){
        numKeyframes = isam2FixedLagSmoother_->timestamps().size();
    }

    fgStateLogger_.recordFGState(
        keyframeTimestamp,                              // const double& keyframeTimestamp,
        timeCurrScanBeg_,                               // const double& timeCurrScanBeg,
        timeCurrScanEnd_,                               // const double& timeCurrScanEnd,
        timeImuBeg,                                     // const double& timeImuBeg,
        timeImuEnd,                                     // const double& timeImuEnd,
        key_,                                           // const int& keyFrameId,
        T_bPrevKf_bCurrKf_lo_,                          // const gtsam::Pose3& lidarBetweenFactor,
        T_w_bCurrKf_lo_,                                     // const gtsam::Pose3& lidarPriorFactor,
        lidarOdomCovariance_log_,                                // const Eigen::MatrixXd& lidarOdomCovariance,
        T_bPrevKf_bCurrKf_imu_log_,                          // const gtsam::Pose3& imuBetweenFactor,
        imuPreintCovariance_log_,                            // const Eigen::MatrixXd& imuPreintCovariance,
        imuBiasCovariance_log_,                              // const Eigen::MatrixXd& imuBiasCovariance,
        statePrevKf_,                                   // const gtsam::NavState& statePrevKf,
        biasPrevKf_,                                    // const gtsam::imuBias::ConstantBias& biasPrevKf,
        stateCurrKf_initGuess_lo_log_,                           // const gtsam::NavState& stateCurrKf_initGuess_lo,
        biasCurrKf_initGuess_lo_log_,                          // const gtsam::imuBias::ConstantBias& biasCurrKf_initGuess_lo,
        stateCurrKf_initGuess_imu_log_,                           // const gtsam::NavState& stateCurrKf_initGuess_imu,
        biasCurrKf_initGuess_imu_log_,                          // const gtsam::imuBias::ConstantBias& biasCurrKf_initGuess_imu,
        stateCovPredCurrKf_imu_,
        stateCurrKf_,                                   // const gtsam::NavState& optimizedStateCurrKf,
        biasCurrKf_,                                    // const gtsam::imuBias::ConstantBias& optimizedBiasCurrKf,
        stateCovCurrKf_,
        // batchFixedLagSmoother_->timestamps().size(),         // const int& numKeyframes,
        numKeyframes,
        isFGInitialized_                                // const bool& isFactorGraphOn,
    );
}


void Frontend::computeWeightsFromResiduals(Eigen::Ref<Eigen::VectorXd> r, Eigen::VectorXd& w, double s, std::string kernel, double c, bool mad_scale_estimation)
{
    // compute a scale correction by using median absolute deviation(MAD):
    //  https://en.wikipedia.org/wiki/Median_absolute_deviation
    if (mad_scale_estimation)
    {
        timeLogger_.start("[SQP]: Hessian Computation - Residual Median Calculation", __FUNCTION__, __LINE__);
        double r_med = getMedianInPlace(r);      // X_bar = median(X)
        Eigen::VectorXd absdev = (r.array() - r_med).abs();
        double absdev_med = getMedianInPlace(absdev);    // MAD = median(|Xi - X_bar|)
        s = 1.4826 * std::max(absdev_med, 1e-12);   // 1.4826 makes MAD ~ σ for Gaussian. Also 1e-12 caring numerical stability
        timeLogger_.stop("[SQP]: Hessian Computation - Residual Median Calculation", __FUNCTION__, __LINE__);
    }

    // compute robust kernel weights for each residuals
    //  ref: http://ceres-solver.org/nnls_modeling.html#instances
    // We need to refine these robust kernel weight computations.
    const double eps = 1e-12;
    w.resize(r.size());  // r should be resized with the decleration
    for (int i = 0; i < r.size(); ++i) 
    {
        double ri = r(i);
        double u  = ri / std::max(s, eps);
        double wi = 1.0;
        if (kernel == "Huber")
        {
            c = 1.345; // gives 95% efficiency for Gaussian noise
            double a = std::abs(u);
            wi = (a <= c) ? 1.0 : (c / (a + eps));
        }
        else if (kernel == "Cauchy")
        {
            c = 3.0; // typically 2.0 - 4.0
            double t = (u*u) / (c*c);
            wi = 1.0 / (1.0 + t);
        }
        else if (kernel == "Tukey")
        {
            c = 4.5; // typically 4.5
            double a = std::abs(u);
            if (a <= c) 
            {
                double z = 1.0 - (u*u)/(c*c);
                wi = z*z;
            } 
            else 
            {
                wi = 0.0;
            }
        }
        w(i) = wi;
    }
}


// void Frontend::solveLeastSquares_InequalityConstraints_ActiveSet()
// {
//     // [Rule of Thumb for Rotation Optimization]
//     //  - map SO(3)/SE(3) to so(3)/se(3) when do optimizations on rotations
//     //      -> So, we use x_qp_iter(so(3)/se(3)) only in QP iteration where optimization happens
//     //  - for the same reason, construct constraints with so(3)/se(3) (intuitively, linearize constraints(SE(3)->se(3)) when putting into optimization)
//     //  - do rotation composition on SO(3)/SE(3). DO NOT just compose so(3)/se(3) like vector additions
//     //      -> So, we compose a rotation in ICP iteration in SE(3)
//     //      -> As you can see T1*exp(x2) != exp(x1+x2) -> exp(x1+x2) != exp(x1)(x2) - see BCH formula in State Estimation for Robotics


//     if(!config_.use_voxel_map)
//     {
//         // set the local map point cloud to the kd_tree to nearest neighbor search
//         timeLogger_.start("kdTreeMapLocal_->setInputCloud", __FUNCTION__, __LINE__);
//         kdTreeMapLocal_->setInputCloud(cloudMapLocalDs_);
//         timeLogger_.stop("kdTreeMapLocal_->setInputCloud", __FUNCTION__, __LINE__);
//     }

//     if (debug_print_qp_active_set_init_guess)
//     {
//         RCLCPP_INFO_STREAM(get_logger(), "t_bPrevKf_bCurrKf_initGuess_: " << t_bPrevKf_bCurrKf_initGuess_);
//         RCLCPP_INFO_STREAM(get_logger(), "q_bPrevKf_bCurrKf_initGuess_: " << q_bPrevKf_bCurrKf_initGuess_);
//         RCLCPP_INFO_STREAM(get_logger(), "t_w_bCurrKf_: " << t_w_bCurrKf_.transpose());
//         RCLCPP_INFO_STREAM(get_logger(), "q_w_bCurrKf_: " << q_w_bCurrKf_);
//     }


//     // Icp pose variables used for point-plane correspondance in each ICP iteration 
//     Eigen::Vector3d t_w_bCurrKf_icp = t_w_bCurrKf_;
//     Eigen::Quaterniond q_w_bCurrKf_icp = q_w_bCurrKf_;
//     Sophus::SE3d T_w_bCurrKf_icp(q_w_bCurrKf_icp.toRotationMatrix(), t_w_bCurrKf_icp);
//     // should be fed into preparePointPlane() directly

//     // keep the initial pose right before entering ICP
//     Eigen::Vector3d t_icp_init = t_w_bCurrKf_;
//     Eigen::Quaterniond q_icp_init = q_w_bCurrKf_;
//     Sophus::SE3d T_icp_init(q_icp_init.toRotationMatrix(), t_icp_init);

//     // total transformation increment through ICP + QP
//     Eigen::Vector3d t_total_inc = t_bPrevKf_bCurrKf_initGuess_;
//     Eigen::Quaterniond q_total_inc = q_bPrevKf_bCurrKf_initGuess_;
//     Sophus::SE3d T_total_inc(q_total_inc.toRotationMatrix(), t_total_inc);

//     bool ineqConActivated = false;

//     // ICP iteration (point-to-plane)
//     for (int iter_cnt = 0; iter_cnt < config_.icp_iteration_num; iter_cnt++) {
//         if (debug_print_qp_icp_iter_num)
//             RCLCPP_INFO_STREAM(get_logger(), "icp_iter_num: " << iter_cnt);

//         // Recompute point-to-plane residuals based on q_w_bCurr_ and t_w_bCurr_
//         pointScanCurrInBForResidual_.clear();
//         planeNormalForResidual_.clear();
//         planeDistFromOriginForResidual_.clear();

//         timeLogger_.start("preparePointPlaneResidual", __FUNCTION__, __LINE__);
//         preparePointPlaneResidual();
//         timeLogger_.stop("preparePointPlaneResidual", __FUNCTION__, __LINE__);

//         // for debugging
//         if (debug_print_num_residuals) {
//             RCLCPP_INFO_STREAM(get_logger(), "num. of pts nn found: " << numPtsNnFound_);
//             RCLCPP_INFO_STREAM(get_logger(), "num. of residual points: " << numResidual_);
//         }

//         lm_lambda_ = config_.levenberg_marquardt_lambda_qp_active_set; // initialize L-M lambda parameter

//         // Non-linear optimization inner iteration (This is not technically SQP since this is just linearization+QP and also Lagrangian is not always involved)
//         for (int iter_qp = 0; iter_qp < config_.sqp_iteration_num_qp_active_set; iter_qp++){
//             if (debug_print_qp_sqp_iter_num)
//                 RCLCPP_INFO_STREAM(get_logger(), "sqp_iter_num: " << iter_qp);

//             // Compute the current pose for Jacobian computation in QP iteration
//             Sophus::SE3d T_w_bCurrKf_qp = T_icp_init * T_total_inc; // Just need the world pose info for point-plane correspondance
//             Eigen::Quaterniond q_w_bCurrKf_qp = Eigen::Quaterniond(T_w_bCurrKf_qp.rotationMatrix());
//             q_w_bCurrKf_qp.normalize();
//             Eigen::Vector3d t_w_bCurrKf_qp = T_w_bCurrKf_qp.translation();

//             Eigen::MatrixXd A(numResidual_, 6); // stacked Jacobian
//             Eigen::MatrixXd b(numResidual_, 1); // residual vector
//             Eigen::MatrixXd b_for_kernel(numResidual_, 1); // residual vector to be used for kernel weight computation

//             // ---- Jacobian Computation ----
//             timeLogger_.start("[SQP]: Jacobian Computation", __FUNCTION__, __LINE__);
//             for (int i = 0; i < numResidual_; ++i) {
//                 const Eigen::Vector3d bp = pointScanCurrInBForResidual_[i];
//                 // const Eigen::Vector3d wp = q_w_bCurrKf_ * bp + t_w_bCurrKf_; // should be updated at each iteration based on the current pose update
//                 const Eigen::Vector3d wp = q_w_bCurrKf_qp * bp + t_w_bCurrKf_qp; // should be updated at each iteration based on the current pose update
//                 const Eigen::Vector3d wn = planeNormalForResidual_[i];  // plane normal (does not change depending on the current body pose)
//                 const double d = planeDistFromOriginForResidual_[i];    // plane offset (does not change depending on the current body pose)
            
//                 if(config_.pose_increment_multiplication == "right"){
//                     // Right Jacobian
//                     // Eigen::Matrix3d Rwb = q_w_bCurrKf_.toRotationMatrix();
//                     Eigen::Matrix3d Rwb = q_w_bCurrKf_qp.toRotationMatrix();
//                     Eigen::Matrix3d bp_hat = getSkewSymMatrix(bp);
//                     // row i of A: [ -wn^T @ Rwb @ [bp]x  wn^T @ Rwb ]
//                     A.row(i).head<3>() = -wn.transpose() * Rwb * bp_hat;
//                     A.row(i).tail<3>() = wn.transpose() * Rwb;
//                     // residual b = - (wn^T @ wp + d)
//                     b(i) = - (wn.transpose() * wp + d);
//                     b_for_kernel(i) = b(i);             // for kernel weighting for robustifying the cost function, which is to be used later
//                     // following a conventional || A x - b' || notation
//                     // = || A x - (- (wn^T @ wp + d)) || 
//                     // = || A x + (wn^T @ wp + d) ||  
//                     // = || A x + b0 || where b0 = wn.transpose() * wp(0) + d
//                 } else if (config_.pose_increment_multiplication == "left"){
//                     // Left Jacobian
//                     // row i of A: [ (p × n)^T  n^T ]
//                     A.row(i).head<3>() = wp.cross(wn).transpose();
//                     A.row(i).tail<3>() = wn.transpose();
//                     // residual b = - (wn^T @ wp + d)
//                     b(i) = - (wn.transpose() * wp + d);
//                     b_for_kernel(i) = b(i);
//                 } else {
//                     RCLCPP_INFO_STREAM(get_logger(), "Left or Right Jacobian is not defined. Jacobian is set to zero.");
//                 }

//                 // ----- Sanity check for Jacobian (analytical Jacobian vs finite difference Jacobian) ----- 
//                 // // auto residual = [&](const Eigen::Vector3d& w, const Eigen::Vector3d& t){
//                 // //   return wn.dot( (Eigen::AngleAxisd(w.norm(), (w.norm()>1e-12)? w.normalized():Eigen::Vector3d::UnitX()).toRotationMatrix() * bp) + t ) + d;
//                 // // };
//                 // Eigen::Vector3d w0 = Eigen::Vector3d::Zero();
//                 // Eigen::Vector3d t0 = Eigen::Vector3d::Zero();

//                 // // Your analytical J at (w0,t0): A_i = [ -n^T R [b]x , n^T R ] with R from current pose
//                 // Eigen::RowVector<double,6> Ai;
//                 // Ai.head<3>() = -wn.transpose() * Rwb * bp_hat;
//                 // Ai.tail<3>() =  wn.transpose() * Rwb;

//                 // // Finite-diff directional check
//                 // Eigen::Matrix<double,6,1> v; v.setRandom(); v.normalize();
//                 // double eps = 1e-8;
//                 // Eigen::Vector3d dw = v.head<3>() * eps;
//                 // Eigen::Vector3d dt = v.tail<3>() * eps;

//                 // double r0 = wn.dot(Rwb*bp + t_w_bCurrKf_) + d;
//                 // Eigen::Matrix3d R1 = Rwb * Eigen::AngleAxisd(dw.norm(), (dw.norm()>1e-12)? dw.normalized():Eigen::Vector3d::UnitX()).toRotationMatrix();
//                 // double r1 = wn.dot(R1*bp + (t_w_bCurrKf_ + Rwb*dt)) + d;

//                 // double fd = (r1 - r0) / eps;
//                 // double an = Ai * v;
//                 // // std::cout << "dir-deriv num=" << fd << "  analytic=" << an << "  diff=" << std::abs(fd-an) << "\n";
//                 // RCLCPP_INFO_STREAM(get_logger(), "dir-deriv num=" << fd << "  analytic=" << an << "  diff=" << std::abs(fd-an));
//                 // ----- Sanity check for Jacobian (analytical Jacobian vs finite difference Jacobian) ----- 
//             }
//             timeLogger_.stop("[SQP]: Jacobian Computation", __FUNCTION__, __LINE__);

//             // ---- Jacobian Column Scaling ----
//             timeLogger_.start("[SQP]: Jacobian Column Scaling", __FUNCTION__, __LINE__);
//             Eigen::DiagonalMatrix<double, Eigen::Dynamic> D; // Jacobian Scaling Matrix
//             Eigen::VectorXd dj;  // scaling factor for each column
//             if (config_.turn_on_jacobian_column_scaling_qp_active_set){
//                 // 1) Build column-scaling matrix D so that columns of A_s = A * D have norm ~1 ---
//                 const int n = A.cols();
//                 Eigen::VectorXd col_norms(n);
//                 for (int j = 0; j < n; ++j){
//                     col_norms(j) = A.col(j).norm();
//                 }
//                 const double eps   = 1e-12;    // avoid division by zero
//                 const double dmin  = 1e-5;     // clamp factors to avoid extreme scaling
//                 const double dmax  = 1e+5;
//                 dj = (col_norms.array() > eps).select(1.0 / col_norms.array(), 1.0); // scaling factor for each column, eps: caring for numerical stability
//                 dj = dj.cwiseMax(dmin).cwiseMin(dmax);
//                 D = dj.asDiagonal(); // construct a column-scaling matrix D, where diagonal elements are dj

//                 // 2) Scaled Jacobian and transform bounds/constraints ---
//                 Eigen::MatrixXd A_s = A * D;           // columns -> unit-norm

//                 // 3) Build QP in z: min 1/2 z^T H_s z + h_s^T z ---
//                 //  done in the following scripts

//                 // 4) Summary
//                 // Now that se have ||Ax - b||2 = ||(AD)(D^-1x) - b||2 = ||(A_s)z - b||2 
//                 //  (A_s) =    AD   : column-wise scaled Jacobian
//                 //    z   = (D^-1x) : scaled state vector (to be mapped back to x after solving QP)

//                 // update A to As (column-wise scaled A)
//                 A = A_s; 
//             }
//             timeLogger_.stop("[SQP]: Jacobian Column Scaling", __FUNCTION__, __LINE__);

//             timeLogger_.start("[SQP]: Hessian Computation", __FUNCTION__, __LINE__);
//             Eigen::Matrix<double,6,6> HQP = Eigen::Matrix<double,6,6>::Zero();  // Hessian in QP
//             Eigen::Matrix<double,6,1> hQP = Eigen::Matrix<double,6,1>::Zero();  // h matrix in QP

//             // ---- Weights Computation: Robust Kernel for Outlier Handling ----
//             Eigen::VectorXd w;  
//             if(config_.turn_on_robust_kernel_qp_active_set){
//                 timeLogger_.start("[SQP]: Hessian Computation - Kernel Weights", __FUNCTION__, __LINE__);
//                 double s = 0.0; // default scaling just as an option manually tune it
//                 double c = 0.0; // default range just as an option manually tune it
//                 timeLogger_.start("[SQP]: computeWeightsFromResiduals", __FUNCTION__, __LINE__);
//                 computeWeightsFromResiduals(b_for_kernel, w, s, config_.robust_kernel_qp_active_set, c, true);
//                 timeLogger_.stop("[SQP]: computeWeightsFromResiduals", __FUNCTION__, __LINE__);
//             } else {
//                 w = Eigen::VectorXd::Ones(b_for_kernel.size());
//             }

//             if (debug_print_qp_active_set_matrices){
//                 RCLCPP_INFO_STREAM(get_logger(), "w: ");
//                 RCLCPP_INFO_STREAM(get_logger(), w.transpose());
//             }

//             timeLogger_.start("[SQP]: W = w.asDiagonal()", __FUNCTION__, __LINE__);
//             // Eigen::MatrixXd W = w.asDiagonal(); // This is SUPER expensive, keep this line for a lesson just in case
//             Eigen::DiagonalMatrix<double, Eigen::Dynamic> W = w.asDiagonal();   // faster
//             timeLogger_.stop("[SQP]: W = w.asDiagonal()", __FUNCTION__, __LINE__);

//             timeLogger_.stop("[SQP]: Hessian Computation - Kernel Weights", __FUNCTION__, __LINE__);

//             if(config_.qp_active_set_Hessian_computation == "matrix_multiplication"){
//                 // cleaner but slow way
//                 timeLogger_.start("[SQP]: Hessian Computation - Hessian (Mat. Multiplication)", __FUNCTION__, __LINE__);
//                 HQP = 2 * A.transpose() * W * A;
//                 hQP = - 2 * A.transpose() * W * b;
//                 timeLogger_.stop("[SQP]: Hessian Computation - Hessian (Mat. Multiplication)", __FUNCTION__, __LINE__);
//             } else if(config_.qp_active_set_Hessian_computation == "for_loop"){
//                 // fast way
//                 timeLogger_.start("[SQP]: Hessian Computation - Hessian (For-loop)", __FUNCTION__, __LINE__);
//                 for (int i = 0; i < numResidual_; ++i) {
//                     double wi = w(i);
//                     Eigen::Matrix<double,1,6> ai = A.row(i);
//                     HQP.noalias() += 2.0 * wi * (ai.transpose() * ai);
//                     hQP.noalias() += - 2.0 * wi * ai.transpose() * b(i);
//                 }
//                 timeLogger_.stop("[SQP]: Hessian Computation - Hessian (For-loop)", __FUNCTION__, __LINE__);
//             } else {
//                 RCLCPP_INFO_STREAM(get_logger(), "Hessian Computation is not defined well. Set Hessian to zero.");
//             }

//             // ---- Levenberg-Marquardt Method: Adding a Damping Term Here ----
//             if(config_.turn_on_levenberg_marquardt_qp_active_set){
//                 timeLogger_.start("[SQP]: Hessian Computation - LM Hessian", __FUNCTION__, __LINE__);
//                 if(config_.turn_on_levenberg_marquardt_qp_active_set_marquardt_damping){
//                     HQP += 2.0 * lm_lambda_ * (HQP.diagonal().asDiagonal()); // // Marquardt-type damping, "2.0 *" is for conversion to QP formulation (* 1/2 afterward)
//                 } else {
//                     HQP += 2.0 * lm_lambda_ * Eigen::MatrixXd::Identity(6,6); // "2.0 *" is for conversion to QP formulation (* 1/2 afterward)
//                 }
//                 timeLogger_.stop("[SQP]: Hessian Computation - LM Hessian", __FUNCTION__, __LINE__);
//             }
//             timeLogger_.stop("[SQP]: Hessian Computation", __FUNCTION__, __LINE__);

//             // solution container
//             Eigen::VectorXd x_qp_iter(6); // solution container (roll, pitch, yaw, tx, ty, tz)
//             x_qp_iter.setZero();

//             // ---- Set Inequality Constraints: Box Constraint ----
//             Eigen::VectorXd lb(6); // scaler constraint container (roll, pitch, yaw, tx, ty, tz)
//             Eigen::VectorXd ub(6);
//             if (t_bPrev2Kf_bPrevKf_ != Eigen::Vector3d::Zero() && config_.turn_on_ineq_constraints && config_.ineq_constraints_type == "box"){
//                 timeLogger_.start("[SQP]: Inequality Setting", __FUNCTION__, __LINE__);
//                 // linear velocity from the 2nd previous frame to the previous frame, expressed in the body at the 2nd previous frame
//                 const double linVel = t_bPrev2Kf_bPrevKf_.x() / (timePrevScanEnd_ - timePrev2ScanEnd_);

//                 const double maxStrAng = config_.vehicle_kinematic_constraints_max_steering_angle * (M_PI/180);   // [rad]
//                 const double whlbase = config_.vehicle_kinematic_constraints_wheel_base; // [m]
//                 double maxYawRate = linVel * std::tan(maxStrAng) / whlbase;
//                 if (linVel < 0) maxYawRate *= -1;
//                 double minYawRate = - maxYawRate;

//                 RCLCPP_INFO_STREAM(get_logger(), "forward linear velocity: " << linVel << " [m/s]");
//                 RCLCPP_INFO_STREAM(get_logger(), "max yaw rate: " << maxYawRate * (180/M_PI) << " [deg/s]");
//                 RCLCPP_INFO_STREAM(get_logger(), "min yaw rate: " << minYawRate * (180/M_PI) << " [deg/s]");

//                 // [Note] We have to impose constraints on AXIS-ANGLE VECTOR (alpha, beta, gamma) not really (roll, pitch, yaw)
//                 // But we can approximate delta_yaw in body frame with the z element of the axis-angle vector
//                 // A concrete way is axis-angle vector -> rotation matrix -> Euler angle (yaw)
//                 //  and we can linearize the constraint by using Jacobian of Euler angle (yaw) wrt axis-angle vector
//                 //  axis-angle vector always starts from identity in body frame(the previous body pose)
//                 //  in that case Jacobian of Euler angle (yaw) wrt axis-angle vector is identity
//                 //   since the right Jacobian evaluated at the identity (Jr(0)) is identity.
//                 //  We can use this approximation because typically it would be a small rotation between scans.
//                 //  Thus, we can typically approximate delta_yaw in body frame with the z element of the axis-angle vector
//                 //  One potential failure is that if the body z-axis does not align with the true yaw axis, the approximation would not hold
//                 //  In that case, we need to perform and update a ground plane normal estimation and update body z-axis according to it.

//                 for (int i=0; i<6; ++i){   // initialize all the bounds as inf(no constraint essentially)
//                     lb(i) = - std::numeric_limits<double>::max();
//                     ub(i) = std::numeric_limits<double>::max();
//                 }
//                 // compute yaw rate bounds (actually wrong and it should be an affine constraint)
//                 lb(2) = minYawRate * (timePrevScanEnd_ - timePrev2ScanEnd_);
//                 ub(2) = maxYawRate * (timePrevScanEnd_ - timePrev2ScanEnd_);

//                 RCLCPP_INFO_STREAM(get_logger(), "timePrevScanEnd_: " << timePrevScanEnd_);
//                 RCLCPP_INFO_STREAM(get_logger(), "timePrev2ScanEnd_: " << timePrev2ScanEnd_);
//                 RCLCPP_INFO_STREAM(get_logger(), "timePrevScanEnd_ - timePrev2ScanEnd_: " << timePrevScanEnd_ - timePrev2ScanEnd_);
//                 RCLCPP_INFO_STREAM(get_logger(), "lb(2): " << lb(2));
//                 RCLCPP_INFO_STREAM(get_logger(), "ub(2): " << ub(2));

//                 // for cases we use imu covariance for box constraint
//                 if(config_.qp_compute_ineq_from_imu_cov && !stateCovPredCurrKf_imu_.isApprox(Eigen::MatrixXd::Zero(9, 9))){
//                     const gtsam::Vector9 var = stateCovPredCurrKf_imu_.diagonal();
//                     const gtsam::Vector9 std = var.array().sqrt().matrix();

//                     gtsam::Vector6 mean = gtsam::Pose3::Logmap(T_bPrevKf_bCurrKf_imu_log_);  // [Rx,Ry,Rz,Tx,Ty,Tz]
//                     lb(0) = mean(0) - std(0) * config_.qp_compute_ineq_from_imu_cov_sigma_coef;
//                     lb(1) = mean(1) - std(1) * config_.qp_compute_ineq_from_imu_cov_sigma_coef;
//                     lb(2) = mean(2) - std(2) * config_.qp_compute_ineq_from_imu_cov_sigma_coef;
//                     lb(3) = mean(3) - std(6) * config_.qp_compute_ineq_from_imu_cov_sigma_coef; // std(4) - std(6) would be velocity -> see gtsam::InvariantEKF
//                     lb(4) = mean(4) - std(7) * config_.qp_compute_ineq_from_imu_cov_sigma_coef;
//                     lb(5) = mean(5) - std(8) * config_.qp_compute_ineq_from_imu_cov_sigma_coef;
//                     ub(0) = mean(0) + std(0) * config_.qp_compute_ineq_from_imu_cov_sigma_coef;
//                     ub(1) = mean(1) + std(1) * config_.qp_compute_ineq_from_imu_cov_sigma_coef;
//                     ub(2) = mean(2) + std(2) * config_.qp_compute_ineq_from_imu_cov_sigma_coef;
//                     ub(3) = mean(3) + std(6) * config_.qp_compute_ineq_from_imu_cov_sigma_coef;
//                     ub(4) = mean(4) + std(7) * config_.qp_compute_ineq_from_imu_cov_sigma_coef;
//                     ub(5) = mean(5) + std(8) * config_.qp_compute_ineq_from_imu_cov_sigma_coef;
//                 }

//                 // if using column-wise Jacobian scaling for numerical stability, we also have to scale the constraint
//                 if(config_.turn_on_jacobian_column_scaling_qp_active_set) {
//                     Eigen::VectorXd d_inv = dj.cwiseInverse();
//                     lb = lb.cwiseProduct(d_inv);
//                     ub = ub.cwiseProduct(d_inv);
//                 }

//                 timeLogger_.stop("[SQP]: Inequality Setting", __FUNCTION__, __LINE__);
//             } else {
//                 // no constraints
//                 lb = Eigen::VectorXd(); 
//                 ub = Eigen::VectorXd();
//             }

//             // ---- Set Inequality Constraints: Affine Constraint ----
//             Eigen::MatrixXd constraintMatrix = Eigen::MatrixXd();
//             Eigen::VectorXd Alb = Eigen::VectorXd();
//             Eigen::VectorXd Aub = Eigen::VectorXd();
//             if(config_.turn_on_ineq_constraints && config_.ineq_constraints_type == "curvature"){
//                 const double maxStrAng = config_.vehicle_kinematic_constraints_max_steering_angle * (M_PI/180);   // [rad]
//                 const double whlbase = config_.vehicle_kinematic_constraints_wheel_base; // [m]
//                 const double kappaMax = std::tan(maxStrAng) / whlbase;

//                 // remember x = [rx, ry, rz, tx, ty, tz]T
//                 // A x <= 0
//                 // [ 0 0 -l tan(-phi_max) 0 0]  
//                 // [ 0 0  l -tan(phi_max) 0 0] x <= 0
//                 // [ 0 0  0            -1 0 0]
//                 // equivalently (for simplicity):
//                 // [ 0 0 -1 -kappa_max 0 0]  
//                 // [ 0 0  1 -kappa_max 0 0] x <= 0
//                 // [ 0 0  0         -1 0 0]

//                 // This means:
//                 // constraintMatrix * x <= 0

//                 // Actually, it should be:
//                 // constraintMatrix * (x_qp_iter + x_total_inc) <= 0
//                 // , where x_qp_iter: the latest delta_x estimation in this QP inner loop,  x_total_inc: total incremental delta_x estimation in this consecutive lidar frames so far (ICP + QP)
//                 // So it should be:
//                 // constraintMatrix * x_qp_iter <= -constraintMatrix * x_total_inc
//                 // constraintMatrix * x_qp_iter <= Aub, where Aub = -constraintMatrix * x_total_inc
//                 // Now that x is actually scaled:
//                 //   constraintMatrix * D * (D^-1 x) <= Aub
//                 //   constraintMatrix * D * z <= Aub, we are actually solving this z (=D^-1 x) (to be mapped back afterward)
//                 //   constraintMatrix' * z <= Aub, where constraintMatrix' = constraintMatrix * D
//                 // Thus, we have to scale constraintMatrix but we don't have to do that with Aub


//                 constraintMatrix = Eigen::MatrixXd::Zero(3,6);
//                 constraintMatrix(0,2) = -1.0;
//                 constraintMatrix(0,3) = -kappaMax;
//                 constraintMatrix(1,2) = 1.0;
//                 constraintMatrix(1,3) = -kappaMax;
//                 constraintMatrix(2,3) = -1.0;

//                 Alb = Eigen::VectorXd::Constant(3, -std::numeric_limits<double>::max());

//                 Eigen::Vector<double, 6> a_total_inc = T_total_inc.log();
//                 Eigen::Vector<double, 6> x_total_inc;
//                 x_total_inc.head<3>() = a_total_inc.tail<3>();
//                 x_total_inc.tail<3>() = a_total_inc.head<3>();
//                 Aub = -constraintMatrix * x_total_inc;

//                 // we need to scale the constraintMatrix
//                 if(config_.turn_on_jacobian_column_scaling_qp_active_set){
//                     constraintMatrix = constraintMatrix * D;
//                 }
//             }

//             if (debug_print_qp_active_set_matrices_jacobian_residual){
//                 RCLCPP_INFO_STREAM(get_logger(), "A: " << A);
//                 RCLCPP_INFO_STREAM(get_logger(), "b: " << b);
//             }

//             if (debug_print_qp_active_set_matrices){
//                 RCLCPP_INFO_STREAM(get_logger(), "HQP: ");
//                 RCLCPP_INFO_STREAM(get_logger(), HQP);
//                 RCLCPP_INFO_STREAM(get_logger(), "hQP: ");
//                 RCLCPP_INFO_STREAM(get_logger(), hQP);
//                 RCLCPP_INFO_STREAM(get_logger(), "lb: " << lb);
//                 RCLCPP_INFO_STREAM(get_logger(), "ub: " << ub);
//                 RCLCPP_INFO_STREAM(get_logger(), "constraintMatrix: " << constraintMatrix);
//                 RCLCPP_INFO_STREAM(get_logger(), "Alb: " << Alb);
//                 RCLCPP_INFO_STREAM(get_logger(), "Aub: " << Aub);
//             }

//             // ---- Solve Inequality Constrained QP ----
//             timeLogger_.start("[SQP]: Solve QP", __FUNCTION__, __LINE__);
//             qpmad::Solver qpmadSolver;   // Solver = SolverTemplate<double, Eigen::Dynamic, 1, Eigen::Dynamic>; in qpmad/solver.h
//             qpmad::SolverParameters param;
//             param.hessian_type_ = qpmad::SolverParameters::HESSIAN_LOWER_TRIANGULAR;
//             qpmad::Solver::ReturnStatus status =
//                 qpmadSolver.solve(x_qp_iter, HQP, hQP, lb, ub, constraintMatrix, Alb, Aub, param);
//             // [out] Vector primal – solution vector, mandatory, allocated if needed
//             // [in,out] Matrix H – Hessian, mandatory, non-empty, factorized in-place
//             // [in] Vector h – objective vector, mandatory, may be empty
//             // [in] Vector lb – vector of lower bounds, may be omitted or may be empty consistently with ub
//             // [in] Vector ub – vector of upper bounds, may be omitted or may be empty consistently with lb
//             // [in] Matrix A – general constraints matrix, may be omitted with all general constraints, may be empty
//             // [in] Vector Alb – lower bounds of general constraints, may be omitted with all general constraints, may be empty
//             // [in] Vector Aub – upper bounds of general constraints, may be omitted (if A and Alb are present they are assumed to be equality constraints), may be empty
//             // [in] SolverParameters param – solver parameters, may be omitted

//             // sanity check with Cholesky Decomposition linear solver
//             if (debug_try_qp_active_set_Cholesky_Decomposition_unconstrained){
//                 auto H_test = HQP / 2;
//                 auto h_test = - hQP / 2;
//                 x_qp_iter = H_test.ldlt().solve(-h_test);
//             }

//             // check optimization status
//             if (status != qpmad::Solver::OK){
//                 RCLCPP_INFO_STREAM(get_logger(), "Error in solving inequality constrained optimization problem with QPmad library.");
//             }

//             // extract the optimization status
//             Eigen::VectorXd dual;
//             Eigen::Matrix<qpmad::MatrixIndex, Eigen::Dynamic, 1> indices;
//             Eigen::Matrix<bool, Eigen::Dynamic, 1> is_lower;
//             qpmadSolver.getInequalityDual(dual, indices, is_lower);

//             if (debug_print_qp_active_set_solution_analysis){
//                 dualLogger_.recordDual(timeCurrScanBeg_, dual, indices, is_lower);
//                 if (dual.size() > 0){
//                     ineqConActivated = true;
//                     timeLogger_.start("[SQP]: Inequality Constraints activated", __FUNCTION__, __LINE__);
//                     RCLCPP_INFO_STREAM(get_logger(), "Number of active Inquality Constraints: " << dual.size());
//                     RCLCPP_INFO_STREAM(get_logger(), "Do Constraints satisfied? 0 == satisfied: ");
//                     timeLogger_.stop("[SQP]: Inequality Constraints activated", __FUNCTION__, __LINE__);

//                     if (constraintMatrix.size() == 0) {
//                         RCLCPP_INFO_STREAM(get_logger(), "No general constraints (A is empty).");
//                     } else if (constraintMatrix.cols() != x_qp_iter.size()) {
//                         RCLCPP_WARN_STREAM(get_logger(), "Constraint matrix cols != x size.");
//                     } else {
//                         RCLCPP_INFO_STREAM(get_logger(), "A*x = " << (constraintMatrix * x_qp_iter).transpose());
//                     }
//                 }
//             }

//             // scale the solution back if necessary
//             Eigen::VectorXd z = Eigen::VectorXd::Zero(6);
//             if (config_.turn_on_jacobian_column_scaling_qp_active_set) {
//                 z = x_qp_iter;
//                 //    z   = (D^-1 x) : scaled state vector (to be mapped back to x after solving QP)
//                 // --- 5) Map back to original variables (unscale) ---
//                 x_qp_iter = D * z;  // x = Dz
//             }
//             timeLogger_.stop("[SQP]: Solve QP", __FUNCTION__, __LINE__);

//             // ---- Step Size Check: L-M Lambda Update ----
//             if(config_.levenberg_marquardt_trust_region_lambda_adjustment){
//                 if(!config_.turn_on_jacobian_column_scaling_qp_active_set) z = x_qp_iter;
//                 // Here we examine the step size, and adjust radius_(=1/lamdba_, following Ceres implementation)
//                 // 1. solve the normal equation
//                 // 2. compute the function decrease and step size quality
//                 // 3. if step size quality is good -> widen the radius (smaller lambda) -> and go to the following step
//                 //    if step size quality is bad  -> reduce the radius (larger lambda) 
//                 const auto ATWA = 0.5 * HQP;
//                 const auto ATWb = 0.5 * hQP;
//                 const Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6);
//                 const double f = computeResidualWithDeltaX_OriginalObjective(x0, w);
//                 const double f_delta_x = computeResidualWithDeltaX_OriginalObjective(x_qp_iter, w); // f should be based on the solution scaled back (x_qp_iter)
//                 const double m = computeModelPredictedReduction(x0, ATWA, ATWb);
//                 const double m_delta_x = computeModelPredictedReduction(z, ATWA, ATWb); // m (quadratic model) should be based on the solution scaled (z) because ATWA, ATWB are scaled
//                 const double rho = (f - f_delta_x) / (m - m_delta_x);

//                 // update lambda in LM trust region depending on the step quality
//                 if(rho < 0.25){     // if step size quality is bad  -> reduce the radius (larger lambda) 
//                     lm_lambda_ = 2 * lm_lambda_;
//                 }else if(rho > 0.75){   // if step size quality is good -> widen the radius (smaller lambda)
//                     lm_lambda_ = std::max(lm_lambda_ / 1.414, config_.levenberg_marquardt_trust_region_lambda_min);
//                 }

//                 if (debug_print_qp_levenberg_marquardt_adaptive_lambda){
//                     RCLCPP_INFO_STREAM(get_logger(), "[LM iteration]          lm_lambda : " << lm_lambda_);
//                     RCLCPP_INFO_STREAM(get_logger(), "[LM iteration]            delta_x : " << z.transpose());
//                     RCLCPP_INFO_STREAM(get_logger(), "[LM iteration] delta_x_scaledback : " << x_qp_iter.transpose());
//                     RCLCPP_INFO_STREAM(get_logger(), "[LM iteration]                  f : " << f);
//                     RCLCPP_INFO_STREAM(get_logger(), "[LM iteration]          f_delta_x : " << f_delta_x);
//                     RCLCPP_INFO_STREAM(get_logger(), "[LM iteration]                  m : " << m);
//                     RCLCPP_INFO_STREAM(get_logger(), "[LM iteration]          m_delta_x : " << m_delta_x);
//                     RCLCPP_INFO_STREAM(get_logger(), "[LM iteration]       step quality : " << rho);
//                 }

//                 // if the step is not good, reject it and go to the next iteration
//                 if(rho < config_.levenberg_marquardt_trust_region_step_quality_threshold){
//                     RCLCPP_INFO_STREAM(get_logger(), "[LM iteration] step quality: " << rho << ", current step is rejected.");
//                     continue;
//                 }
//             }

//             if (debug_print_qp_active_set_solution){
//                 for (unsigned int i=0; i<6; ++i){
//                     RCLCPP_INFO_STREAM(get_logger(), "x_qp_iter(" << i << "): " << x_qp_iter(i));
//                 }
//             }

//             // ---- Pose Increment Update in QP + ICP so far ----
//             Eigen::Vector<double, 6> a_qp_iter;
//             a_qp_iter.tail<3>() = x_qp_iter.head<3>(); // axis-angle vector (alpha, beta, gamma)
//             a_qp_iter.head<3>() = x_qp_iter.tail<3>(); // translation vector (x,y,z)
//             const Sophus::SE3d exp_x_qp_iter = Sophus::SE3d::exp(a_qp_iter);

//             T_total_inc = T_total_inc * exp_x_qp_iter;

//             if (debug_print_qp_active_set_solution_pose_update){
//                 // RCLCPP_INFO_STREAM(get_logger(), "T_total_inc: " << T_total_inc);
//             }

//         } // End: iterative QP inner iteration

//         // ---- Sensor Pose in world frame Update in ICP for new point-plane correspondance computation ----
//         T_w_bCurrKf_icp = T_icp_init * T_total_inc;
//         if (config_.pose_increment_multiplication == "right"){
//             T_w_bCurrKf_icp = T_icp_init * T_total_inc; // update by right multiplication
//         } else if (config_.pose_increment_multiplication == "left"){
//             T_w_bCurrKf_icp = T_total_inc * T_icp_init; // update by left multiplication
//         } else {
//             RCLCPP_INFO_STREAM(get_logger(), "Left or Right Pose Update is not defined. Current Pose is not updated.");
//         }

//         // The pose update should be outside the icp-qp loop (to be corrected)
//         q_w_bCurrKf_ = Eigen::Quaterniond(T_w_bCurrKf_icp.rotationMatrix()).normalized();
//         t_w_bCurrKf_ = T_w_bCurrKf_icp.translation();

//     } // End: icp iteration

//     // log time
//     if(ineqConActivated){
//         ineqConstLogger_.recordInequalityConstraints(timeCurrScanBeg_, true);
//     } else {
//         ineqConstLogger_.recordInequalityConstraints(timeCurrScanBeg_, false);
//     }

//     // end of the point-to-plane icp with inequality constraints
// }

void Frontend::solveLeastSquares_InequalityConstraints_ActiveSet()
{
    if(!config_.use_voxel_map)
    {
        // set the local map point cloud to the kd_tree to nearest neighbor search
        timeLogger_.start("kdTreeMapLocal_->setInputCloud", __FUNCTION__, __LINE__);
        kdTreeMapLocal_->setInputCloud(cloudMapLocalDs_);
        timeLogger_.stop("kdTreeMapLocal_->setInputCloud", __FUNCTION__, __LINE__);
    }

    // transform the current absolute pose based on initial guess (to be used for preparePointPlaneResidual())
    t_w_bCurrKf_ = q_w_bCurrKf_ * t_bPrevKf_bCurrKf_initGuess_ + t_w_bCurrKf_;
    q_w_bCurrKf_ = (q_w_bCurrKf_ * q_bPrevKf_bCurrKf_initGuess_).normalized();

    if (debug_print_qp_active_set_init_guess)
    {
        RCLCPP_INFO_STREAM(get_logger(), "t_bPrevKf_bCurrKf_initGuess_: " << t_bPrevKf_bCurrKf_initGuess_);
        RCLCPP_INFO_STREAM(get_logger(), "q_bPrevKf_bCurrKf_initGuess_: " << q_bPrevKf_bCurrKf_initGuess_);
        RCLCPP_INFO_STREAM(get_logger(), "t_w_bCurrKf_: " << t_w_bCurrKf_.transpose());
        RCLCPP_INFO_STREAM(get_logger(), "q_w_bCurrKf_: " << q_w_bCurrKf_);
    }

    Eigen::VectorXd x(6); // state vector (roll, pitch, yaw, tx, ty, tz)

    // ICP iteration (point-to-plane)
    for (int iter_cnt = 0; iter_cnt < config_.icp_iteration_num; iter_cnt++) 
    {
        if (debug_print_qp_icp_iter_num)
            RCLCPP_INFO_STREAM(get_logger(), "icp_iter_num: " << iter_cnt);

        timeLogger_.start("preparePointPlaneResidual", __FUNCTION__, __LINE__);
        preparePointPlaneResidual();
        timeLogger_.stop("preparePointPlaneResidual", __FUNCTION__, __LINE__);

        // if (debug_print_qp_active_set_point_plane_residual_pose)
        // {
        //     RCLCPP_INFO_STREAM(get_logger(), "t_w_bCurrKf_: " << t_w_bCurrKf_);
        //     RCLCPP_INFO_STREAM(get_logger(), "q_w_bCurrKf_: " << q_w_bCurrKf_);
        // }

        // for debugging
        if (debug_print_num_residuals) 
        {
            RCLCPP_INFO_STREAM(get_logger(), "num. of pts nn found: " << numPtsNnFound_);
            RCLCPP_INFO_STREAM(get_logger(), "num. of residual points: " << numResidual_);
        }

        // Non-linear optimization inner iteration (SQP)
        for (int iter_qp = 0; iter_qp < config_.sqp_iteration_num_qp_active_set; iter_qp++)
        {
            if (debug_print_qp_sqp_iter_num)
                RCLCPP_INFO_STREAM(get_logger(), "sqp_iter_num: " << iter_qp);

            Eigen::MatrixXd A(numResidual_, 6); // stacked Jacobian
            Eigen::MatrixXd b(numResidual_, 1); // residual vectorn
            Eigen::MatrixXd b_for_kernel(numResidual_, 1); // residual vector to be used for kernel weight computation

            timeLogger_.start("[SQP]: Jacobian Computation", __FUNCTION__, __LINE__);
            // loop over all the residuals and add them to the objective funtion, forming the entire objective function in the least squares problem
            for (int i = 0; i < numResidual_; ++i) 
            {
                const Eigen::Vector3d bp = pointScanCurrInBForResidual_[i];
                const Eigen::Vector3d wp = q_w_bCurrKf_ * bp + t_w_bCurrKf_; // should be updated at each iteration based on the current pose update
                const Eigen::Vector3d wn = planeNormalForResidual_[i];  // plane normal (does not change depending on the current body pose)
                const double d = planeDistFromOriginForResidual_[i];    // plane offset (does not change depending on the current body pose)

                // Jacobian Computation
                if(config_.pose_increment_multiplication == "right")
                {
                    // Right Jacobian
                    Eigen::Matrix3d Rwb = q_w_bCurrKf_.toRotationMatrix();
                    Eigen::Matrix3d bp_hat = getSkewSymMatrix(bp);
                    // row i of A: [ -wn^T @ Rwb @ [bp]x  wn^T @ Rwb ]
                    A.row(i).head<3>() = -wn.transpose() * Rwb * bp_hat;
                    A.row(i).tail<3>() = wn.transpose() * Rwb;
                    // residual b = - (wn^T @ wp + d)
                    b(i) = - (wn.transpose() * wp + d);
                    b_for_kernel(i) = b(i);             // for kernel weighting for robustifying the cost function, which is to be used later
                    // following a conventional || A x - b' || notation
                    // = || A x - (- (wn^T @ wp + d)) || 
                    // = || A x + (wn^T @ wp + d) ||  
                    // = || A x + b0 || where b0 = wn.transpose() * wp(0) + d
                }
                else if (config_.pose_increment_multiplication == "left")
                {
                    // Left Jacobian
                    // row i of A: [ (p × n)^T  n^T ]
                    A.row(i).head<3>() = wp.cross(wn).transpose();
                    A.row(i).tail<3>() = wn.transpose();
                    // residual b = - (wn^T @ wp + d)
                    b(i) = - (wn.transpose() * wp + d);
                    b_for_kernel(i) = b(i);
                }
                else
                {
                    RCLCPP_INFO_STREAM(get_logger(), "Left or Right Jacobian is not defined. Jacobian is set to zero.");
                }

                // ----- Sanity check for Jacobian (analytical Jacobian vs finite difference Jacobian) ----- 
                // // auto residual = [&](const Eigen::Vector3d& w, const Eigen::Vector3d& t){
                // //   return wn.dot( (Eigen::AngleAxisd(w.norm(), (w.norm()>1e-12)? w.normalized():Eigen::Vector3d::UnitX()).toRotationMatrix() * bp) + t ) + d;
                // // };
                // Eigen::Vector3d w0 = Eigen::Vector3d::Zero();
                // Eigen::Vector3d t0 = Eigen::Vector3d::Zero();

                // // Your analytical J at (w0,t0): A_i = [ -n^T R [b]x , n^T R ] with R from current pose
                // Eigen::RowVector<double,6> Ai;
                // Ai.head<3>() = -wn.transpose() * Rwb * bp_hat;
                // Ai.tail<3>() =  wn.transpose() * Rwb;

                // // Finite-diff directional check
                // Eigen::Matrix<double,6,1> v; v.setRandom(); v.normalize();
                // double eps = 1e-8;
                // Eigen::Vector3d dw = v.head<3>() * eps;
                // Eigen::Vector3d dt = v.tail<3>() * eps;

                // double r0 = wn.dot(Rwb*bp + t_w_bCurrKf_) + d;
                // Eigen::Matrix3d R1 = Rwb * Eigen::AngleAxisd(dw.norm(), (dw.norm()>1e-12)? dw.normalized():Eigen::Vector3d::UnitX()).toRotationMatrix();
                // double r1 = wn.dot(R1*bp + (t_w_bCurrKf_ + Rwb*dt)) + d;

                // double fd = (r1 - r0) / eps;
                // double an = Ai * v;
                // // std::cout << "dir-deriv num=" << fd << "  analytic=" << an << "  diff=" << std::abs(fd-an) << "\n";
                // RCLCPP_INFO_STREAM(get_logger(), "dir-deriv num=" << fd << "  analytic=" << an << "  diff=" << std::abs(fd-an));
                // ----- Sanity check for Jacobian (analytical Jacobian vs finite difference Jacobian) ----- 

            }
            timeLogger_.stop("[SQP]: Jacobian Computation", __FUNCTION__, __LINE__);

            timeLogger_.start("[SQP]: Jacobian Column Scaling", __FUNCTION__, __LINE__);
            Eigen::DiagonalMatrix<double, Eigen::Dynamic> D; // Jacobian Scaling Matrix
            Eigen::VectorXd dj;  // scaling factor for each column
            if (config_.turn_on_jacobian_column_scaling_qp_active_set)
            {
                // bounds/constraints scaled on the step x 
                // Eigen::VectorXd lb_x, ub_x;        // direct bounds on x
                // Eigen::MatrixXd Acon;              // general constraint matrix on x 
                // Eigen::VectorXd Alb_x, Aub_x;

                // 1) Build column-scaling matrix D so that columns of A_s = A * D have norm ~1 ---
                const int n = A.cols();
                Eigen::VectorXd col_norms(n);
                for (int j = 0; j < n; ++j)
                {
                    col_norms(j) = A.col(j).norm();
                }
                const double eps   = 1e-12;    // avoid division by zero
                const double dmin  = 1e-5;     // clamp factors to avoid extreme scaling
                const double dmax  = 1e+5;
                // Eigen::VectorXd dj = (col_norms.array() > eps).select(1.0 / col_norms.array(), 1.0); // scaling factor for each column
                dj = (col_norms.array() > eps).select(1.0 / col_norms.array(), 1.0); // scaling factor for each column, eps: caring for numerical stability
                dj = dj.cwiseMax(dmin).cwiseMin(dmax);
                // construct a column-scaling matrix D, where diagonal elements are dj
                D = dj.asDiagonal();

                // 2) Scaled Jacobian and transform bounds/constraints ---
                // column scaling, matrix multiplication
                Eigen::MatrixXd A_s = A * D;           // columns ~ unit-norm
                // // If you have simple bounds on the step x, convert to z: z = D^{-1} x
                // Eigen::VectorXd lb_z, ub_z;
                // if (lb_x.size() == n && ub_x.size() == n) {
                // // Dinv is just 1/dj
                // Eigen::VectorXd d_inv = dj.cwiseInverse();
                // lb_z = lb_x.cwiseProduct(d_inv);
                // ub_z = ub_x.cwiseProduct(d_inv);
                // }
                // // If you have general linear constraints on x: Acon * x in [Alb_x, Aub_x]
                // // convert to z: (Acon * D) * z in [Alb_x, Aub_x]
                // Eigen::MatrixXd Acon_z;
                // if (Acon.size() > 0) Acon_z = Acon * D;

                // 3) Build QP in z: min 1/2 z^T H_s z + h_s^T z ---
                //  done in the following scripts

                // Now that se have ||Ax - b||2 = ||(AD)(D^-1x) - b||2 = ||(A_s)z - b||2 
                //  (A_s) =    AD   : column-wise scaled Jacobian
                //    z   = (D^-1x) : scaled state vector (to be mapped back to x after solving QP)

                // update A to As (column-wise scaled A)
                A = A_s; 
            }
            timeLogger_.stop("[SQP]: Jacobian Column Scaling", __FUNCTION__, __LINE__);

            // build the QP formulation
            // min(xQP): 1/2 xQP.T @ HQP @ xQP + hQP.T @ xQP
            // s.t.    : lb  <=   xQP   <= ub
            //           Alb <= A @ xQP <= Aub

            Eigen::Matrix<double,6,6> HQP = Eigen::Matrix<double,6,6>::Zero();  // Hessian in QP
            Eigen::Matrix<double,6,1> hQP = Eigen::Matrix<double,6,1>::Zero();  // h matrix in QP

            timeLogger_.start("[SQP]: Hessian Computation", __FUNCTION__, __LINE__);
            // compute the weights first based on the robust kernel
            Eigen::VectorXd w;  
            if(config_.turn_on_robust_kernel_qp_active_set)
            {
                // if(config_.turn_on_mad_based_scaling_for_kernel_weights_qp_active_set)
                // {
                timeLogger_.start("[SQP]: Hessian Computation - Kernel Weights", __FUNCTION__, __LINE__);
                double s = 0.0; // default scaling just as an option manually tune it
                double c = 0.0; // default range just as an option manually tune it
                timeLogger_.start("[SQP]: computeWeightsFromResiduals", __FUNCTION__, __LINE__);
                computeWeightsFromResiduals(b_for_kernel, w, s, config_.robust_kernel_qp_active_set, c, true);
                timeLogger_.stop("[SQP]: computeWeightsFromResiduals", __FUNCTION__, __LINE__);
            }
            else
            {
                w = Eigen::VectorXd::Ones(b_for_kernel.size());
            }

            if (debug_print_qp_active_set_matrices)
            {
                RCLCPP_INFO_STREAM(get_logger(), "w: ");
                RCLCPP_INFO_STREAM(get_logger(), w.transpose());
            }

            timeLogger_.start("[SQP]: W = w.asDiagonal()", __FUNCTION__, __LINE__);
            // Eigen::MatrixXd W = w.asDiagonal(); // This is super expensive, keep this just in case
            Eigen::DiagonalMatrix<double, Eigen::Dynamic> W = w.asDiagonal();   // fast
            timeLogger_.stop("[SQP]: W = w.asDiagonal()", __FUNCTION__, __LINE__);

            timeLogger_.stop("[SQP]: Hessian Computation - Kernel Weights", __FUNCTION__, __LINE__);

            if(config_.qp_active_set_Hessian_computation == "matrix_multiplication")
            {
                // cleaner but slow way
                timeLogger_.start("[SQP]: Hessian Computation - Hessian (Mat. Multiplication)", __FUNCTION__, __LINE__);
                HQP = 2 * A.transpose() * W * A;
                hQP = - 2 * A.transpose() * W * b;
                timeLogger_.stop("[SQP]: Hessian Computation - Hessian (Mat. Multiplication)", __FUNCTION__, __LINE__);
            }
            else if(config_.qp_active_set_Hessian_computation == "for_loop")
            {
                // fast way
                timeLogger_.start("[SQP]: Hessian Computation - Hessian (For-loop)", __FUNCTION__, __LINE__);
                for (int i = 0; i < numResidual_; ++i) 
                {
                    double wi = w(i);
                    Eigen::Matrix<double,1,6> ai = A.row(i);
                    HQP.noalias() += 2.0 * wi * (ai.transpose() * ai);
                    hQP.noalias() += - 2.0 * wi * ai.transpose() * b(i);
                }
                timeLogger_.stop("[SQP]: Hessian Computation - Hessian (For-loop)", __FUNCTION__, __LINE__);
            }
            else
            {
                RCLCPP_INFO_STREAM(get_logger(), "Hessian Computation is not defined well. Set Hessian to zero.");
            }

            if(config_.turn_on_levenberg_marquardt_qp_active_set)
            {
                timeLogger_.start("[SQP]: Hessian Computation - LM Hessian", __FUNCTION__, __LINE__);
                const double lambda = config_.levenberg_marquardt_lambda_qp_active_set;
                if(config_.turn_on_levenberg_marquardt_qp_active_set_marquardt_damping)
                {
                    HQP += 2.0 * lambda * (HQP.diagonal().asDiagonal()); // // Marquardt-type damping, "2.0 *" is for conversion to QP formulation (* 1/2 afterward)
                }
                else
                {
                    HQP += 2.0 * lambda * Eigen::MatrixXd::Identity(6,6); // "2.0 *" is for conversion to QP formulation (* 1/2 afterward)
                }
                timeLogger_.stop("[SQP]: Hessian Computation - LM Hessian", __FUNCTION__, __LINE__);
            }
            timeLogger_.stop("[SQP]: Hessian Computation", __FUNCTION__, __LINE__);

            // solution vector
            Eigen::VectorXd xQP(6); // solution vector (roll, pitch, yaw, tx, ty, tz)

            // scaler inequality constraints
            Eigen::VectorXd lb(6); // scaler constraint vector (roll, pitch, yaw, tx, ty, tz)
            Eigen::VectorXd ub(6);
            if (t_bPrev2Kf_bPrevKf_ != Eigen::Vector3d::Zero() && config_.turn_on_ineq_constraints)
            {
                timeLogger_.start("[SQP]: Inequality Setting", __FUNCTION__, __LINE__);
                // linear velocity from the 2nd previous frame to the previous frame, expressed in the body at the 2nd previous frame
                const double linVel = t_bPrev2Kf_bPrevKf_.x() / (timePrevScanEnd_ - timePrev2ScanEnd_);

                const double maxStrAng = config_.vehicle_kinematic_constraints_max_steering_angle * (M_PI/180);   // [rad]
                // const double minStrAng = config_.vehicle_kinematic_constraints_min_steering_angle * (M_PI/180);  // [rad]
                const double whlbase = config_.vehicle_kinematic_constraints_wheel_base; // [m]
                double maxYawRate = linVel * std::tan(maxStrAng) / whlbase;
                // double minYawRate = linVel.x() * std::tan(-maxStrAng) / whlbase;
                if (linVel < 0) maxYawRate *= -1;
                double minYawRate = - maxYawRate;

                RCLCPP_INFO_STREAM(get_logger(), "----------------forward linear velocity-----------------: " << linVel << " [m/s]");
                RCLCPP_INFO_STREAM(get_logger(), "max yaw rate : " << maxYawRate * (180/M_PI) << " [deg/s]");
                RCLCPP_INFO_STREAM(get_logger(), "min yaw rate : " << minYawRate * (180/M_PI) << " [deg/s]");

                // !!! We have to impose constraints on AXIS-ANGLE VECTOR (alpha, beta, gamma) not really (roll, pitch, yaw) !!!
                // But we can approximate delta_yaw in body frame with the z element of the axis-angle vector
                // A concrete way is axis-angle vector -> rotation matrix -> Euler angle (yaw)
                //  and we can linearize the constraint by using Jacobian of Euler angle (yaw) wrt axis-angle vector
                //  axis-angle vector always starts from identity in body frame(the previous body pose)
                //  in that case Jacobian of Euler angle (yaw) wrt axis-angle vector is identity
                //   since the right Jacobian evaluated at the identity (Jr(0)) is identity.
                //  We can use this approximation because typically it would be a small rotation between scans.
                //  Thus, we can typically approximate delta_yaw in body frame with the z element of the axis-angle vector
                //  One potential failure is that if the body z-axis does not align with the true yaw axis, the approximation would not hold
                //  In that case, we need to perform and update a ground plane normal estimation and update body z-axis according to it.

                for (int i=0; i<6; ++i)   // initialize all the bounds as inf(no constraint essentially)
                {
                    lb(i) = - std::numeric_limits<double>::max();
                    ub(i) = std::numeric_limits<double>::max();
                }
                // put yaw rate constraints only on z-component of axis-angle representation
                lb(2) = minYawRate * (timePrevScanEnd_ - timePrev2ScanEnd_);    // need to scale actually
                ub(2) = maxYawRate * (timePrevScanEnd_ - timePrev2ScanEnd_);

                lb(5) = 0.0;    // need to scale actually
                ub(5) = 0.05;

                RCLCPP_INFO_STREAM(get_logger(), "Unscaled lb(2): " << lb(2)* (180/M_PI) << " [deg]");
                RCLCPP_INFO_STREAM(get_logger(), "Unscaled ub(2): " << ub(2)* (180/M_PI) << " [deg]");
                RCLCPP_INFO_STREAM(get_logger(), "[Constraint check] delta-yaw attempt (axis angle) = " << x(2)*(180/M_PI)<< " [deg]");
                RCLCPP_INFO_STREAM(get_logger(), "[Constraint check] Z attempt = " << x(5)<< " [m]");


                // if using column-wise Jacobian scaling for numerical stability, we also have to scale the constraint
                if(config_.turn_on_jacobian_column_scaling_qp_active_set)
                {
                    Eigen::VectorXd d_inv = dj.cwiseInverse();
                    //Eigen::VectorXd d_inv = dj;
                    lb = lb.cwiseProduct(d_inv);
                    ub = ub.cwiseProduct(d_inv);
                    RCLCPP_INFO_STREAM(get_logger(), "Scaled lb(2): " << lb(2));
                    RCLCPP_INFO_STREAM(get_logger(), "Scaled ub(2): " << ub(2));
                }

                timeLogger_.stop("[SQP]: Inequality Setting", __FUNCTION__, __LINE__);
            }
            else 
            {
                // no constraints
                lb = Eigen::VectorXd(); 
                ub = Eigen::VectorXd();
            }

            // linear inequality constraint (We don't enforce linear constraint for now)
            Eigen::MatrixXd constraintMatrix = Eigen::MatrixXd();
            Eigen::VectorXd Alb = Eigen::VectorXd();
            Eigen::VectorXd Aub = Eigen::VectorXd();

            if (debug_print_qp_active_set_matrices_jacobian_residual)
            {
                RCLCPP_INFO_STREAM(get_logger(), "A: " << A);
                RCLCPP_INFO_STREAM(get_logger(), "b: " << b);
            }

            if (debug_print_qp_active_set_matrices)
            {
                RCLCPP_INFO_STREAM(get_logger(), "HQP: ");
                RCLCPP_INFO_STREAM(get_logger(), HQP);
                RCLCPP_INFO_STREAM(get_logger(), "hQP: ");
                RCLCPP_INFO_STREAM(get_logger(), hQP);
                RCLCPP_INFO_STREAM(get_logger(), "lb: " << lb);
                RCLCPP_INFO_STREAM(get_logger(), "ub: " << ub);
                // RCLCPP_INFO_STREAM(get_logger(), "vel: " << vel);
                // RCLCPP_INFO_STREAM(get_logger(), "maxStrAng: " << maxStrAng);
                // RCLCPP_INFO_STREAM(get_logger(), "minStrAng: " << minStrAng);
                // RCLCPP_INFO_STREAM(get_logger(), "whlbase: " << whlbase);
                // RCLCPP_INFO_STREAM(get_logger(), "maxYawRate: " << maxYawRate);
                // RCLCPP_INFO_STREAM(get_logger(), "minYawRate: " << minYawRate);
                RCLCPP_INFO_STREAM(get_logger(), "constraintMatrix: " << constraintMatrix);
                RCLCPP_INFO_STREAM(get_logger(), "Alb: " << Alb);
                RCLCPP_INFO_STREAM(get_logger(), "Aub: " << Aub);
            }

            timeLogger_.start("[SQP]: Solve QP", __FUNCTION__, __LINE__);

            Eigen::VectorXd z_full(6);
            z_full.setZero();

            // Line search / outer loop parameters

            const double mu             = 10.0;   // penalty on violations in merit
            const double armijo_c       = 1e-2;   // Armijo parameter
            const double backtrack_beta = 0.5;    // alpha <- beta * alpha
            const int    max_backtrack  = 20;
            const double tol_step_norm  = 1e-6;

            // set solver parameters and solve
            qpmad::Solver qpmadSolver;   // Solver = SolverTemplate<double, Eigen::Dynamic, 1, Eigen::Dynamic>; in qpmad/solver.h
            qpmad::SolverParameters param;
            // param.hessian_type_ = qpmad::SolverParameters::HESSIAN_LOWER_TRIANGULAR;
            // qpmad::Solver::ReturnStatus status =
            //     qpmadSolver.solve(xQP, HQP, hQP, lb, ub, constraintMatrix, Alb, Aub, param);


            // [out] Vector primal – solution vector, mandatory, allocated if needed
            // [in,out] Matrix H – Hessian, mandatory, non-empty, factorized in-place
            // [in] Vector h – objective vector, mandatory, may be empty
            // [in] Vector lb – vector of lower bounds, may be omitted or may be empty consistently with ub
            // [in] Vector ub – vector of upper bounds, may be omitted or may be empty consistently with lb
            // [in] Matrix A – general constraints matrix, may be omitted with all general constraints, may be empty
            // [in] Vector Alb – lower bounds of general constraints, may be omitted with all general constraints, may be empty
            // [in] Vector Aub – upper bounds of general constraints, may be omitted (if A and Alb are present they are assumed to be equality constraints), may be empty
            // [in] SolverParameters param – solver parameters, may be omitted

            std::cout << "z_full.T = [" << z_full.transpose() << "]\n";

            // ----- 1) Solve the QP to get a "target" point x_qp -----
            Eigen::VectorXd zQP = z_full; // initial guess
            qpmad::Solver::ReturnStatus status =
                qpmadSolver.solve(zQP, HQP, hQP, lb, ub, constraintMatrix, Alb, Aub, param);



            // sanity check with Cholesky Decomposition linear solver
            if (debug_try_qp_active_set_Cholesky_Decomposition_unconstrained)
            {
                auto H_test = HQP / 2;
                auto h_test = - hQP / 2;
                zQP = H_test.ldlt().solve(-h_test);
                //xQP = H_test.ldlt().solve(-h_test);
            }

            // check optimization status
            if (status != qpmad::Solver::OK)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Error in solving inequality constrained optimization problem with QPmad library.");
            }

            // extract the optimization status
            Eigen::VectorXd dual;
            Eigen::Matrix<qpmad::MatrixIndex, Eigen::Dynamic, 1> indices;
            Eigen::Matrix<bool, Eigen::Dynamic, 1> is_lower;
            qpmadSolver.getInequalityDual(dual, indices, is_lower);

            //=======================================LINE SEARCH BEGIN ===================================================

            // Direction = target - current
            Eigen::VectorXd dir = zQP - z_full;
            double dir_norm = dir.norm();
            std::cout << "dir norm = " << dir_norm << "\n";

            if (dir_norm < tol_step_norm)
            {
                std::cout << "Step is tiny, stopping.\n";
                break;
            }

            // ----- 2) Compute merit at current point and directional slope -----
            double phi_x = merit(z_full, HQP, hQP, mu, constraintMatrix, Alb, Aub, lb, ub);
            Eigen::VectorXd g = HQP*z_full + hQP;
            double slope = g.dot(dir);

            std::cout << "merit(x) = " << phi_x << ", slope = " << slope << "\n";

            if (slope >= 0.0)
            {
                std::cout << "Warning: direction is not a descent direction, stopping.\n";
                break;
            }

            // ----- 3) Backtracking line search on alpha -----
        double alpha   = 1.0;
        bool accepted  = false;

        for (int bt = 0; bt < max_backtrack; ++bt)
        {
            std::cout << "  iter_cnt = " << iter_cnt << "  iter_qp = " << iter_qp <<"  bt = " << bt;
            Eigen::VectorXd z_trial = z_full + alpha * dir;
            double phi_trial = merit(z_trial, HQP, hQP, mu, constraintMatrix, Alb, Aub, lb, ub);

            double armijo_rhs = phi_x + armijo_c * alpha * slope;
            //double armijo_rhs = armijo_c * alpha * slope;

            std::cout << "  alpha = " << alpha
                      << ", merit(z_trial) = " << phi_trial
                      << ", Armijo RHS = " << armijo_rhs << "\n";

            if (phi_trial <= armijo_rhs)
            {
                // Accept step
                z_full = z_trial;
                accepted = true;
                std::cout << "  Step accepted.\n";
                break;
            }
            else
            {
                // Shrink alpha
                alpha *= backtrack_beta;
            }
        }

        if (!accepted)
        {
            std::cout << "Line search failed to find acceptable step, stopping.\n";
            break;
        }





            if (debug_print_qp_active_set_solution_analysis)
            {
                if (dual.size() > 0)
                {
                    RCLCPP_INFO_STREAM(get_logger(), "Number of active Inquality Constraints: " << dual.size());
                    RCLCPP_INFO_STREAM(get_logger(), "Do Constraints satisfied? 0 == satisfied: ");
                    if (constraintMatrix.size() == 0) 
                    {
                        RCLCPP_INFO_STREAM(get_logger(), "No general constraints (A is empty).");
                    } 
                    //else if (constraintMatrix.cols() != xQP.size())
                    else if (constraintMatrix.cols() != z_full.size()) 
                    {
                        RCLCPP_WARN_STREAM(get_logger(), "Constraint matrix cols != x size.");
                    } 
                    else 
                    {
                        //RCLCPP_INFO_STREAM(get_logger(), "A*x = " << (constraintMatrix * xQP).transpose());
                        RCLCPP_INFO_STREAM(get_logger(), "A*x = " << (constraintMatrix * z_full).transpose());
                    }
                    // std::cout << constraintMatrix*xQP << std::endl;
                    // RCLCPP_INFO_STREAM(get_logger(), constraintMatrix*xQP);
                }
            }

            // scale the solution back if necessary
            if (config_.turn_on_jacobian_column_scaling_qp_active_set)
            {   
                //Eigen::VectorXd z = xQP;
                Eigen::VectorXd z = z_full;
                //    z   = (D^-1 x) : scaled state vector (to be mapped back to x after solving QP)
                // --- 5) Map back to original variables (unscale) ---
                x = D * z;  // x = Dz
            }
            else
            {
                //x = xQP;
                x = z_full;
            }
            timeLogger_.stop("[SQP]: Solve QP", __FUNCTION__, __LINE__);

            // ------------------- TURN DETECTOR START------------------------
            double yaw_inc_rad = x(2);                           // axis-angle rotation about z [rad]
            double yaw_inc_deg = yaw_inc_rad * 180.0 / M_PI;     // convert to degrees

            double delta_Z_m = x(5);

            // First: update stats (only if finite)
            if (std::isfinite(yaw_inc_deg))
            {
                double abs_yaw = std::abs(yaw_inc_deg);
                double abs_Z = std::abs(delta_Z_m);

                // initialize min/max on first iteration
                if (config_.yaw_deg_count == 0)
                {
                    config_.yaw_deg_max = abs_yaw;
                    config_.yaw_deg_min = abs_yaw;
                    config_.Z_max = abs_Z;
                    config_.Z_min = abs_Z;
                }

                // update min / max
                config_.yaw_deg_max = std::max(config_.yaw_deg_max, abs_yaw);
                config_.yaw_deg_min = std::min(config_.yaw_deg_min, abs_yaw);

                config_.Z_max = std::max(config_.Z_max, abs_Z);
                config_.Z_min = std::min(config_.Z_min, abs_Z);

                // update running sum & count for average
                config_.yaw_deg_sum += abs_yaw;
                config_.Z_sum += abs_Z;
                config_.yaw_deg_count++;

                // (Optional) print stats every N steps
                if (config_.yaw_deg_count % 20 == 0)
                {
                    double avg_yaw = config_.yaw_deg_sum / config_.yaw_deg_count;
                    double avg_Z = config_.Z_sum / config_.yaw_deg_count;
                    RCLCPP_INFO_STREAM(get_logger(),
                        "[TURN STATS] max yaw increment =" << config_.yaw_deg_max 
                        << " deg, min yaw increment =" << config_.yaw_deg_min
                        << " deg, avg yaw increment =" << avg_yaw
                        << " deg" 
                        << " , max Z increment =" << config_.Z_max
                        << " m, min Z increment =" << config_.Z_min
                        << " m, avg Z increment =" << avg_Z
                        << " m ("
                        << config_.yaw_deg_count << " samples)");
                }
            }


            // Sanity checks: catch NaN, Inf, or huge spikes
            if (!std::isfinite(yaw_inc_rad) || std::abs(yaw_inc_rad) > 1.0) {  
                RCLCPP_ERROR_STREAM(get_logger(),
                    "BAD yaw increment detected: " << yaw_inc_rad 
                    << " rad (" << yaw_inc_deg << " deg). Possible solver/scaling issue.");
            } 
            else 
            {
                // Classify turning behavior
                if (std::abs(yaw_inc_deg) < 0.05) {
                    // basically straight
                    RCLCPP_INFO_STREAM(get_logger(),
                        "  Straight: Δyaw = " << yaw_inc_deg << " deg");
                } 
                else if (std::abs(yaw_inc_deg) < 0.5) {
                    // small adjustment
                    RCLCPP_INFO_STREAM(get_logger(),
                        "  Slight turn: Δyaw = " << yaw_inc_deg << " deg");
                } 
                else if (std::abs(yaw_inc_deg) < 2.0) {
                    // real turning
                    RCLCPP_WARN_STREAM(get_logger(),
                        "  Turning: Δyaw = " << yaw_inc_deg << " deg");
                } 
                else {
                    // sharp turn approaching constraint
                    RCLCPP_ERROR_STREAM(get_logger(),
                        "  SHARP TURN: Δyaw = " << yaw_inc_deg << " deg");
                }
            }
            // --------------------------TURN DETECTOR END---------------------------------


            if (debug_print_qp_active_set_solution)
            {
                // for (unsigned int i=0; i<6; ++i)
                // {
                //     RCLCPP_INFO_STREAM(get_logger(), "xQP(" << i << "): " << xQP(i));
                // }
                // for (unsigned int i=0; i<6; ++i)
                // {
                //     RCLCPP_INFO_STREAM(get_logger(), "xQPfloat(" << i << "): " << xQPfloat(i));
                // }
                for (unsigned int i=0; i<6; ++i)
                {
                    RCLCPP_INFO_STREAM(get_logger(), "x(" << i << "): " << x(i));
                }
            }

            timeLogger_.start("[SQP]: Update solution pose", __FUNCTION__, __LINE__);

            // Map state vector x (6DoF) to a transformation matrix via exp map in Sophus
            // reorder the state vector to adjust to Sophus twist 
            Eigen::Vector<double, 6> a;
            a.tail<3>() = x.head<3>(); // axis-angle vector (alpha, beta, gamma)
            a.head<3>() = x.tail<3>(); // translation vector (x,y,z)
            const Sophus::SE3d exp_x = Sophus::SE3d::exp(a);
            Sophus::SE3d T(q_w_bCurrKf_.toRotationMatrix(), t_w_bCurrKf_);

            if (config_.pose_increment_multiplication == "right")
            {
                T = T * exp_x; // update by right multiplication
            }
            else if (config_.pose_increment_multiplication == "left")
            {
                T = exp_x * T; // update by left multiplication
            }
            else
            {
                RCLCPP_INFO_STREAM(get_logger(), "Left or Right Pose Update is not defined. Current Pose is not updated.");
            }

            q_w_bCurrKf_ = Eigen::Quaterniond(T.rotationMatrix());
            q_w_bCurrKf_.normalize();
            t_w_bCurrKf_ = T.translation();
            timeLogger_.stop("[SQP]: Update solution pose", __FUNCTION__, __LINE__);

            if (debug_print_qp_active_set_solution_pose_update)
            {
                RCLCPP_INFO_STREAM(get_logger(), "t_delta: " << exp_x.translation());
                RCLCPP_INFO_STREAM(get_logger(), "q_delta: " << Eigen::Quaterniond(exp_x.rotationMatrix()));
                RCLCPP_INFO_STREAM(get_logger(), "t_w_bCurrKf_: " << t_w_bCurrKf_.transpose());
                RCLCPP_INFO_STREAM(get_logger(), "q_w_bCurrKf_: " << q_w_bCurrKf_);
            }

        } // QP inner iteration (SQP)

        pointScanCurrInBForResidual_.clear();
        planeNormalForResidual_.clear();
        planeDistFromOriginForResidual_.clear();

    }
    // end of the point-to-plane icp with inequality constraints
}


void Frontend::solveLeastSquares_InequalityConstraints_BFGS()
{
    if(!config_.use_voxel_map)
    {
        // set the local map point cloud to the kd_tree to nearest neighbor search
        timeLogger_.start("kdTreeMapLocal_->setInputCloud", __FUNCTION__, __LINE__);
        kdTreeMapLocal_->setInputCloud(cloudMapLocalDs_);
        timeLogger_.stop("kdTreeMapLocal_->setInputCloud", __FUNCTION__, __LINE__);
    }

    // // This is redundant because the current pose is updated with an Initial guess in setInitialPoseLO()
    // // transform the current absolute pose based on initial guess (to be used for preparePointPlaneResidual())
    // t_w_bCurrKf_ = q_w_bCurrKf_ * t_bPrevKf_bCurrKf_initGuess_ + t_w_bCurrKf_;
    // q_w_bCurrKf_ = (q_w_bCurrKf_ * q_bPrevKf_bCurrKf_initGuess_).normalized();

    if (debug_print_qp_active_set_init_guess)
    {
        RCLCPP_INFO_STREAM(get_logger(), "t_bPrevKf_bCurrKf_initGuess_: " << t_bPrevKf_bCurrKf_initGuess_);
        RCLCPP_INFO_STREAM(get_logger(), "q_bPrevKf_bCurrKf_initGuess_: " << q_bPrevKf_bCurrKf_initGuess_);
        RCLCPP_INFO_STREAM(get_logger(), "t_w_bCurrKf_: " << t_w_bCurrKf_.transpose());
        RCLCPP_INFO_STREAM(get_logger(), "q_w_bCurrKf_: " << q_w_bCurrKf_);
    }

    Eigen::VectorXd x(6); // state vector (roll, pitch, yaw, tx, ty, tz)

    // ICP iteration (point-to-plane)
    for (int iter_cnt = 0; iter_cnt < config_.icp_iteration_num; iter_cnt++) 
    {
        RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

        if (debug_print_qp_icp_iter_num)
            RCLCPP_INFO_STREAM(get_logger(), "icp_iter_num: " << iter_cnt);

        RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

        // moved to here to keep the last residual parameters for Hessian analysis
        pointScanCurrInBForResidual_.clear();
        planeNormalForResidual_.clear();
        planeDistFromOriginForResidual_.clear();

        RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

        timeLogger_.start("preparePointPlaneResidual", __FUNCTION__, __LINE__);
        preparePointPlaneResidual();
        timeLogger_.stop("preparePointPlaneResidual", __FUNCTION__, __LINE__);

        RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

        // Set up parameters
        LBFGSpp::LBFGSParam<double> param;
        param.epsilon = 1e-6;
        param.max_iterations = 100;

        // Initial guess 
        Sophus::SE3d T(q_w_bCurrKf_.toRotationMatrix(), t_w_bCurrKf_);
        Eigen::VectorXd x = Eigen::VectorXd::Zero(6); // init guess of pose increment

        RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

        // Eigen::VectorXd x = T.log();
        // x = T.log();

        // Create solver and function object
        LBFGSpp::LBFGSSolver<double> solver(param);
        // PointToPlaneCostForLBFGSpp fun(numResidual_, pointScanCurrInBForResidual_, planeNormalForResidual_, planeDistFromOriginForResidual_);    
        PointToPlaneCostForLBFGSpp fun(T, numResidual_, pointScanCurrInBForResidual_, planeNormalForResidual_, planeDistFromOriginForResidual_);       

        RCLCPP_INFO_STREAM(get_logger(), "x: " << x);
        RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

        // x will be overwritten to be the best point found
        double fx;
        int niter = solver.minimize(fun, x, fx);

        RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

        std::cout << niter << " iterations" << std::endl;
        std::cout << "x = \n" << x.transpose() << std::endl;
        std::cout << "f(x) = " << fx << std::endl;

        // Map state vector x (6DoF) to a transformation matrix via exp map in Sophus
        // reorder the state vector to adjust to Sophus twist 
        Eigen::Vector<double, 6> a;
        a.tail<3>() = x.head<3>(); // axis-angle vector (alpha, beta, gamma)
        a.head<3>() = x.tail<3>(); // translation vector (x,y,z)
        const Sophus::SE3d exp_x = Sophus::SE3d::exp(a);
        // Sophus::SE3d T(q_w_bCurrKf_.toRotationMatrix(), t_w_bCurrKf_);

        RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

        if (config_.pose_increment_multiplication == "right")
        {
            T = T * exp_x; // update by right multiplication
        }
        else if (config_.pose_increment_multiplication == "left")
        {
            T = exp_x * T; // update by left multiplication
        }
        else
        {
            RCLCPP_INFO_STREAM(get_logger(), "Left or Right Pose Update is not defined. Current Pose is not updated.");
        }

        RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

        q_w_bCurrKf_ = Eigen::Quaterniond(T.rotationMatrix());
        q_w_bCurrKf_.normalize();
        t_w_bCurrKf_ = T.translation();

        RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
    }
    // end of the point-to-plane icp with inequality constraints
}


void Frontend::solveLeastSquares_InequalityConstraints_IFOPT()
{
    if(!config_.use_voxel_map)
    {
        // set the local map point cloud to the kd_tree to nearest neighbor search
        timeLogger_.start("kdTreeMapLocal_->setInputCloud", __FUNCTION__, __LINE__);
        kdTreeMapLocal_->setInputCloud(cloudMapLocalDs_);
        timeLogger_.stop("kdTreeMapLocal_->setInputCloud", __FUNCTION__, __LINE__);
    }

    // // This is redundant because the current pose is updated with an Initial guess in setInitialPoseLO()
    // // transform the current absolute pose based on initial guess (to be used for preparePointPlaneResidual())
    // t_w_bCurrKf_ = q_w_bCurrKf_ * t_bPrevKf_bCurrKf_initGuess_ + t_w_bCurrKf_;
    // q_w_bCurrKf_ = (q_w_bCurrKf_ * q_bPrevKf_bCurrKf_initGuess_).normalized();

    if (debug_print_qp_active_set_init_guess)
    {
        RCLCPP_INFO_STREAM(get_logger(), "t_bPrevKf_bCurrKf_initGuess_: " << t_bPrevKf_bCurrKf_initGuess_);
        RCLCPP_INFO_STREAM(get_logger(), "q_bPrevKf_bCurrKf_initGuess_: " << q_bPrevKf_bCurrKf_initGuess_);
        RCLCPP_INFO_STREAM(get_logger(), "t_w_bCurrKf_: " << t_w_bCurrKf_.transpose());
        RCLCPP_INFO_STREAM(get_logger(), "q_w_bCurrKf_: " << q_w_bCurrKf_);
    }

    Eigen::VectorXd x(6); // state vector (roll, pitch, yaw, tx, ty, tz)
    // scaler inequality constraints
    Eigen::VectorXd lb(6); // scaler constraint vector (roll, pitch, yaw, tx, ty, tz)
    Eigen::VectorXd ub(6);
    
    // no bound
    // for (int i=0; i<6; ++i)   // initialize all the bounds as inf(no constraint essentially)
    // {
    //     lb(i) = - std::numeric_limits<double>::max();
    //     ub(i) = std::numeric_limits<double>::max();
    // }

    // put some realistic bound
    // rotation
    for (int i=0; i<3; ++i)   // initialize all the bounds as inf(no constraint essentially)
    {
        lb(i) = - 3.14 / 4; // [rad] = - 45 [deg]
        ub(i) = 3.14 / 4;   // [rad] = + 45 [deg]
    }
    // translation
    for (int i=3; i<6; ++i)   // initialize all the bounds as inf(no constraint essentially)
    {
        lb(i) = - 2; // - 2 m
        ub(i) = 2;   // + 2 m
    }

    // ICP iteration (point-to-plane)
    for (int iter_cnt = 0; iter_cnt < config_.icp_iteration_num; iter_cnt++) 
    {
        // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

        if (debug_print_qp_icp_iter_num)
            RCLCPP_INFO_STREAM(get_logger(), "icp_iter_num: " << iter_cnt);

        // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

        // moved to here to keep the last residual parameters for Hessian analysis
        pointScanCurrInBForResidual_.clear();
        planeNormalForResidual_.clear();
        planeDistFromOriginForResidual_.clear();

        // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

        timeLogger_.start("preparePointPlaneResidual", __FUNCTION__, __LINE__);
        preparePointPlaneResidual();
        timeLogger_.stop("preparePointPlaneResidual", __FUNCTION__, __LINE__);

        // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

        // Initial guess
        Sophus::SE3d T(q_w_bCurrKf_.toRotationMatrix(), t_w_bCurrKf_); // init guess of abs. pose
        Eigen::VectorXd x = Eigen::VectorXd::Zero(6); // init guess of pose increment
        // Initial guess starts from zero since we already compose q_w_bCurrKf_ and t_w_bCurrKf_ with init guesses

        // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

        // Eigen::VectorXd x = T.log();
        // x = T.log();
        
        // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

        // ======== SOLVE ICP ITERATION WITH IFOPT ========  
        // Define the solver independent problem
        ifopt::Problem nlp;
        auto twist = std::make_shared<TwistVariables>("twist");
        twist->SetInitialVariables(x, lb, ub, T);
        // twist->SetInitialVariables(x, T);
        nlp.AddVariableSet(twist);
        // check bounds
        auto bounds = twist->GetBounds();
        for (int i = 0; i < 6; ++i) {
        std::cout << "twist[" << i << "] bounds = ["
                    << bounds.at(i).lower_ << ", "
                    << bounds.at(i).upper_ << "]\n";
        }

        // auto kinematicConstraint = std::make_shared<KinematicConstraint>("kinematic_constraint");
        // kinematicConstraint->SetConstraints(lb, ub);
        // nlp.AddConstraintSet(kinematicConstraint);

        auto pointToPlaneCost = std::make_shared<PointPlaneCost>("point_to_plane_cost");
        pointToPlaneCost->SetInitialPoseAndResidual(T, numResidual_, 
                    pointScanCurrInBForResidual_, planeNormalForResidual_, planeDistFromOriginForResidual_);
        nlp.AddCostSet(pointToPlaneCost);

        // sanity check for gradient and cost        
        Eigen::VectorXd x0 = twist->GetValues();
        double c0;
        Eigen::SparseMatrix<double, Eigen::RowMajor> J0;
        // also compute the initial residual and fix the robust weights
        pointToPlaneCost->ComputeCostAndJacobian(x0, c0, J0);
        std::cout << "Initial cost: " << c0
                << ", grad norm: " << Eigen::VectorXd(J0.transpose()).norm() << std::endl;

        // Initialize solver and options: ref https://coin-or.github.io/Ipopt/OPTIONS.html
        ifopt::IpoptSolver ipopt;
        ipopt.SetOption("linear_solver", "mumps");
        ipopt.SetOption("jacobian_approximation", "exact");   
        // ipopt.SetOption("jacobian_approximation", "finite-difference-values");   
        // ipopt.SetOption("gradient_approximation", "finite-difference-values");   
        ipopt.SetOption("hessian_approximation", "limited-memory");  
        // ipopt.SetOption("hessian_approximation", "exact");  
        // ipopt.SetOption("nlp_scaling_method", "gradient-based");  

        ipopt.SetOption("tol", 1e-4);
        ipopt.SetOption("acceptable_tol", 1e-2);
        ipopt.SetOption("max_iter", 50);
        ipopt.SetOption("print_level", 5);

        // ipopt.SetOption("derivative_test", "first-order");
        // ipopt.SetOption("derivative_test_tol", 1e-4);


        // Solve
        ipopt.Solve(nlp);   
        // std::cout << nlp.GetOptVariables()->GetValues().transpose() << std::endl;
        RCLCPP_INFO_STREAM(get_logger(), "IFOPT SOLUTION: " << nlp.GetOptVariables()->GetValues().transpose());
        // ======== SOLVE ICP ITERATION WITH IFOPT ======== 

        x = nlp.GetOptVariables()->GetValues();

        // Map state vector x (6DoF) to a transformation matrix via exp map in Sophus
        // reorder the state vector to adjust to Sophus twist 
        Eigen::Vector<double, 6> a;
        a.tail<3>() = x.head<3>(); // axis-angle vector (alpha, beta, gamma)
        a.head<3>() = x.tail<3>(); // translation vector (x,y,z)
        const Sophus::SE3d exp_x = Sophus::SE3d::exp(a);
        // Sophus::SE3d T(q_w_bCurrKf_.toRotationMatrix(), t_w_bCurrKf_);

        // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

        if (config_.pose_increment_multiplication == "right")
        {
            T = T * exp_x; // update by right multiplication
        }
        else if (config_.pose_increment_multiplication == "left")
        {
            T = exp_x * T; // update by left multiplication
        }
        else
        {
            RCLCPP_INFO_STREAM(get_logger(), "Left or Right Pose Update is not defined. Current Pose is not updated.");
        }

        // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);

        q_w_bCurrKf_ = Eigen::Quaterniond(T.rotationMatrix());
        q_w_bCurrKf_.normalize();
        t_w_bCurrKf_ = T.translation();

        // RCLCPP_INFO_STREAM(get_logger(), __FUNCTION__ << __LINE__);
    }
    // end of the point-to-plane icp with inequality constraints
}

// void Frontend::evaluateStepSizeTrustRegionLM(Eigen::Matrix<double,6,6> JTJ, Eigen::Matrix<double,6,1> JTr, Eigen::VectorXd w)
// {
//     // solve delta_x = -(JTJ)JTr
    
//     // compute f(x)
//     // compute f(x + delta_x)
    
//     // compute m(0) = f(x)
//     // compute m(delta_x) = f(x) + J(x)*delta_x + 1/2*delta_x.T*(JTJ)*delta_x

//     // compute step_quality: rho = (f(x)-f(x+delta_x))/(m(0)-m(delta_x))
//     // 

//     // from Ceres:
//     // new_model_cost
//     //  = 1/2 [f + J * step]^2
//     //  = 1/2 [ f'f + 2f'J * step + step' * J' * J * step ]
//     // model_cost_change
//     //  = cost - new_model_cost
//     //  = f'f/2  - 1/2 [ f'f + 2f'J * step + step' * J' * J * step]
//     //  = -f'J * step - step' * J' * J * step / 2
//     //  = -(J * step)'(f + J * step / 2)


// }

double Frontend::computeResidualWithDeltaX_OriginalObjective(Eigen::VectorXd delta_x, Eigen::VectorXd weights)
{
    const Sophus::SE3d T(q_w_bCurrKf_.toRotationMatrix(), t_w_bCurrKf_);

    Eigen::Vector<double, 6> a;
    a.tail<3>() = delta_x.head<3>(); // axis-angle vector (alpha, beta, gamma)
    a.head<3>() = delta_x.tail<3>(); // translation vector (x,y,z)

    const Sophus::SE3d exp_x = Sophus::SE3d::exp(a);
    const Sophus::SE3d Twb = T * exp_x;

    const Eigen::Matrix3d Rwb = Twb.rotationMatrix();
    const Eigen::Vector3d twb = Twb.translation();

    double r2 = 0.0;
    for (size_t i=0; i < numResidual_; ++i){
        const Eigen::Vector3d wp = Rwb * pointScanCurrInBForResidual_.at(i) + twb;
        const double r = planeNormalForResidual_.at(i).transpose() * wp + planeDistFromOriginForResidual_.at(i);
        r2 += r * r * weights(i);
    }
    
    return 0.5 * r2;
}


// double Frontend::computeModelPredictedReduction(Eigen::VectorXd delta_x, Eigen::Matrix<double,6,6> ATWA, Eigen::Matrix<double,6,1> ATWb)
double Frontend::computeModelPredictedReduction(Eigen::Vector<double, 6> delta_x, Eigen::Matrix<double,6,6> ATWA, Eigen::Matrix<double,6,1> ATWb)
{   
    // HQP and hQP should be weighted already
    // return 1/2 * delta_x.transpose() * ATWA * delta_x + ATWb.transpose() * delta_x;
    return 0.5 * (delta_x.transpose() * ATWA * delta_x)(0,0) + (ATWb.transpose() * delta_x)(0,0);
}

} // namespace lo_dev
