#include "lo_dev/parameter/parameter.h"

std::string IMU_TOPIC;
std::string LIDAR_TOPIC;
std::string ODOM_TOPIC;
std::string ProjectName;

std::string MAP_FRAME;
std::string ODOM_FRAME;
std::string BASE_FRAME;
std::string IMU_FRAME;
std::string LIDAR_FRAME;

// Always identity, but explicitly declared for clarity
Eigen::Matrix3d R_MAP_ODOM;
Eigen::Vector3d t_MAP_ODOM;
Eigen::Quaterniond q_MAP_ODOM;

// Always identity, but explicitly declared for clarity
Eigen::Matrix3d R_BASE_IMU;
Eigen::Vector3d t_BASE_IMU;
Eigen::Quaterniond q_BASE_IMU;

// Provided from config.yaml
Eigen::Matrix3d R_IMU_LIDAR;
Eigen::Vector3d t_IMU_LIDAR;
Eigen::Quaterniond q_IMU_LIDAR;

float IMU_ACC_X_LIMIT;
float IMU_ACC_Y_LIMIT;
float IMU_ACC_Z_LIMIT;

std::string SENSOR;

SensorType sensor;


// for debugging 
bool debug_print_imu_forward_propagation_state;
bool debug_print_get_imu_pose_at_measurement_time;
bool debug_print_imu_pose_timeline;
bool debug_print_deskew_pointcloud;
bool debug_print_initial_guess_lo;
bool debug_print_relative_transform_lo;
bool debug_print_num_residuals;
bool debug_print_point_plane_residual_preparation;
bool debug_print_scan_imu_time_sync;
bool debug_print_first_point_in_current_scan;
bool debug_print_cloud_map_size;

// for initialization parameter
bool init_imu_init_fastlio2;

// for imu handling parameter
bool imu_isotropic_scale_calibration_to_G;
bool imu_enalbe_init_guess_for_lo;


void getExtrinsics_T_A_B(
    rclcpp::Node::SharedPtr node, std::string nameFrameA, std::string nameFrameB, 
    Eigen::Matrix3d& R_A_B, Eigen::Vector3d& t_A_B, Eigen::Quaterniond& q_A_B
    )
{
    std::string R_str = "R_" + nameFrameA + "_" + nameFrameB;
    std::string t_str = "t_" + nameFrameA + "_" + nameFrameB;
    std::string q_str = "q_" + nameFrameA + "_" + nameFrameB;

    double ida[] = { 1.0,  0.0,  0.0,
                     0.0,  1.0,  0.0,
                     0.0,  0.0,  1.0};
    std::vector < double > id(ida, std::end(ida));
    std::vector<double> R_flat;
    Eigen::Matrix3d R_map;
    node->declare_parameter(R_str, id);
    node->get_parameter(R_str, R_flat);
    R_map = Eigen::Map<const Eigen::Matrix<double,-1,-1,Eigen::RowMajor>>(R_flat.data(), 3, 3);
    R_A_B = R_map;

    double zea[] = {0.0, 0.0, 0.0};
    std::vector < double > ze(zea, std::end(zea));
    std::vector<double> t_flat;
    Eigen::Vector3d t_map;
    node->declare_parameter(t_str, ze);
    node->get_parameter(t_str, t_flat);
    t_map = Eigen::Map<const Eigen::Matrix<double,-1,-1,Eigen::RowMajor>>(t_flat.data(),3,1);
    t_A_B = t_map;

    q_A_B = Eigen::Quaterniond(R_A_B);

    RCLCPP_INFO_STREAM(node->get_logger(), R_str << " = [" << R_A_B(0,0) << ", " << R_A_B(0,1) << ", " << R_A_B(0,2) << " // "
                                                             << R_A_B(1,0) << ", " << R_A_B(1,1) << ", " << R_A_B(1,2) << " // "
                                                             << R_A_B(2,0) << ", " << R_A_B(2,1) << ", " << R_A_B(2,2) << "]");

    RCLCPP_INFO_STREAM(node->get_logger(), t_str << " = [" << t_A_B.x() << ", " 
                                                             << t_A_B.y() << ", "
                                                             << t_A_B.z() << "]");

    RCLCPP_INFO_STREAM(node->get_logger(), q_str << " = [" << q_A_B.w() << ", " 
                                                             << q_A_B.x() << ", "
                                                             << q_A_B.y() << ", " 
                                                             << q_A_B.z() << "]");
}



bool readGlobalparam(rclcpp::Node::SharedPtr node)
{
    node->declare_parameter<std::string>("imu_topic","imu/data");
    node->declare_parameter<std::string>("lidar_topic","velodyne_points");
    node->declare_parameter<std::string>("odom_topic","integrated_to_init");

    node->declare_parameter<std::string>("map_frame", "map");
    node->declare_parameter<std::string>("odom_frame", "odom");
    node->declare_parameter<std::string>("base_frame", "base_link");
    node->declare_parameter<std::string>("imu_frame", "imu_link");
    node->declare_parameter<std::string>("lidar_frame", "lidar_link");

    node->declare_parameter<std::string>("PROJECT_NAME", "");
    node->declare_parameter<std::string>("sensor", "ouster");

    node->declare_parameter<double>("imu_acc_x_limit", 0.5);
    node->declare_parameter<double>("imu_acc_y_limit", 0.2);
    node->declare_parameter<double>("imu_acc_z_limit", 0.4);

    LIDAR_TOPIC = node->get_parameter("lidar_topic").as_string();
    IMU_TOPIC = node->get_parameter("imu_topic").as_string();
    ODOM_TOPIC = node->get_parameter("odom_topic").as_string();

    MAP_FRAME = node->get_parameter("map_frame").as_string();
    ODOM_FRAME = node->get_parameter("odom_frame").as_string();
    BASE_FRAME = node->get_parameter("base_frame").as_string();
    IMU_FRAME = node->get_parameter("imu_frame").as_string();
    LIDAR_FRAME = node->get_parameter("lidar_frame").as_string();

    ProjectName = node->get_parameter("PROJECT_NAME").as_string();
    SENSOR = node->get_parameter("sensor").as_string();

    IMU_ACC_X_LIMIT = node->get_parameter("imu_acc_x_limit").as_double();
    IMU_ACC_Y_LIMIT = node->get_parameter("imu_acc_y_limit").as_double();
    IMU_ACC_Z_LIMIT = node->get_parameter("imu_acc_z_limit").as_double();
    //check whether sensor is support 
    const std::unordered_map<std::string, SensorType> sensorTypeMap = {
        {"velodyne", SensorType::VELODYNE},
        {"ouster", SensorType::OUSTER},
        {"livox", SensorType::LIVOX}
    };

    if (sensorTypeMap.find(SENSOR) == sensorTypeMap.end()) 
    {
        RCLCPP_ERROR(node->get_logger(), "Unsupported sensor type: %s", SENSOR.c_str());
        return false;
    }
    
    RCLCPP_INFO(node->get_logger(), "LIDAR_TOPIC %s", LIDAR_TOPIC.c_str());
    RCLCPP_INFO(node->get_logger(), "IMU_TOPIC %s", IMU_TOPIC.c_str());
    RCLCPP_INFO(node->get_logger(), "ODOM_TOPIC %s", ODOM_TOPIC.c_str());
    RCLCPP_INFO(node->get_logger(), "MAP_FRAME %s", MAP_FRAME.c_str());
    RCLCPP_INFO(node->get_logger(), "ODOM_FRAME %s", ODOM_FRAME.c_str());
    RCLCPP_INFO(node->get_logger(), "BASE_FRAME %s", BASE_FRAME.c_str());
    RCLCPP_INFO(node->get_logger(), "IMU_FRAME %s", IMU_FRAME.c_str());
    RCLCPP_INFO(node->get_logger(), "LIDAR_FRAME %s", LIDAR_FRAME.c_str());
    RCLCPP_INFO(node->get_logger(), "ProjectName %s", ProjectName.c_str());
    RCLCPP_INFO(node->get_logger(), "SENSOR %s", SENSOR.c_str());

    getExtrinsics_T_A_B(node, "map", "odom", R_MAP_ODOM, t_MAP_ODOM, q_MAP_ODOM);
    getExtrinsics_T_A_B(node, "base", "imu", R_BASE_IMU, t_BASE_IMU, q_BASE_IMU);
    getExtrinsics_T_A_B(node, "imu", "lidar", R_IMU_LIDAR, t_IMU_LIDAR, q_IMU_LIDAR);


    // ---------- for debugging parameter ---------- 
    node->declare_parameter<bool>("debug.print_imu_forward_propagation_state", false);
    node->declare_parameter<bool>("debug.print_get_imu_pose_at_measurement_time", false);
    node->declare_parameter<bool>("debug.print_imu_pose_timeline", false);
    node->declare_parameter<bool>("debug.print_deskew_pointcloud", false);
    node->declare_parameter<bool>("debug.print_initial_guess_lo", false);
    node->declare_parameter<bool>("debug.print_relative_transform_lo", false);
    node->declare_parameter<bool>("debug.print_num_residuals", false);
    node->declare_parameter<bool>("debug.print_point_plane_residual_preparation", false);
    node->declare_parameter<bool>("debug.print_scan_imu_time_sync", false);
    node->declare_parameter<bool>("debug.print_first_point_in_current_scan", false);
    node->declare_parameter<bool>("debug.print_cloud_map_size", false);

    debug_print_imu_forward_propagation_state = node->get_parameter("debug.print_imu_forward_propagation_state").as_bool();
    debug_print_get_imu_pose_at_measurement_time = node->get_parameter("debug.print_get_imu_pose_at_measurement_time").as_bool();
    debug_print_imu_pose_timeline = node->get_parameter("debug.print_imu_pose_timeline").as_bool();
    debug_print_deskew_pointcloud = node->get_parameter("debug.print_deskew_pointcloud").as_bool();
    debug_print_initial_guess_lo = node->get_parameter("debug.print_initial_guess_lo").as_bool();
    debug_print_relative_transform_lo = node->get_parameter("debug.print_relative_transform_lo").as_bool();
    debug_print_num_residuals = node->get_parameter("debug.print_num_residuals").as_bool();
    debug_print_point_plane_residual_preparation = node->get_parameter("debug.print_point_plane_residual_preparation").as_bool();
    debug_print_scan_imu_time_sync = node->get_parameter("debug.print_scan_imu_time_sync").as_bool();
    debug_print_first_point_in_current_scan = node->get_parameter("debug.print_first_point_in_current_scan").as_bool();
    debug_print_cloud_map_size = node->get_parameter("debug.print_cloud_map_size").as_bool();

    RCLCPP_INFO_STREAM(node->get_logger(), "debug.print_imu_forward_propagation_state " << debug_print_imu_forward_propagation_state);
    RCLCPP_INFO_STREAM(node->get_logger(), "debug.print_get_imu_pose_at_measurement_time " << debug_print_get_imu_pose_at_measurement_time);
    RCLCPP_INFO_STREAM(node->get_logger(), "debug.print_imu_pose_timeline " << debug_print_imu_pose_timeline);
    RCLCPP_INFO_STREAM(node->get_logger(), "debug.print_deskew_pointcloud " << debug_print_deskew_pointcloud);
    RCLCPP_INFO_STREAM(node->get_logger(), "debug.print_initial_guess_lo " << debug_print_initial_guess_lo);
    RCLCPP_INFO_STREAM(node->get_logger(), "debug.print_relative_transform_lo " << debug_print_relative_transform_lo);
    RCLCPP_INFO_STREAM(node->get_logger(), "debug.print_num_residuals " << debug_print_num_residuals);
    RCLCPP_INFO_STREAM(node->get_logger(), "debug.print_point_plane_residual_preparation " << debug_print_point_plane_residual_preparation);
    RCLCPP_INFO_STREAM(node->get_logger(), "debug.print_scan_imu_time_sync" << debug_print_scan_imu_time_sync);
    RCLCPP_INFO_STREAM(node->get_logger(), "debug.print_first_point_in_current_scan" << debug_print_first_point_in_current_scan);
    RCLCPP_INFO_STREAM(node->get_logger(), "debug.print_cloud_map_size" << debug_print_cloud_map_size);


    // ---------- for initialization parameter ---------- 
    node->declare_parameter<bool>("init.imu_init_fastlio2", false);

    init_imu_init_fastlio2 = node->get_parameter("init.imu_init_fastlio2").as_bool();

    RCLCPP_INFO_STREAM(node->get_logger(), "init.imu_init_fastlio2 " << init_imu_init_fastlio2);

    
    //  ---------- for imu handling parameter ---------- 
    node->declare_parameter<bool>("imu.isotropic_scale_calibration_to_G", false); // what's done in FastLio2
    node->declare_parameter<bool>("imu.enalbe_init_guess_for_lo", false);

    imu_isotropic_scale_calibration_to_G = node->get_parameter("imu.isotropic_scale_calibration_to_G").as_bool();
    imu_enalbe_init_guess_for_lo = node->get_parameter("imu.enalbe_init_guess_for_lo").as_bool();

    RCLCPP_INFO_STREAM(node->get_logger(), "imu.isotropic_scale_calibration_to_G " << imu_isotropic_scale_calibration_to_G);
    RCLCPP_INFO_STREAM(node->get_logger(), "imu.enalbe_init_guess_for_lo " << imu_enalbe_init_guess_for_lo);

    return true;
}