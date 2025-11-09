#ifndef COMMON_H
#define COMMON_H

// [20250915] Copied from common.h in liliom

#include <chrono>
#include <mutex>
#include <string>
#include <iomanip>
#include <limits>
#include <rclcpp/rclcpp.hpp>

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

#include <tsl/robin_map.h>


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

inline void getGtsamFromEigen(const Eigen::Quaterniond& qIn, const Eigen::Vector3d& tIn, gtsam::Pose3& poseOut)
{
    const gtsam::Rot3 rotIn = gtsam::Rot3::Quaternion(qIn.w(), qIn.x(), qIn.y(), qIn.z());
    const gtsam::Point3 ptIn = gtsam::Point3(tIn);

    poseOut = gtsam::Pose3(rotIn, ptIn);
}


// inline void getGtsamFromEigen(const Eigen::Quaterniond& qIn, const Eigen::Vector3d& tIn, gtsam::Pose3& poseOut)
// {
//     const gtsam::Rot3 rotIn = gtsam::Rot3::Quaternion(qIn.w(), qIn.x(), qIn.y(), qIn.z());
//     const gtsam::Point3 ptIn = gtsam::Point3(tIn);

//     poseOut = gtsam::Pose3(rotIn, ptIn);
// }

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

// template <typename T>
// Eigen::Matrix<T, 3, 3> skewSymmetricMatrix(const Eigen::Vector<T> &q)??
Eigen::Matrix3d getSkewSymMatrix(const Eigen::Vector3d x)
{
    Eigen::Matrix3d x_hat;
    x_hat <<     0, -x(2),  x(1),
              x(2),     0, -x(0),
             -x(1),  x(0),     0;

    return x_hat;
}


// median in-place: v will be permuted
double getMedianInPlace(Eigen::Ref<Eigen::VectorXd> v) {
    const std::size_t n = static_cast<std::size_t>(v.size());
    if (n == 0) return std::numeric_limits<double>::quiet_NaN();
    double* first = v.data();
    double* last  = v.data() + n;
    const std::size_t mid = n / 2;

    std::nth_element(first, first + mid, last);
    double hi = first[mid];
    if (n & 1) return hi;  // odd

    // lower median = max of [first, first+mid)
    double lo = *std::max_element(first, first + mid);
    return 0.5 * (lo + hi);
}

// // exponential map operation based on right multiplicative one (not implemented in Sophus)
// Sophus::SE3<Sophus::Scalar> exp_right(Sophus::Tangent const& a) 
// {
//     using std::cos;
//     using std::sin;
//     Sophus::Vector3<Sophus::Scalar> const omega = a.template tail<3>();

//     Sophus::Scalar theta;
//     Sophus::SO3<Sophus::Scalar> const so3 = Sophus::SO3<Scalar>::expAndTheta(omega, &theta);
//     Sophus::Matrix3<Scalar> const V = Sophus::SO3<Scalar>::leftJacobian(omega, theta);
//     return SE3<Scalar>(so3, V * a.template head<3>());
// }


class TimeLogger 
{
private:
    // enum class State
    // {
    //     RUN, 
    //     STOP
    // };

public:
    struct TimerRecord
    {
        enum class State
        {
            RUN, 
            STOP
        };

        TimerRecord() = default;
        TimerRecord(const std::string &funcName)
        {
            funcName_ = funcName;
        }

        std::string funcName_;
        int lineStart_;
        int lineEnd_;
        std::vector<std::chrono::steady_clock::time_point> time_start_;
        std::vector<std::chrono::steady_clock::time_point> time_end_;
        std::vector<double> time_usage_in_ms_;
        double time_mean_in_ms_;

        State state_;
    };

    TimeLogger() : logger_(rclcpp::get_logger("Default")) {}
    explicit TimeLogger(const rclcpp::Logger& logger) : logger_(logger) {}

    ~TimeLogger() 
    {
        // LOG Everything in the Destructor//
        computeMeanTime();
        // dumpIntoFile("Log");
        dumpIntoFile("/sandbox/ysugano/ros2_ws/src/livo_dev/lo_dev/Log");

        RCLCPP_INFO_STREAM(logger_, __FUNCTION__ << __LINE__);

        // Then clear the file //
        clear();
    }
    void clear()
    {
        records_.clear();
    }

    void setLogger(const rclcpp::Logger& logger)
    {
        logger_=logger;
        // reset();
    }

    // void start(std::string label, std::string function, int line0)
    void start(const std::string &label, const std::string &funcName, int lineStart)
    {
        std::lock_guard<std::mutex> lock(m_); // to make sure it's thread-safe

        auto tStart = std::chrono::steady_clock::now();
        
        if (records_.find(label) == records_.end())     // static std::map<std::string, TimerRecord> records_;
        {
            records_.insert({label, TimerRecord(funcName)});
            records_[label].time_start_.emplace_back(tStart);
            records_[label].lineStart_=lineStart;
            records_[label].state_ = TimerRecord::State::RUN;
        }
        else
        {
            if(records_[label].state_!=TimerRecord::State::STOP)
            {
                RCLCPP_INFO_STREAM(logger_, "[" << label << "]: " << "TimeLogger is still running. Please stop it before restarting.");   
                return;
            }

            records_[label].time_start_.emplace_back(tStart);
            records_[label].state_=TimerRecord::State::RUN;
        }

        // std::mutex m_ is automatically unlocked when it gets out of the function
    }

    // void stop(int line1)
    void stop(const std::string &label, const std::string &funcName, int lineEnd)
    {
        std::lock_guard<std::mutex> lock(m_); // to make sure it's thread-safe

        if (records_.find(label) == records_.end())     // static std::map<std::string, TimerRecord> records_;
        {
            RCLCPP_INFO_STREAM(logger_, "[" << label << "]: " << "TimeLogger is not running. Please start it before logging.");
            // return;
        }
        else
        {
            if(records_[label].state_!=TimerRecord::State::RUN)
            {
                RCLCPP_INFO_STREAM(logger_, "[" << label << "]: " << "TimeLogger is not running. Please start it before logging.");
                return;
            }
        
            auto tStart = records_[label].time_start_.back();
            auto tEnd = std::chrono::steady_clock::now();
            auto tDelta = std::chrono::duration<double, std::milli>(tEnd - tStart).count();

            records_[label].lineEnd_=lineEnd;
            records_[label].time_end_.emplace_back(tEnd);
            records_[label].time_usage_in_ms_.emplace_back(tDelta);
            records_[label].state_=TimerRecord::State::STOP;
        }
    }
    
    void dumpIntoFile(const std::string &filePath)
    {
        std::string fileName;

        for (const auto &iter : records_)
        {
            fileName = filePath + "/" + iter.first + ".txt";
            std::ofstream ofs(fileName, std::ios::out);
            if (!ofs.is_open())
            {
                // std::cout << ANSI_COLOR_RED_BOLD << "Failed to open file: " << file_name << std::endl;
                RCLCPP_INFO_STREAM(logger_, "Failed to open file: " << fileName);
                return;
            }
            else
            {
                // std::cout << ANSI_COLOR_GREEN_BOLD << "Dump Time Records into file: " << file_name << ANSI_COLOR_RESET << std::endl;
                RCLCPP_INFO_STREAM(logger_, "Dump Time Records into file: " << fileName);
            }
            
            ofs.setf(std::ios::fixed);
            ofs << std::setprecision(std::numeric_limits<double>::max_digits10);

            ofs << "Label:" << iter.first << ", Function:'" << iter.second.funcName_ << "' Line:" << iter.second.lineStart_ << "-" << iter.second.lineEnd_ << std::endl;
            ofs << "Average time usage[ms]: " << iter.second.time_mean_in_ms_ << std::endl;
            ofs << "Time usage[ms]      , " << "Time start[ms]              , " << "Time end[ms]" << std::endl;

            for (int i=0; i<iter.second.time_usage_in_ms_.size(); i++)
            {
                ofs << iter.second.time_usage_in_ms_[i] << ", " 
                    << std::chrono::duration<double, std::milli>(iter.second.time_start_[i].time_since_epoch()).count() << ", " 
                    << std::chrono::duration<double, std::milli>(iter.second.time_end_[i].time_since_epoch()).count() << std::endl;
            }

            ofs.close();
        }
    }

    void computeMeanTime()
    {
        for (auto &iter : records_)
        {
            if(iter.second.time_usage_in_ms_.size()==0)
            {
                break;
            }
            
            iter.second.time_mean_in_ms_ = std::accumulate(iter.second.time_usage_in_ms_.begin(), iter.second.time_usage_in_ms_.end(), 0.0) / double(iter.second.time_usage_in_ms_.size());
        }
    }
    

private:
    rclcpp::Logger logger_;
    std::map<std::string, TimerRecord> records_; // <record_label, TimerRecord Object> // should be unordered_map (hash map)?
    std::mutex m_;
};



#endif // COMMON_H