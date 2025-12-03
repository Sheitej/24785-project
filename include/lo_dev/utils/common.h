#ifndef COMMON_H
#define COMMON_H

// [20250915] Copied from common.h in liliom

#include <chrono>
#include <mutex>
#include <string>
#include <iomanip>
#include <limits>
#include <ctime>
#include <sstream>
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
#include <gtsam/navigation/ImuFactor.h>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <tsl/robin_map.h>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <ifopt/variable_set.h>

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

inline void convertQuatToEuler(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
    Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();
    Eigen::Vector3d eulerAngle = rotationMatrix.eulerAngles(0,1,2);
    roll = eulerAngle.x();
    pitch = eulerAngle.y();
    yaw = eulerAngle.z();
}

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

inline void getEigenFromGtsam(const gtsam::Pose3& poseIn, Eigen::Quaterniond& qOut, Eigen::Vector3d& tOut)
{
    const gtsam::Rot3 rotIn = poseIn.rotation();
    const gtsam::Quaternion quatGIn = rotIn.toQuaternion().normalized();
    Eigen::Quaterniond quatEIn(quatGIn.w(),quatGIn.x(),quatGIn.y(),quatGIn.z());
    quatEIn.normalize();

    const gtsam::Point3 tranIn = poseIn.translation();
    Eigen::Vector3d tranEIn(tranIn.x(),tranIn.y(),tranIn.z());

    qOut = quatEIn;
    tOut = tranEIn;
}

inline void getStateVectorFromGtsam(const gtsam::Pose3& poseIn, double& x, double& y, double& z, double& roll, double& pitch, double& yaw)
{
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    getEigenFromGtsam(poseIn, q, t);
    convertQuatToEuler(q, roll, pitch, yaw);

    x = t.x(); 
    y = t.y(); 
    z = t.z(); 
}

inline void getStateVectorFromGtsam(const gtsam::NavState& stateIn, double& x, double& y, double& z, double& roll, double& pitch, double& yaw)
{
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    getEigenFromGtsam(stateIn, q, t);
    convertQuatToEuler(q, roll, pitch, yaw);

    x = t.x(); 
    y = t.y(); 
    z = t.z(); 
}

inline void getStateVectorFromGtsam(const gtsam::NavState& stateIn, 
    double& x, double& y, double& z, double& roll, double& pitch, double& yaw, double& vx, double& vy, double& vz)
{
    getStateVectorFromGtsam(stateIn, x, y, z, roll, pitch, yaw);
    Eigen::Vector3d vel = stateIn.velocity();
    vx = vel.x();
    vy = vel.y();
    vz = vel.z();
}

inline void getGtsamFromEigen(const Eigen::Quaterniond& qIn, const Eigen::Vector3d& tIn, gtsam::NavState& stateOut)
{
    const gtsam::Rot3 rotIn = gtsam::Rot3(qIn);
    const gtsam::Point3 ptIn = gtsam::Point3(tIn);
    const gtsam::Velocity3 velIn = gtsam::Velocity3();

    stateOut = gtsam::NavState(rotIn, ptIn, velIn);
}

inline void getGtsamFromEigen(const Eigen::Quaterniond& qIn, const Eigen::Vector3d& tIn, const Eigen::Vector3d& vIn, gtsam::NavState& stateOut)
{
    const gtsam::Rot3 rotIn = gtsam::Rot3(qIn);
    const gtsam::Point3 ptIn = gtsam::Point3(tIn);
    const gtsam::Velocity3 velIn = gtsam::Velocity3(vIn);

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

std::string getTime()
{
    // using namespace std::chrono;

    auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&tt);

    // std::cout << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << '\n';
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

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
        dumpIntoFile("/sandbox/ysugano/ros2_ws/src/livo_dev/lo_dev/Log/time");

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


class StateLogger 
{
public:
    struct StateRecord
    {
        StateRecord() = default;
        StateRecord(const std::string &recordName)
        {
            recordName_ = recordName;
        }
        std::string recordName_;
        std::map<double, std::pair<Eigen::Vector3d, Eigen::Quaterniond>> stateTimeLine_;
    };

    StateLogger() : logger_(rclcpp::get_logger("Default")) {}
    explicit StateLogger(const rclcpp::Logger& logger) : logger_(logger) {}

    ~StateLogger()
    {
        // dumpIntoFile("Log");
        dumpIntoFile("/sandbox/ysugano/ros2_ws/src/livo_dev/lo_dev/Log");
    }

    void setLogger(const rclcpp::Logger& logger)
    {
        logger_=logger;
        // reset();
    }

    void createRecord(const std::string &recordName)
    {
        StateRecord newRecord(recordName);
        records_.insert(std::make_pair(recordName, newRecord));
    }

    void recordState(const std::string &recordName, double time, const Eigen::Quaterniond& q, const Eigen::Vector3d& p)
    {
        // std::cout << __FUNCTION__ << __LINE__ << std::endl; 
        auto state = std::make_pair(p, q);

        // std::cout << __FUNCTION__ << __LINE__ << std::endl;
        auto it = records_.find(recordName);
        it->second.stateTimeLine_.insert(std::make_pair(time, state));

        // std::cout << __FUNCTION__ << __LINE__ << std::endl;
        if (timeLine_.empty() || std::abs(timeLine_.back()-time) > TIME_EPS) timeLine_.push_back(time); // store the record time 
    }

    void dumpIntoFile(const std::string &filePath)
    {
        std::string fileName;

        fileName = filePath + "/state_estimation_log.txt";
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
            RCLCPP_INFO_STREAM(logger_, "Dump State Records into file: " << fileName);
        }
        
        ofs.setf(std::ios::fixed);
        ofs << std::setprecision(std::numeric_limits<double>::max_digits10);
        
        for (size_t i=0; i<timeLine_.size(); i++)
        {
            auto time = timeLine_[i];
            // write time
            ofs.setf(std::ios::fixed);
            ofs << std::setprecision(std::numeric_limits<double>::max_digits10);
            ofs << "Time: " << time << std::endl;

            for (auto it: records_)
            {
                // auto itRecord = it->second.stateTimeLine_.find(time);
                auto itRecord = it.second.stateTimeLine_.find(time);
                if (itRecord != it.second.stateTimeLine_.end())
                {
                    Eigen::Vector3d pos = itRecord->second.first;
                    double roll, pitch, yaw;
                    convertQuatToEuler(itRecord->second.second, roll, pitch, yaw);

                    auto offset = it.first.length();

                    // write down the state name
                    ofs << std::setw(10) << "[" << it.first << "]:" ;
                    ofs << std::setw(15-offset) << std::setprecision(6) << pos.x() << ", " << pos.y() << ", " << pos.z() << ", " << roll << ", " << pitch << ", " << yaw << std::endl;
                }
                // add an empty line for readability
            }
            ofs << std::endl;
        }

        ofs.close();

    }

private:
    rclcpp::Logger logger_;
    std::map<std::string, StateRecord> records_;
    std::vector<double> timeLine_;
};


class ImuRawLogger 
{
public:
    struct ImuRawRecord
    {
        ImuRawRecord() = default;
        ImuRawRecord(double integrationBeginningTime)
        {
            integrationBeginningTime_ = integrationBeginningTime;
        }
        // std::string recordName_;
        // std::map<double, std::pair<Eigen::Vector3d, Eigen::Quaterniond>> stateTimeLine_;
        double integrationBeginningTime_;
        std::vector<double> tPrev_;
        std::vector<double> tNext_;
        std::vector<double> dt_;
        std::vector<gtsam::Vector3> vecAcc_;
        std::vector<gtsam::Vector3> vecGyr_;
    };

    ImuRawLogger() : logger_(rclcpp::get_logger("Default")) {}
    explicit ImuRawLogger(const rclcpp::Logger& logger) : logger_(logger) {}

    ~ImuRawLogger()
    {
        // dumpIntoFile("Log");
        dumpIntoFile("/sandbox/ysugano/ros2_ws/src/livo_dev/lo_dev/Log/imu");
    }

    void setLogger(const rclcpp::Logger& logger)
    {
        logger_=logger;
        // reset();
    }

    // void createRecord(const std::string &recordName)
    // {
    //     StateRecord newRecord(recordName);
    //     records_.insert(std::make_pair(recordName, newRecord));
    // }

    void recordImuRaw(double time, double tPrev, double tNext, double dt, gtsam::Vector3 vecAcc, gtsam::Vector3 vecGyr)
    {
        auto it = records_.find(time);
        if (it == records_.end())
        {
            ImuRawRecord data(time);
            data.tPrev_.push_back(tPrev);
            data.tNext_.push_back(tNext);
            data.dt_.push_back(dt);
            data.vecAcc_.push_back(vecAcc);
            data.vecGyr_.push_back(vecGyr);
            records_.insert(std::make_pair(time,data));
        }
        else
        {
            it->second.tPrev_.push_back(tPrev);
            it->second.tNext_.push_back(tNext);
            it->second.dt_.push_back(dt);
            it->second.vecAcc_.push_back(vecAcc);
            it->second.vecGyr_.push_back(vecGyr);
        }

        // std::cout << __FUNCTION__ << __LINE__ << std::endl;
        if (timeLine_.empty() || std::abs(timeLine_.back()-time) > TIME_EPS) timeLine_.push_back(time); // store the record time 
    }

    void dumpIntoFile(const std::string &filePath)
    {
        std::string fileName;

        fileName = filePath + "/imu_raw_log.txt";
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
            RCLCPP_INFO_STREAM(logger_, "Dump State Records into file: " << fileName);
        }
        
        ofs.setf(std::ios::fixed);
        ofs << std::setprecision(std::numeric_limits<double>::max_digits10);
        
        for (size_t t=0; t<timeLine_.size(); t++)
        {
            auto time = timeLine_[t];
            // write time
            ofs.setf(std::ios::fixed);
            // ofs << std::setprecision(6) << "Integration Beginning Time: " << time << std::endl;
            ofs << std::setprecision(6) << "Time: " << time << std::endl;
            ofs << std::setw(5) << "tPrev,             tNext,             dt,       vecAcc.x, vecAcc.y, vecAcc.z, vecGyr.x, vecGyr.y, vecGyr.z" << std::endl;
            // ofs << std::setprecision(std::numeric_limits<double>::max_digits10);

            auto it = records_.find(time);
            if (it != records_.end())
            {
                ImuRawRecord& data = it->second;

                for (size_t i=0; i<data.tPrev_.size(); ++i)
                {
                    // write down the state name
                    // ofs << std::setw(10) << "[" << it.first << "]:" ;
                    // ofs << std::setw(15-offset) << std::setprecision(6) 
                    ofs << std::setw(5) << std::setprecision(6) 
                        << data.tPrev_[i] << ", " << data.tNext_[i] << ", " << data.dt_[i] << ", " 
                        << data.vecAcc_[i].x() << ", " << data.vecAcc_[i].y() << ", " << data.vecAcc_[i].z() << ", " 
                        << data.vecGyr_[i].x() << ", " << data.vecGyr_[i].y() << ", " << data.vecGyr_[i].z() << ", " << std::endl;
                }
            }

            // for (auto it: records_)
            // {
            //     auto itRecord = it.second.stateTimeLine_.find(time);
            //     if (itRecord != it.second.stateTimeLine_.end())
            //     {
            //         Eigen::Vector3d pos = itRecord->second.first;
            //         double roll, pitch, yaw;
            //         convertQuatToEuler(itRecord->second.second, roll, pitch, yaw);

            //         auto offset = it.first.length();

            //         // write down the state name
            //         ofs << std::setw(10) << "[" << it.first << "]:" ;
            //         ofs << std::setw(15-offset) << std::setprecision(6) << pos.x() << ", " << pos.y() << ", " << pos.z() << ", " << roll << ", " << pitch << ", " << yaw << std::endl;
            //     }
            //     // add an empty line for readability
            // }
            ofs << std::endl;
        }

        ofs.close();

    }

private:
    rclcpp::Logger logger_;
    std::map<double, ImuRawRecord> records_;
    std::vector<double> timeLine_;
    // std::vector<double> ;
};


class InequalityConstraintsLogger 
{
public:
    struct InequalityConstraintsRecord
    {
        InequalityConstraintsRecord() = default;
        double time_;
        bool isActivated_;
    };

    InequalityConstraintsLogger () : logger_(rclcpp::get_logger("Default")) {}
    explicit InequalityConstraintsLogger (const rclcpp::Logger& logger) : logger_(logger) {}

    ~InequalityConstraintsLogger()
    {
        dumpIntoFile("/sandbox/ysugano/ros2_ws/src/livo_dev/lo_dev/Log/InequalityConstraint");
    }

    void setLogger(const rclcpp::Logger& logger)
    {
        logger_=logger;
        // reset();
    }

    void recordInequalityConstraints(
        double time, 
        bool isActivated)
    {
        InequalityConstraintsRecord record;
        record.time_ = time;
        record.isActivated_ = isActivated;

        records_.push_back(record);
    }

    void dumpIntoFile(const std::string &filePath)
    {
        std::string fileName;

        fileName = filePath + "/inequality_constraints_log.txt";
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
            RCLCPP_INFO_STREAM(logger_, "Dump State Records into file: " << fileName);
        }
        
        ofs.setf(std::ios::fixed);
        ofs << std::setprecision(std::numeric_limits<double>::max_digits10);

        ofs.setf(std::ios::fixed);
        ofs << "Time, the constraint is activated" << std::endl;

        for (size_t i; i < records_.size(); i++){
            ofs << std::setprecision(6) << records_[i].time_ << ", " << records_[i].isActivated_ << std::endl;
        }

        ofs.close();

    }

private:
    rclcpp::Logger logger_;
    std::vector<InequalityConstraintsRecord> records_;
};


class TwistLogger 
{
public:
    struct TwistRecord
    {
        TwistRecord() = default;
        double time_;
        double tx_;
        double ty_;
        double tz_;
        double rx_;
        double ry_;
        double rz_;
    };

    TwistLogger () : logger_(rclcpp::get_logger("Default")) {}
    explicit TwistLogger (const rclcpp::Logger& logger) : logger_(logger) {}

    ~TwistLogger()
    {
        dumpIntoFile("/sandbox/ysugano/ros2_ws/src/livo_dev/lo_dev/Log/Twist_optimized");
    }

    void setLogger(const rclcpp::Logger& logger)
    {
        logger_=logger;
        // reset();
    }

    void recordTwist(
        double time, 
        double rx, 
        double ry, 
        double rz,
        double tx, 
        double ty, 
        double tz)
    {
        TwistRecord record;
        record.time_ = time;
        record.tx_ = tx;
        record.ty_ = ty;
        record.tz_ = tz;
        record.rx_ = rx;
        record.ry_ = ry;
        record.rz_ = rz;

        records_.push_back(record);
    }

    void dumpIntoFile(const std::string &filePath)
    {
        std::string fileName;

        fileName = filePath + "/twist_optimized_log.txt";
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
            RCLCPP_INFO_STREAM(logger_, "Dump State Records into file: " << fileName);
        }
        
        ofs.setf(std::ios::fixed);
        ofs << std::setprecision(std::numeric_limits<double>::max_digits10);

        ofs.setf(std::ios::fixed);
        ofs << "Time, tx[m], ty[m], tz[m], rx[deg], ry[deg], rz[deg]: (optimized twist)";

        for (size_t i; i < records_.size(); i++){
            ofs << std::setprecision(6) << records_[i].time_ << ", " << records_[i].tx_ << ", " << records_[i].ty_ << ", " << records_[i].tz_ 
                << ", " << records_[i].rx_ << ", " << records_[i].ry_ << ", " << records_[i].rz_ << std::endl;
        }

        ofs.close();

    }

private:
    rclcpp::Logger logger_;
    std::vector<TwistRecord> records_;
};


// to add:
//  - latest state covariance
//  - ekf propagated covariance
//  

class FactorGraphLogger 
{
public:
    struct Config
    {
        double lidarCorrectionNoise_;
        double imuAccNoise_;
        double imuGyrNoise_;
        double imuAccNoise_BiasRandomWalk_;
        double imuGyrNoise_BiasRandomWalk_;
        double gravityNorm_;
        double fixed_lag_;
        bool useLOPriorFactor_;
        std::string fgStateInitGuessSource_;
    };

    struct FactorGraphRecord
    {
        FactorGraphRecord(){}

        double keyframeTimestamp_;
        double timeCurrScanBeg_;
        double timeCurrScanEnd_;
        double timeImuBeg_;
        double timeImuEnd_;
        int keyFrameId_;
        gtsam::Pose3 lidarBetweenFactor_;
        gtsam::Pose3 lidarPriorFactor_;
        Eigen::MatrixXd lidarOdomCovariance_;
        gtsam::Pose3 imuBetweenFactor_;
        Eigen::MatrixXd imuPreintCovariance_;
        Eigen::MatrixXd imuBiasCovariance_;
        gtsam::NavState statePrevKf_;
        gtsam::imuBias::ConstantBias biasPrevKf_;
        gtsam::NavState stateCurrKf_initGuess_lo_;
        gtsam::imuBias::ConstantBias biasCurrKf_initGuess_lo_;
        gtsam::NavState stateCurrKf_initGuess_imu_;
        gtsam::imuBias::ConstantBias biasCurrKf_initGuess_imu_;
        Eigen::MatrixXd stateCovPredCurrKf_imu_;   // New
        gtsam::NavState optimizedStateCurrKf_;
        gtsam::imuBias::ConstantBias optimizedBiasCurrKf_;
        Eigen::MatrixXd optimizedStateCovCurrKf_;   // New
        int numKeyframes_;
        bool isFactorGraphOn_;
    };

    FactorGraphLogger() : logger_(rclcpp::get_logger("Default")) {}
    explicit FactorGraphLogger(const rclcpp::Logger& logger) : logger_(logger) {}

    ~FactorGraphLogger()
    {
        std::cout << __FUNCTION__ << __LINE__ << std::endl;
        // dumpIntoFile("Log");
        dumpIntoFile("/sandbox/ysugano/ros2_ws/src/livo_dev/lo_dev/Log/fg");
    }

    void setLogger(const rclcpp::Logger& logger)
    {
        logger_=logger;
        // reset();
    }

    void setCfgParams(
        const float& lidarCorrectionNoise,
        const float& imuAccNoise,
        const float& imuGyrNoise,
        const float& imuAccNoise_BiasRandomWalk,
        const float& imuGyrNoise_BiasRandomWalk,
        const float& gravityNorm,
        const double& fixed_lag,
        const bool& useLOPriorFactor,
        const std::string& fgStateInitGuessSource
    )
    {
        executionTime_ = getTime();
        cfgParams_.lidarCorrectionNoise_ = lidarCorrectionNoise;
        cfgParams_.imuAccNoise_ = imuAccNoise;
        cfgParams_.imuGyrNoise_ = imuGyrNoise;
        cfgParams_.imuAccNoise_BiasRandomWalk_ = imuAccNoise_BiasRandomWalk;
        cfgParams_.imuGyrNoise_BiasRandomWalk_ = imuGyrNoise_BiasRandomWalk;
        cfgParams_.gravityNorm_ = gravityNorm;
        cfgParams_.fixed_lag_ = fixed_lag;
        cfgParams_.useLOPriorFactor_ = useLOPriorFactor;
        cfgParams_.fgStateInitGuessSource_ = fgStateInitGuessSource;
    }

    void recordFGState(
        const double& keyframeTimestamp,
        const double& timeCurrScanBeg,
        const double& timeCurrScanEnd,
        const double& timeImuBeg,
        const double& timeImuEnd,
        const int& keyFrameId,
        const gtsam::Pose3& lidarBetweenFactor,
        const gtsam::Pose3& lidarPriorFactor,
        const Eigen::MatrixXd& lidarOdomCovariance,
        const gtsam::Pose3& imuBetweenFactor,
        const Eigen::MatrixXd& imuPreintCovariance,
        const Eigen::MatrixXd& imuBiasCovariance,
        const gtsam::NavState& statePrevKf,
        const gtsam::imuBias::ConstantBias& biasPrevKf,
        const gtsam::NavState& stateCurrKf_initGuess_lo,
        const gtsam::imuBias::ConstantBias& biasCurrKf_initGuess_lo,
        const gtsam::NavState& stateCurrKf_initGuess_imu,
        const gtsam::imuBias::ConstantBias& biasCurrKf_initGuess_imu,
        const Eigen::MatrixXd& stateCovPredCurrKf_imu,   // New
        const gtsam::NavState& optimizedStateCurrKf,
        const gtsam::imuBias::ConstantBias& optimizedBiasCurrKf,
        const Eigen::MatrixXd& optimizedStateCovCurrKf,   // New
        const int& numKeyframes,
        const bool& isFactorGraphOn
    )
    {
        FactorGraphRecord record;

        record.keyframeTimestamp_ = keyframeTimestamp;
        record.timeCurrScanBeg_ = timeCurrScanBeg;
        record.timeCurrScanEnd_ = timeCurrScanEnd;
        record.timeImuBeg_ = timeImuBeg;
        record.timeImuEnd_ = timeImuEnd;
        record.keyFrameId_ = keyFrameId;

        record.lidarBetweenFactor_ = lidarBetweenFactor;
        record.lidarPriorFactor_ = lidarPriorFactor;
        record.lidarOdomCovariance_ = lidarOdomCovariance;
        record.imuBetweenFactor_ = imuBetweenFactor;
        record.imuPreintCovariance_ = imuPreintCovariance,
        record.imuBiasCovariance_ = imuBiasCovariance,
        record.statePrevKf_ = statePrevKf;
        record.biasPrevKf_ = biasPrevKf;

        record.stateCurrKf_initGuess_lo_ = stateCurrKf_initGuess_lo;
        record.biasCurrKf_initGuess_lo_ = biasCurrKf_initGuess_lo;
        record.stateCurrKf_initGuess_imu_ = stateCurrKf_initGuess_imu;
        record.biasCurrKf_initGuess_imu_ = biasCurrKf_initGuess_imu;
        record.stateCovPredCurrKf_imu_ = stateCovPredCurrKf_imu,
        record.optimizedStateCurrKf_ = optimizedStateCurrKf;
        record.optimizedBiasCurrKf_ = optimizedBiasCurrKf;
        record.optimizedStateCovCurrKf_ = optimizedStateCovCurrKf;
        record.numKeyframes_ = numKeyframes;
        record.isFactorGraphOn_ = isFactorGraphOn;

        records_.emplace_back(record);
    }

    void dumpIntoFile(const std::string &filePath)
    {
        std::string fileName;

        fileName = filePath + "/[FG]: log_" + executionTime_ + ".txt";
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
            RCLCPP_INFO_STREAM(logger_, "Dump State Records into file: " << fileName);
        }
        
        ofs << "========== Executaion Time: " << executionTime_ << std::endl;
        ofs << "========== Config Parameters ==========" << std::endl;
        ofs << "                lidar_correction_noise: " << cfgParams_.lidarCorrectionNoise_ << std::endl;
        ofs << "                                 acc_n: " << cfgParams_.imuAccNoise_ << std::endl;
        ofs << "                                 gyr_n: " << cfgParams_.imuGyrNoise_ << std::endl;
        ofs << "                                 acc_w: " << cfgParams_.imuAccNoise_BiasRandomWalk_ << std::endl;
        ofs << "                                 gyr_w: " << cfgParams_.imuGyrNoise_BiasRandomWalk_ << std::endl;
        ofs << "                                 acc_w: " << cfgParams_.imuAccNoise_BiasRandomWalk_ << std::endl;
        ofs << "                                g_norm: " << cfgParams_.gravityNorm_ << std::endl;
        ofs << "                             fixed_lag: " << cfgParams_.fixed_lag_ << std::endl;
        ofs << " use_lo_prior_factor_wo_between_factor: " << cfgParams_.useLOPriorFactor_ << std::endl;
        ofs << "        factor_graph_init_guess_source: " << cfgParams_.fgStateInitGuessSource_ << std::endl;
        ofs << std::endl;
        ofs << std::endl;
        
        // write all the records
        for (auto rec: records_){
            ofs << "====================" << std::endl;
            ofs << " Keyframe timestamp          : " << std::fixed << rec.keyframeTimestamp_ << std::endl;
            ofs << " Keyframe id                 : " << rec.keyFrameId_ << std::endl;
            ofs << " Num. of keyframes in window : " << rec.numKeyframes_ << std::endl;
            ofs << " Factor Graph On             : " << rec.isFactorGraphOn_ << std::endl;
            ofs << " Lidar scan beginning time   : " << std::setprecision(std::numeric_limits<double>::max_digits10) << rec.timeCurrScanBeg_ << std::endl;
            ofs << " Lidar scan ending time      : " << std::setprecision(std::numeric_limits<double>::max_digits10) << rec.timeCurrScanEnd_ << std::endl;
            ofs << " Imu beginning time          : " << std::setprecision(std::numeric_limits<double>::max_digits10) << rec.timeImuBeg_ << std::endl;
            ofs << " Imu ending time             : " << std::setprecision(std::numeric_limits<double>::max_digits10) << rec.timeImuEnd_ << std::endl;

            double x, y, z, roll, pitch, yaw, vx, vy, vz;
            x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, vx=0.0, vy=0.0, vz=0.0;
            std::string label;

            ofs << "  ----- Factors (*: used for optimization)-----  " << std::endl;
            writeOdometryHeaderToFile(ofs);
            if (!rec.lidarBetweenFactor_.equals(gtsam::Pose3())){
                getStateVectorFromGtsam(rec.lidarBetweenFactor_, x, y, z, roll, pitch, yaw);
                if(!cfgParams_.useLOPriorFactor_){label="Lidar Odom Between Factor*";} else {label="Lidar Odom Between Factor";}
                writeOdometryToFile(ofs, label, x, y, z, roll, pitch, yaw);
            }
            if (!rec.lidarPriorFactor_.equals(gtsam::Pose3())){
                getStateVectorFromGtsam(rec.lidarPriorFactor_, x, y, z, roll, pitch, yaw);
                if(cfgParams_.useLOPriorFactor_){
                    label = "Lidar Odom Prior Factor(abs.pose)*";
                }else{
                    label="Lidar Odom Prior Factor(abs.pose)";
                }
                writeOdometryToFile(ofs, label, x, y, z, roll, pitch, yaw);
            }
            if (!rec.imuBetweenFactor_.equals(gtsam::Pose3())){
                getStateVectorFromGtsam(rec.imuBetweenFactor_, x, y, z, roll, pitch, yaw);
                if(x>0.00001){label="Imu Between Factor*";} else {label="Imu Between Factor";}
                writeOdometryToFile(ofs, label, x, y, z, roll, pitch, yaw);
            }


            Eigen::VectorXd bias = Eigen::VectorXd::Zero(6);
            ofs << "  ----- States (*: used for optimization)-----  " << std::endl;
            writeStateHeaderToFile(ofs);

            getStateVectorFromGtsam(rec.statePrevKf_, x, y, z, roll, pitch, yaw, vx, vy, vz);
            if (!rec.biasPrevKf_.equals(gtsam::imuBias::ConstantBias())) bias = rec.biasPrevKf_.vector();
            writeStateToFile(ofs, "Previous Kf State", x, y, z, roll, pitch, yaw, vx, vy, vz, bias);

            getStateVectorFromGtsam(rec.stateCurrKf_initGuess_lo_, x, y, z, roll, pitch, yaw, vx, vy, vz);
            if (!rec.biasCurrKf_initGuess_lo_.equals(gtsam::imuBias::ConstantBias())) bias = rec.biasCurrKf_initGuess_lo_.vector();
            if(cfgParams_.fgStateInitGuessSource_ == "lidar") {label="Init Guess Current Kf State(Lidar)*";} else {label="Init Guess Current Kf State(Lidar)";}
            writeStateToFile(ofs, label, x, y, z, roll, pitch, yaw, vx, vy, vz, bias);

            getStateVectorFromGtsam(rec.stateCurrKf_initGuess_imu_, x, y, z, roll, pitch, yaw, vx, vy, vz);
            if (!rec.biasCurrKf_initGuess_imu_.equals(gtsam::imuBias::ConstantBias())) bias = rec.biasCurrKf_initGuess_imu_.vector();
            if(cfgParams_.fgStateInitGuessSource_ == "imu") {label="Init Guess Current Kf State(Imu)*";} else {label="Init Guess Current Kf State(Imu)";}
            writeStateToFile(ofs, label, x, y, z, roll, pitch, yaw, vx, vy, vz, bias);

            getStateVectorFromGtsam(rec.optimizedStateCurrKf_, x, y, z, roll, pitch, yaw, vx, vy, vz);
            if (!rec.optimizedBiasCurrKf_.equals(gtsam::imuBias::ConstantBias())) bias = rec.optimizedBiasCurrKf_.vector();
            writeStateToFile(ofs, "Optimized Current Kf State", x, y, z, roll, pitch, yaw, vx, vy, vz, bias);

            ofs << "  ----- Covariance (state)-----  " << std::endl;
            if (!rec.stateCovPredCurrKf_imu_.isApprox(Eigen::MatrixXd::Zero(9, 9))){
            // if (!rec.stateCovPredCurrKf_imu_ == (Eigen::MatrixXd::Zero(9, 9))){
                writeMatrixToFile(ofs, "State Covariance Imu Pred.", rec.stateCovPredCurrKf_imu_);
            }
            if (!rec.optimizedStateCovCurrKf_.isApprox(Eigen::MatrixXd::Zero(9, 9))){
            // if (!rec.optimizedStateCovCurrKf_ == (Eigen::MatrixXd::Zero(9))){
                writeMatrixToFile(ofs, "Optimized State Covariance", rec.optimizedStateCovCurrKf_);
            }
            
            ofs << "  ----- Covariance (measurement)-----  " << std::endl;
            if (!rec.lidarBetweenFactor_.equals(gtsam::Pose3()) || !rec.lidarPriorFactor_.equals(gtsam::Pose3())){
                writeMatrixToFile(ofs, "Lidar Odom Covariance", rec.lidarOdomCovariance_);
            }
            if (!rec.imuBetweenFactor_.equals(gtsam::Pose3())){
                writeMatrixToFile(ofs, "IMU Preintegration Covariance", rec.imuPreintCovariance_);
                writeMatrixToFile(ofs, "IMU Bias Covariance", rec.imuBiasCovariance_);
            }
            
            ofs << "  ----- Covariance^(-1) (measurement)-----  " << std::endl;
            if (!rec.lidarBetweenFactor_.equals(gtsam::Pose3()) || !rec.lidarPriorFactor_.equals(gtsam::Pose3())){
                writeMatrixToFile(ofs, "Lidar Odom Covariance^(-1)", rec.lidarOdomCovariance_.inverse());
            }
            if (!rec.imuBetweenFactor_.equals(gtsam::Pose3())){
                writeMatrixToFile(ofs, "IMU Preintegration Covariance^(-1)", rec.imuPreintCovariance_.inverse());
                writeMatrixToFile(ofs, "IMU Bias Covariance^(-1)", rec.imuBiasCovariance_.inverse());
            }
            
            ofs << std::endl;
            ofs << std::endl;
        }
    }

    void writeStateHeaderToFile(std::ofstream& ofs)
    {
        ofs << "        " << std::left << std::setw(36) << "Header " << " : "
            << std::right << std::fixed << std::setprecision(6)
            << std::setw(12) << "x"   << ", "
            << std::setw(12) << "y"   << ", "
            << std::setw(12) << "z"   << ", "
            << std::setw(12) << "roll"  << ", "
            << std::setw(12) << "pitch" << ", "
            << std::setw(12) << "yaw"   << ", "
            << std::setw(12) << "vx"    << ", "
            << std::setw(12) << "vy"    << ", "
            << std::setw(12) << "vz"    << ", "
            << std::setw(12) << "bax" << ", "
            << std::setw(12) << "bay" << ", "
            << std::setw(12) << "baz" << ", "
            << std::setw(12) << "bgx" << ", "
            << std::setw(12) << "bgy" << ", "
            << std::setw(12) << "bgz"
            << '\n';
    }

    void writeStateToFile(std::ofstream& ofs, const std::string& label,
                        double x, double y, double z,
                        double roll, double pitch, double yaw,
                        double vx, double vy, double vz,
                        const Eigen::VectorXd& bias)
    {
        ofs << "        " << std::left << std::setw(36) << label << " : "
            << std::right << std::fixed << std::setprecision(6)
            << std::setw(12) << x   << ", "
            << std::setw(12) << y   << ", "
            << std::setw(12) << z   << ", "
            << std::setw(12) << roll  << ", "
            << std::setw(12) << pitch << ", "
            << std::setw(12) << yaw   << ", "
            << std::setw(12) << vx    << ", "
            << std::setw(12) << vy    << ", "
            << std::setw(12) << vz    << ", "
            << std::setw(12) << bias[0] << ", "
            << std::setw(12) << bias[1] << ", "
            << std::setw(12) << bias[2] << ", "
            << std::setw(12) << bias[3] << ", "
            << std::setw(12) << bias[4] << ", "
            << std::setw(12) << bias[5]
            << '\n';
    }

    void writeOdometryHeaderToFile(std::ofstream& ofs)
    {
        ofs << "        " << std::left << std::setw(36) << "Header " << " : "
            << std::right << std::fixed << std::setprecision(6)
            << std::setw(12) << "dx"   << ", "
            << std::setw(12) << "dy"   << ", "
            << std::setw(12) << "dz"   << ", "
            << std::setw(12) << "droll"  << ", "
            << std::setw(12) << "dpitch" << ", "
            << std::setw(12) << "dyaw" 
            << '\n';
    }

    void writeOdometryToFile(std::ofstream& ofs, const std::string& label,
                        double x, double y, double z,
                        double roll, double pitch, double yaw)
    {
        ofs << "        " << std::left << std::setw(36) << label << " : "
            << std::right << std::fixed << std::setprecision(6)
            << std::setw(12) << x   << ", "
            << std::setw(12) << y   << ", "
            << std::setw(12) << z   << ", "
            << std::setw(12) << roll  << ", "
            << std::setw(12) << pitch << ", "
            << std::setw(12) << yaw 
            << '\n';
    }

    template <typename Derived>
    void writeMatrixToFile(std::ostream& ofs,
                    const std::string& title,
                    const Eigen::MatrixBase<Derived>& M)
    {
        std::ios oldState(nullptr);
        oldState.copyfmt(ofs);

        ofs << "  ----- " << title
            << " (" << M.rows() << " x " << M.cols() << ") -----\n";

        // ofs << std::fixed << std::setprecision(10);
        ofs << std::scientific << std::setprecision(3);

        for (int r = 0; r < M.rows(); ++r) {
            // ofs << "        row " << std::setw(2) << r << " : ";
            for (int c = 0; c < M.cols(); ++c) {
                ofs << std::setw(12) << M(r, c);
                if (c + 1 < M.cols())
                    ofs << ", ";
            }
            ofs << '\n';
        }
        ofs.copyfmt(oldState);
        // ofs << '\n';
    }

private:
    rclcpp::Logger logger_;
    std::vector<FactorGraphRecord> records_;

    std::string executionTime_;
    Config cfgParams_;

};


void computeWeightsFromResiduals(const Eigen::VectorXd r, Eigen::VectorXd& w, double s, double c)
{
    Eigen::VectorXd r_copy = r;
    double r_med = getMedianInPlace(r_copy);      // X_bar = median(X)
    Eigen::VectorXd absdev = (r.array() - r_med).abs();
    double absdev_med = getMedianInPlace(absdev);    // MAD = median(|Xi - X_bar|)
    s = 1.4826 * std::max(absdev_med, 1e-12);   // 1.4826 makes MAD ~ σ for Gaussian. Also 1e-12 caring numerical stability

    const double eps = 1e-12;
    w.resize(r.size());  // r should be resized with the decleration

    for (int i = 0; i < r.size(); ++i) 
    {
        double ri = r(i);
        double u  = ri / std::max(s, eps);
        double wi = 1.0;
        c = 1.345; // gives 95% efficiency for Gaussian noise
        double a = std::abs(u);
        wi = (a <= c) ? 1.0 : (c / (a + eps));
        w(i) = wi;
    }
}

// Original objective function in point-to-plane icp:
//  sigma || ( wn @ wp + d) ||2_weighted 
//  sigma || ( wn @ (R @ bp + t) + d) ||2_weighted 
//   we need:
//      wn: normals
//      bp: points
//      d: normal offsetts

// Now LBFGSpp does not work well because the jacobian is correct only locally at the linearization point
//  , while LBFGSpp requires a globally correct gradient for the line search
class PointToPlaneCostForLBFGSpp
{
private:
    Sophus::SE3d T0_;
    int numResidual_ = 0;
    std::vector<Eigen::Vector3d> *pointScanCurrInBForResidual_ = nullptr;
    std::vector<Eigen::Vector3d> *planeNormalForResidual_ = nullptr;
    std::vector<double> *planeDistFromOriginForResidual_ = nullptr;

public:
    PointToPlaneCostForLBFGSpp(){}
    PointToPlaneCostForLBFGSpp(
        Sophus::SE3d T0,
        int numResidual,
        std::vector<Eigen::Vector3d>& pointScanCurrInBForResidual,
        std::vector<Eigen::Vector3d>& planeNormalForResidual,
        std::vector<double>& planeDistFromOriginForResidual
    )
    {
        T0_ = T0;
        numResidual_ = numResidual;
        // pointScanCurrInBForResidual_ = *pointScanCurrInBForResidual;
        // planeNormalForResidual_ = *planeNormalForResidual;
        // planeDistFromOriginForResidual_ = *planeDistFromOriginForResidual;
        pointScanCurrInBForResidual_ = &pointScanCurrInBForResidual;
        planeNormalForResidual_ = &planeNormalForResidual;
        planeDistFromOriginForResidual_ = &planeDistFromOriginForResidual;
    }

    // Analytical Jacobian version
    double operator()(const Eigen::VectorXd& x, Eigen::VectorXd& grad)
    {
        double fx = 0.0; // objective function value
        grad.setZero(x.size());

        // std::cout << __FUNCTION__ << __LINE__ << std::endl;

        // Eigen::VectorXd state = x;

        // exponential map from x: state vector to R in SO(3) and t in R^3            
        // Map state vector x (6DoF) to a transformation matrix via exp map in Sophus
        // reorder the state vector to adjust to Sophus twist 
        Eigen::Vector<double, 6> a;
        a.tail<3>() = x.head<3>(); // axis-angle vector (alpha, beta, gamma)
        a.head<3>() = x.tail<3>(); // translation vector (x,y,z)
        // a.tail<3>() = state.head<3>(); // axis-angle vector (alpha, beta, gamma)
        // a.head<3>() = state.tail<3>(); // translation vector (x,y,z)
        // const Sophus::SE3d Twb = Sophus::SE3d::exp(a);
        // const Eigen::Matrix3d Rwb = Twb.rotationMatrix();
        // const Eigen::Vector3d twb = Twb.translation();
        const Sophus::SE3d exp_x = Sophus::SE3d::exp(a);
        const Sophus::SE3d Twb = T0_ * exp_x;
        const Eigen::Matrix3d Rwb = Twb.rotationMatrix();
        const Eigen::Vector3d twb = Twb.translation();

        // std::cout << __FUNCTION__ << __LINE__ << std::endl;
        
        // std::vector res_unweighted;
        Eigen::VectorXd res(numResidual_);
        Eigen::VectorXd resSqrd(numResidual_);

        // compute residual based on the current pose
        double r2 = 0.0;
        for (size_t i=0; i < numResidual_; ++i)
        {
            const Eigen::Vector3d wp = Rwb * pointScanCurrInBForResidual_->at(i) + twb;
            const double r = planeNormalForResidual_->at(i).transpose() * wp + planeDistFromOriginForResidual_->at(i);
            r2 = r * r;

            // res_unweighted.push_back(res_unweighted);
            res(i) = r;
            resSqrd(i) = r2;
        }

        // std::cout << __FUNCTION__ << __LINE__ << std::endl;
        
        // weight them based on robust kernel
        Eigen::VectorXd w;  
        double s = 0.0; // default scaling just as an option manually tune it
        double c = 0.0; // default range just as an option manually tune it
        // computeWeightsFromResiduals(r2, w, s, c);
        // computeWeightsFromResiduals(res_unweighted, w, s, c);
        // computeWeightsFromResiduals(resSqrd, w, s, c);
        computeWeightsFromResiduals(res, w, s, c);
        // w = Eigen::VectorXd::Ones(res.size());
        // fx = res_unweighted.cwiseProduct(w);
        // fx = w.transpose() * res_unweighted;
        fx = w.transpose() * resSqrd;

        // std::cout << __FUNCTION__ << __LINE__ << std::endl;
        
        // compute gradient and overwrite grad
        for (size_t i=0; i < numResidual_; ++i)
        {
            // Eigen::Matrix3d bp_hat = getSkewSymMatrix(bp);
            Eigen::Matrix3d bp_hat = getSkewSymMatrix(pointScanCurrInBForResidual_->at(i));
            // Eigen::RowVectorXd jacobian(6);

            // jacobian.head<3>() = -planeNormalForResidual->at(i).transpose() * Rwb * bp_hat;
            // jacobian.tail<3>() = planeNormalForResidual->at(i).transpose() * Rwb;
            // grad.head<3>() += w(i) * -planeNormalForResidual_->at(i).transpose() * Rwb * bp_hat;
            // grad.tail<3>() += w(i) * planeNormalForResidual_->at(i).transpose() * Rwb;

            // grad.head<3>() += w(i) * (-planeNormalForResidual_->at(i).transpose() * Rwb * bp_hat).transpose();
            // grad.tail<3>() += w(i) * (planeNormalForResidual_->at(i).transpose() * Rwb).transpose();

            // nabla_f = sigma (2 * wi * ri * Ji.T)

            grad.head<3>() += 2.0 * w(i) * res(i) * (-planeNormalForResidual_->at(i).transpose() * Rwb * bp_hat).transpose();
            grad.tail<3>() += 2.0 * w(i) * res(i) * (planeNormalForResidual_->at(i).transpose() * Rwb).transpose();

            // std::cout << "planeNormalForResidual_->at(i).transpose(): " << planeNormalForResidual_->at(i).transpose() << std::endl;
            // std::cout << "Rwb: " << Rwb << std::endl;
            // std::cout << "bp_hat: " << bp_hat << std::endl;
            // std::cout << "grad.head<3>(): " << grad.head<3>() << std::endl;
            // std::cout << "grad.tail<3>(): " << grad.tail<3>() << std::endl;
        }
        
        std::cout << "x: " << x << std::endl;
        std::cout << "grad: " << grad << std::endl;
        std::cout << "fx: " << fx << std::endl;

        // std::cout << __FUNCTION__ << __LINE__ << std::endl;

        // // ---- DEBUG: finite-difference gradient check at current x ----
        // {
        //     const double eps = 1e-6;
        //     Eigen::VectorXd grad_num(grad.size());
        //     Eigen::VectorXd x_p = x;
        //     Eigen::VectorXd x_m = x;
        //     Eigen::VectorXd dummy_grad(grad.size());

        //     for (int j = 0; j < x.size(); ++j)
        //     {
        //         x_p = x;
        //         x_m = x;
        //         x_p(j) += eps;
        //         x_m(j) -= eps;

        //         double f_plus  = (*this)(x_p, dummy_grad);
        //         double f_minus = (*this)(x_m, dummy_grad);

        //         grad_num(j) = (f_plus - f_minus) / (2.0 * eps);
        //     }

        //     std::cout << "||grad - grad_num|| = " << (grad - grad_num).norm() << std::endl;
        //     std::cout << "grad     = " << grad.transpose()     << std::endl;
        //     std::cout << "grad_num = " << grad_num.transpose() << std::endl;
        // }

        // // sanity check for gradient
        // Eigen::VectorXd direction = -grad;
        // Eigen::VectorXd dummy_grad(x.size());
        // double alpha = 1e-4;
        // double f0 = fx;
        // double f1 = (*this)(x + alpha * direction, dummy_grad);
        // std::cout << "f0: " << f0 << "   f1: " << f1 << std::endl;

        // std::cout << __FUNCTION__ << __LINE__ << std::endl;
        
        return fx;
    }


    // // Finite Differential Jacobian version
    // double operator()(const Eigen::VectorXd& x, Eigen::VectorXd& grad)
    // {
    //     const double eps_pos = 1.0e-6;
    //     const double eps_rot = 1.0e-6;

    //     double fx = 0.0; // objective function value
    //     double fx_neg = 0.0;
    //     double fx_pos = 0.0;

    //     grad.setZero(x.size());

    //     Eigen::Vector<double, 6> a;
    //     a.tail<3>() = x.head<3>(); // axis-angle vector (alpha, beta, gamma)
    //     a.head<3>() = x.tail<3>(); // translation vector (x,y,z);
    //     const Sophus::SE3d exp_x = Sophus::SE3d::exp(a);
    //     const Sophus::SE3d Twb = T0_ * exp_x;
    //     const Eigen::Matrix3d Rwb = Twb.rotationMatrix();
    //     const Eigen::Vector3d twb = Twb.translation();

    //     Eigen::Vector<double, 6> a_neg;
    //     a_neg.tail<3>() = x.head<3>() - eps_pos; // axis-angle vector (alpha, beta, gamma)
    //     a_neg.head<3>() = x.tail<3>() - eps_rot; // translation vector (x,y,z);
    //     const Sophus::SE3d exp_x_neg = Sophus::SE3d::exp(a_neg);
    //     const Sophus::SE3d Twb_neg = T0_ * exp_x_neg;
    //     const Eigen::Matrix3d Rwb_neg = Twb_neg.rotationMatrix();
    //     const Eigen::Vector3d twb_neg = Twb_neg.translation();

    //     Eigen::Vector<double, 6> a_pos;
    //     a_pos.tail<3>() = x.head<3>() + eps_pos; // axis-angle vector (alpha, beta, gamma)
    //     a_pos.head<3>() = x.tail<3>() + eps_rot; // translation vector (x,y,z);
    //     const Sophus::SE3d exp_x_pos = Sophus::SE3d::exp(a_pos);
    //     const Sophus::SE3d Twb_pos = T0_ * exp_x_pos;
    //     const Eigen::Matrix3d Rwb_pos = Twb_pos.rotationMatrix();
    //     const Eigen::Vector3d twb_pos = Twb_pos.translation();

    //     // std::cout << __FUNCTION__ << __LINE__ << std::endl;
        
    //     // std::vector res_unweighted;
    //     Eigen::VectorXd res(numResidual_);
    //     Eigen::VectorXd resSqrd(numResidual_);
    //     double r2 = 0.0;
    //     for (size_t i=0; i < numResidual_; ++i)
    //     {
    //         const Eigen::Vector3d wp = Rwb * pointScanCurrInBForResidual_->at(i) + twb;
    //         const double r = planeNormalForResidual_->at(i).transpose() * wp + planeDistFromOriginForResidual_->at(i);
    //         r2 = r * r;

    //         // res_unweighted.push_back(res_unweighted);
    //         res(i) = r;
    //         resSqrd(i) = r2;
    //     }

    //     Eigen::VectorXd res_neg(numResidual_);
    //     Eigen::VectorXd resSqrd_neg(numResidual_);
    //     r2 = 0.0;
    //     for (size_t i=0; i < numResidual_; ++i)
    //     {
    //         const Eigen::Vector3d wp = Rwb_neg * pointScanCurrInBForResidual_->at(i) + twb_neg;
    //         const double r = planeNormalForResidual_->at(i).transpose() * wp + planeDistFromOriginForResidual_->at(i);
    //         r2 = r * r;

    //         // res_unweighted.push_back(res_unweighted);
    //         res_neg(i) = r;
    //         resSqrd_neg(i) = r2;
    //     }

    //     Eigen::VectorXd res_pos(numResidual_);
    //     Eigen::VectorXd resSqrd_pos(numResidual_);
    //     r2 = 0.0;
    //     for (size_t i=0; i < numResidual_; ++i)
    //     {
    //         const Eigen::Vector3d wp = Rwb_pos * pointScanCurrInBForResidual_->at(i) + twb_pos;
    //         const double r = planeNormalForResidual_->at(i).transpose() * wp + planeDistFromOriginForResidual_->at(i);
    //         r2 = r * r;

    //         // res_unweighted.push_back(res_unweighted);
    //         res_pos(i) = r;
    //         resSqrd_pos(i) = r2;
    //     }

    //     // std::cout << __FUNCTION__ << __LINE__ << std::endl;
        
    //     // weight them based on robust kernel
    //     Eigen::VectorXd w;  
    //     double s = 0.0; // default scaling just as an option manually tune it
    //     double c = 0.0; // default range just as an option manually tune it
    //     // computeWeightsFromResiduals(r2, w, s, c);
    //     // computeWeightsFromResiduals(res_unweighted, w, s, c);
    //     // computeWeightsFromResiduals(resSqrd, w, s, c);
    //     computeWeightsFromResiduals(res, w, s, c);
    //     // w = Eigen::VectorXd::Ones(res.size());
    //     // fx = res_unweighted.cwiseProduct(w);
    //     // fx = w.transpose() * res_unweighted;
    //     fx = w.transpose() * resSqrd;
    //     fx_neg = w.transpose() * resSqrd_neg;
    //     fx_pos = w.transpose() * resSqrd_pos;

    //     // std::cout << __FUNCTION__ << __LINE__ << std::endl;
        
    //     // compute gradient and overwrite grad
    //     for (size_t i=0; i < numResidual_; ++i)
    //     {
    //         Eigen::Matrix3d bp_hat = getSkewSymMatrix(pointScanCurrInBForResidual_->at(i));

    //         grad.head<3>() += 2.0 * w(i) * res(i) * (-planeNormalForResidual_->at(i).transpose() * Rwb * bp_hat).transpose();
    //         grad.tail<3>() += 2.0 * w(i) * res(i) * (planeNormalForResidual_->at(i).transpose() * Rwb).transpose();
    //     }

    //     // we need to do this for each of 6 variables -> expensive
    //     // put it off


    //     return fx;
    // }

};




// ******* for ifopt setting *******
// Things to try to make IFOPT work:
//  - activate actual constraints
//  - derive a detail derivatives(Jacobians)
//  - figure out how to plug jacobian scaling
//  - print out and log error messages


/**
 *  @file test_vars_constr_cost.h
 *
 *  @brief Example to generate a solver-independent formulation for the problem, taken
 *  from the IPOPT cpp_example.
 *
 *  The example problem to be solved is given as:
 *
 *      min_x f(x) = -(x1-2)^2
 *      s.t.
 *           0 = x0^2 + x1 - 1
 *           -1 <= x0 <= 1
 *
 * In this simple example we only use one set of variables, constraints and
 * cost. However, most real world problems have multiple different constraints
 * and also different variable sets representing different quantities. This
 * framework allows to define each set of variables or constraints absolutely
 * independently from another and correctly stitches them together to form the
 * final optimization problem.
 *
 * For a helpful graphical overview, see:
 * http://docs.ros.org/api/ifopt/html/group__ProblemFormulation.html
 */

class TwistVariables : public ifopt::VariableSet {
public:
    // Every variable set has a name, here "var_set1". this allows the constraints
    // and costs to define values and Jacobians specifically w.r.t this variable set.
    //   ExVariables() : ExVariables("var_set1"){};
    TwistVariables() : TwistVariables("twist"){};
    //   ExVariables(const std::string& name) : VariableSet(2, name)
    //   ExVariables(const std::string& name) : VariableSet(6, name)
    TwistVariables(const std::string& name) : ifopt::VariableSet(6, name)
    {
        // std::cout << __FUNCTION__ << __LINE__ << std::endl;

        // the initial values where the NLP starts iterating from
        // x0_ = 3.5;
        // x1_ = 1.5;

        x_ = Eigen::VectorXd::Zero(6);
        x_lb_ = Eigen::VectorXd::Zero(6);
        x_ub_ = Eigen::VectorXd::Zero(6);

        // // for now, just initialize the variables
        // rx_ = 0.0;
        // ry_ = 0.0;
        // rz_ = 0.0;
        // tx_ = 0.0;
        // ty_ = 0.0;
        // tz_ = 0.0;
    }

    // Here is where you can transform the Eigen::Vector into whatever
    // internal representation of your variables you have (here two doubles, but
    // can also be complex classes such as splines, etc..

    // VectorXd x is a 18 DoF vector where all the variable init values and constraints are stacked
    void SetVariables(const Eigen::VectorXd& x) override
    {
        // std::cout << __FUNCTION__ << __LINE__ << std::endl;
        x_ = x;
    };

    // Here is the reverse transformation from the internal representation to
    // to the Eigen::Vector
    //   VectorXd GetValues() const override { return Vector2d(x0_, x1_); };
    Eigen::VectorXd GetValues() const override { return x_; };

    // Each variable has an upper and lower bound set here
    ifopt::Component::VecBound GetBounds() const override
    {
        // std::cout << __FUNCTION__ << __LINE__ << std::endl;
        ifopt::Component::VecBound bounds(GetRows());
        // bounds.at(0) = Bounds(-1.0, 1.0);
        // bounds.at(1) = NoBound;

        bounds.at(0) = ifopt::Bounds(x_lb_(0), x_ub_(0));
        bounds.at(1) = ifopt::Bounds(x_lb_(1), x_ub_(1));
        bounds.at(2) = ifopt::Bounds(x_lb_(2), x_ub_(2));
        bounds.at(3) = ifopt::Bounds(x_lb_(3), x_ub_(3));
        bounds.at(4) = ifopt::Bounds(x_lb_(4), x_ub_(4));
        bounds.at(5) = ifopt::Bounds(x_lb_(5), x_ub_(5));

        // bounds.at(0) = ifopt::NoBound;
        // bounds.at(1) = ifopt::NoBound;
        // bounds.at(2) = ifopt::NoBound;
        // bounds.at(3) = ifopt::NoBound;
        // bounds.at(4) = ifopt::NoBound;
        // bounds.at(5) = ifopt::NoBound;

        return bounds;
    }

    void SetInitialVariables(const Eigen::VectorXd& x0, const Eigen::VectorXd& x_lb, const Eigen::VectorXd& x_ub, Sophus::SE3d& T0){
        
        // std::cout << __FUNCTION__ << __LINE__ << std::endl;
        x_ = x0;
        x_lb_ = x_lb;
        x_ub_ = x_ub;
        T0_ = T0;
    }
    
    // void SetInitialVariables(const Eigen::VectorXd& x0, Sophus::SE3d& T0){
        
    //     // std::cout << __FUNCTION__ << __LINE__ << std::endl;
    //     x_ = x0;
    //     T0_ = T0;
    // }

    private:
    //   double x0_, x1_;
    Eigen::VectorXd x_;
    Eigen::VectorXd x_lb_;
    Eigen::VectorXd x_ub_;
    Sophus::SE3d T0_;
};

class KinematicConstraint : public ifopt::ConstraintSet {
public:
    //   ExConstraint() : ExConstraint("constraint1") {}
    KinematicConstraint() : KinematicConstraint("kinematic_constraint") {}

    // This constraint set just contains 1 constraint, however generally
    // each set can contain multiple related constraints.
    // ExConstraint(const std::string& name) : ConstraintSet(1, name) {}
    // KinematicConstraint(const std::string& name) : ifopt::ConstraintSet(1, name) {}
    // KinematicConstraint(const std::string& name) : ifopt::ConstraintSet(0, name) {}
    KinematicConstraint(const std::string& name) : ifopt::ConstraintSet(12, name) {}
    
    void SetConstraints(const Eigen::VectorXd& lb, const Eigen::VectorXd& ub)
    {
        lb_ = lb;
        ub_ = ub;
    }

    // The constraint value minus the constant value "1", moved to bounds.
    Eigen::VectorXd GetValues() const override
    {
        // // std::cout << __FUNCTION__ << __LINE__ << std::endl;
        // Eigen::VectorXd g(GetRows());
        // // Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
        // // g(0)       = std::pow(x(0), 2) + x(1);
        // return g;

        Eigen::VectorXd g(GetRows()); // size 12

        Eigen::VectorXd x =
            GetVariables()->GetComponent("twist")->GetValues(); // size 6

        // lower bounds: x_i - lb_i >= 0
        for (int i = 0; i < 6; ++i)
            g(i) = x(i) - lb_(i);

        // upper bounds: ub_i - x_i >= 0
        for (int i = 0; i < 6; ++i)
            g(6 + i) = ub_(i) - x(i);

        return g;
    }

    // The only constraint in this set is an equality constraint to 1.
    // Constant values should always be put into GetBounds(), not GetValues().
    // For inequality constraints (<,>), use Bounds(x, inf) or Bounds(-inf, x).
    ifopt::Component::VecBound GetBounds() const override
    {
        // // std::cout << __FUNCTION__ << __LINE__ << std::endl;
        // ifopt::Component::VecBound b(GetRows());
        // // b.at(0) = Bounds(1.0, 1.0);
        // return b;

        ifopt::Component::VecBound b(GetRows());
        for (int i = 0; i < GetRows(); ++i)
            b.at(i) = ifopt::Bounds(0.0, ifopt::inf); // g_i >= 0
        return b;
    }

    // This function provides the first derivative of the constraints.
    // In case this is too difficult to write, you can also tell the solvers to
    // approximate the derivatives by finite differences and not overwrite this
    // function, e.g. in ipopt.cc::use_jacobian_approximation_ = true
    // Attention: see the parent class function for important information on sparsity pattern.
    void FillJacobianBlock(std::string var_set,
                            Eigen::SparseMatrix<double, Eigen::RowMajor>& jac_block) const override
    {
        // std::cout << __FUNCTION__ << __LINE__ << std::endl;
        // must fill only that submatrix of the overall Jacobian that relates
        // to this constraint and "var_set1". even if more constraints or variables
        // classes are added, this submatrix will always start at row 0 and column 0,
        // thereby being independent from the overall problem.
        // if (var_set == "var_set1") {
        // Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();

        // jac_block.coeffRef(0, 0) =
        //     2.0 * x(0);  // derivative of first constraint w.r.t x0
        // jac_block.coeffRef(0, 1) =
        //     1.0;  // derivative of first constraint w.r.t x1
        // }

        // for now, we don't use general constraints so leave this blank
        // jac_block.setZero();

        // for setting inequality constraints in this constraint block
        if (var_set != "twist")
            return;

        jac_block.setZero();

        // Each row has exactly one non-zero derivative
        // d(x_i - lb_i)/dx_i = +1
        for (int i = 0; i < 6; ++i)
            jac_block.coeffRef(i, i) = 1.0;

        // d(ub_i - x_i)/dx_i = -1
        for (int i = 0; i < 6; ++i)
            jac_block.coeffRef(6 + i, i) = -1.0;
    }

    Eigen::VectorXd lb_, ub_;    
};

class PointPlaneCost : public ifopt::CostTerm {
public:
    PointPlaneCost() : PointPlaneCost("point_to_plane_cost") {}
    PointPlaneCost(const std::string& name) : ifopt::CostTerm(name) {}

    double GetCost() const override
    {
        // std::cout << __FUNCTION__ << __LINE__ << std::endl;
        // Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
        // return -std::pow(x(1) - 2, 2);

        Eigen::VectorXd x = GetVariables()->GetComponent("twist")->GetValues();
        Eigen::SparseMatrix<double, Eigen::RowMajor> dummy_jacobian;
        double cost_computed = 0.0;
        ComputeCostAndJacobian(x, cost_computed, dummy_jacobian);

        return cost_computed;
    }

    void FillJacobianBlock(std::string var_set, Eigen::SparseMatrix<double, Eigen::RowMajor>& jac) const override
    {
        // // std::cout << __FUNCTION__ << __LINE__ << std::endl;
        // // if (var_set == "var_set1") {
        // //     Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();

        // //     jac.coeffRef(0, 0) = 0.0;                  // derivative of cost w.r.t x0
        // //     jac.coeffRef(0, 1) = -2.0 * (x(1) - 2.0);  // derivative of cost w.r.t x1
        // // }

        if (var_set == "twist") {
            Eigen::VectorXd x = GetVariables()->GetComponent("twist")->GetValues();
            
            Eigen::SparseMatrix<double, Eigen::RowMajor> jacobian_computed;
            double dummy_cost = 0.0;

            ComputeCostAndJacobian(x, dummy_cost, jacobian_computed);
            jac.coeffRef(0, 0) = jacobian_computed.coeffRef(0, 0);  // derivative of cost w.r.t rx
            jac.coeffRef(0, 1) = jacobian_computed.coeffRef(0, 1);  // derivative of cost w.r.t ry
            jac.coeffRef(0, 2) = jacobian_computed.coeffRef(0, 2);  // derivative of cost w.r.t rz
            jac.coeffRef(0, 3) = jacobian_computed.coeffRef(0, 3);  // derivative of cost w.r.t tx
            jac.coeffRef(0, 4) = jacobian_computed.coeffRef(0, 4);  // derivative of cost w.r.t ty
            jac.coeffRef(0, 5) = jacobian_computed.coeffRef(0, 5);  // derivative of cost w.r.t tz
        }

        // jac.setZero();
    }

    void ComputeCostAndJacobian(const Eigen::VectorXd& x, double& cost, Eigen::SparseMatrix<double, Eigen::RowMajor>& jacobian) const
    {        
        // std::cout << __FUNCTION__ << __LINE__ << std::endl;

        if(cache_valid_ && x.isApprox(x_cost_jacobian_computed_)){
            // if the cost and jacobian is already computed, just return them
            jacobian = jacobian_computed_;
            cost = cost_computed_;
            // std::cout << __FUNCTION__ << __LINE__ << std::endl;
            return;
        }
        
        // if the cost and jacobian is not computed yet, compute them based on the current T0_ and x

        Eigen::Vector<double, 6> a;
        a.tail<3>() = x.head<3>(); // axis-angle vector (alpha, beta, gamma)
        a.head<3>() = x.tail<3>(); // translation vector (x,y,z)
        const Sophus::SE3d exp_x = Sophus::SE3d::exp(a);
        const Sophus::SE3d Twb = T0_ * exp_x;
        const Eigen::Matrix3d Rwb = Twb.rotationMatrix();
        const Eigen::Vector3d twb = Twb.translation();

        Eigen::VectorXd res(numResidual_);
        Eigen::VectorXd res_forWeight(numResidual_);
        Eigen::VectorXd resSqrd(numResidual_);
        // compute residual based on the current pose
        double r2 = 0.0;
        for (size_t i=0; i < numResidual_; ++i){
            const Eigen::Vector3d wp = Rwb * pointScanCurrInBForResidual_->at(i) + twb;
            const double r = planeNormalForResidual_->at(i).transpose() * wp + planeDistFromOriginForResidual_->at(i);
            r2 = r * r;

            // res_unweighted.push_back(res_unweighted);
            res(i) = r;
            resSqrd(i) = r2;
        }
        // weight them based on robust kernel
        // Eigen::VectorXd w;  
        double s = 0.0; // default scaling just as an option manually tune it
        double c = 0.0; // default range just as an option manually tune it
        // computeWeightsFromResiduals(res, w, s, c);

        if(weight_fixed_.isApprox(Eigen::VectorXd::Zero(numResidual_))){
            computeWeightsFromResiduals(res, weight_fixed_, s, c);
        }
        // w = Eigen::VectorXd::Ones(res.size()); // for now no robust weight calculation
        // cost_computed_ = w.transpose() * resSqrd;
        cost_computed_ = weight_fixed_.transpose() * resSqrd;

        Eigen::Matrix<double, 6, 1> grad;
        grad.setZero();
        // compute gradient and overwrite grad
        for (size_t i=0; i < numResidual_; ++i){
            Eigen::Matrix3d bp_hat = getSkewSymMatrix(pointScanCurrInBForResidual_->at(i));

            // grad.head<3>() += 2.0 * w(i) * res(i) * (-planeNormalForResidual_->at(i).transpose() * Rwb * bp_hat).transpose();
            // grad.tail<3>() += 2.0 * w(i) * res(i) * (planeNormalForResidual_->at(i).transpose() * Rwb).transpose();
            grad.head<3>() += 2.0 * weight_fixed_(i) * res(i) * (-planeNormalForResidual_->at(i).transpose() * Rwb * bp_hat).transpose();
            grad.tail<3>() += 2.0 * weight_fixed_(i) * res(i) * (planeNormalForResidual_->at(i).transpose() * Rwb).transpose();
        }

        // std::cout << "grad: " << grad.transpose() << std::endl;
        
        // Now transfer grad -> sparse
        // jacobian_computed_.setZero(1, 6);
        jacobian_computed_.resize(1, 6);    // set size to 1×6
        jacobian_computed_.setZero();       // zero all entries
        for (int k = 0; k < 6; ++k) {
            jacobian_computed_.coeffRef(0, k) = grad(k);
        }

        // update cache key and output
        x_cost_jacobian_computed_ = x;
        cost = cost_computed_;
        jacobian = jacobian_computed_;
        cache_valid_ = true;

        // std::cout << __FUNCTION__ << __LINE__ << std::endl;
    }

    void SetInitialPoseAndResidual(
        Sophus::SE3d T0,
        int numResidual,
        std::vector<Eigen::Vector3d>& pointScanCurrInBForResidual,
        std::vector<Eigen::Vector3d>& planeNormalForResidual,
        std::vector<double>& planeDistFromOriginForResidual
    )
    {
        // std::cout << __FUNCTION__ << __LINE__ << std::endl;

        T0_ = T0;
        numResidual_ = numResidual;
        pointScanCurrInBForResidual_ = &pointScanCurrInBForResidual;
        planeNormalForResidual_ = &planeNormalForResidual;
        planeDistFromOriginForResidual_ = &planeDistFromOriginForResidual;

        // x_cost_jacobian_computed_ = Eigen::VectorXd::Zero(6);
        // // jacobian_computed_ = Eigen::SparseMatrix<double, Eigen::RowMajor>::Zero(6);
        // jacobian_computed_;
        x_cost_jacobian_computed_.resize(6);
        x_cost_jacobian_computed_.setZero();
        jacobian_computed_.resize(0,0);
        cost_computed_ = 0.0;
        cache_valid_ = false;

        weight_fixed_ = Eigen::VectorXd::Zero(numResidual_);
    }

    Sophus::SE3d T0_;
    int numResidual_ = 0;
    std::vector<Eigen::Vector3d> *pointScanCurrInBForResidual_ = nullptr;
    std::vector<Eigen::Vector3d> *planeNormalForResidual_ = nullptr;
    std::vector<double> *planeDistFromOriginForResidual_ = nullptr;
    mutable Eigen::VectorXd weight_fixed_;

    mutable Eigen::VectorXd x_cost_jacobian_computed_ = Eigen::VectorXd::Zero(6);
    // Eigen::SparseMatrix<double, Eigen::RowMajor> jacobian_computed_ = Eigen::SparseMatrix<double, Eigen::RowMajor>::Zero(6);
    mutable Eigen::SparseMatrix<double, Eigen::RowMajor> jacobian_computed_;
    mutable double cost_computed_ = 0.0;
    mutable bool cache_valid_ = false;
};


// Todo with inequality constraints:
//  [DONE] - Ceres + box constraint -> hard to put constraints on quaternion parameterization. Also, right multipicative Ceres implementation does not work.
//  [Pending] - LBFGSpp + box constraint -> try as it is (the jacobian is accurate only locally), and see numerical dev. available
//              -> Implementation is kind of time-consuming. Put this off
//  - IFOPT + reasonable box constraint or trust-region like setting
//  - qpmad + on LM objective function -> this is not really active set + trust region because it does not perform trust-region on Lagrangian
//      -> probably I can look into qpmad code and see if it uses the trust-region like step size correction.


#endif // COMMON_H