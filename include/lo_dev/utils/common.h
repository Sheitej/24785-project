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
#include <gtsam/navigation/ImuFactor.h>

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
        dumpIntoFile("/sandbox/ysugano/ros2_ws/src/livo_dev/lo_dev/Log");
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
        double lag_;
        bool useLOPriorFactor_;
        std::string fgStateInitGuessSource_;
    };

    struct FactorGraphRecord
    {
        // FactorGraphStateRecord() = default;
        // FactorGraphStateRecord(const std::string &recordName)
        // {
        //     recordName_ = recordName;
        // }

        FactorGraphRecord(){}

        // std::string recordName_;
        // std::map<double, std::pair<Eigen::Vector3d, Eigen::Quaterniond>> stateTimeLine_;
        double keyframeTimestamp_;
        double timeCurrScanBeg_;
        double timeCurrScanEnd_;
        double timeImuBeg_;
        double timeImuEnd_;
        int keyFrameId_;
        gtsam::Pose3 lidarBetweenFactor_;
        gtsam::Pose3 lidarPriorFactor_;
        gtsam::Pose3 imuBetweenFactor_;
        gtsam::NavState statePrevKf_;
        gtsam::imuBias::ConstantBias biasPrevKf_;
        gtsam::NavState initGuessStateCurrKf_;
        gtsam::imuBias::ConstantBias initGuessBiasCurrKf_;
        gtsam::NavState optimizedStateCurrKf_;
        gtsam::imuBias::ConstantBias optimizedBiasCurrKf_;
        int numKeyframes_;
        bool isFactorGraphOn_;
    };

    FactorGraphLogger() : logger_(rclcpp::get_logger("Default")) {}
    explicit FactorGraphLogger(const rclcpp::Logger& logger) : logger_(logger) {}

    ~FactorGraphLogger()
    {
        // dumpIntoFile("Log");
        dumpIntoFile("/sandbox/ysugano/ros2_ws/src/livo_dev/lo_dev/Log");
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
        const double& lag,
        const bool& useLOPriorFactor,
        const std::string& fgStateInitGuessSource
    )
    {
        auto now = std::chrono::steady_clock::now();
        executionTime_ = std::chrono::duration<double>(now.time_since_epoch()).count();
        cfgParams_.lidarCorrectionNoise_ = lidarCorrectionNoise;
        cfgParams_.imuAccNoise_ = imuAccNoise;
        cfgParams_.imuGyrNoise_ = imuGyrNoise;
        cfgParams_.imuAccNoise_BiasRandomWalk_ = imuAccNoise_BiasRandomWalk;
        cfgParams_.imuGyrNoise_BiasRandomWalk_ = imuGyrNoise_BiasRandomWalk;
        cfgParams_.gravityNorm_ = gravityNorm;
        cfgParams_.lag_ = lag;
        cfgParams_.useLOPriorFactor_ = useLOPriorFactor;
        cfgParams_.fgStateInitGuessSource_ = fgStateInitGuessSource;
    }

    // void createRecord(const std::string &recordName)
    // {
    //     StateRecord newRecord(recordName);
    //     records_.insert(std::make_pair(recordName, newRecord));
    // }

    void recordFGState(
        const double& keyframeTimestamp,
        const double& timeCurrScanBeg,
        const double& timeCurrScanEnd,
        const double& timeImuBeg,
        const double& timeImuEnd,
        const int& keyFrameId,
        const gtsam::Pose3& lidarBetweenFactor,
        const gtsam::Pose3& lidarPriorFactor,
        const gtsam::Pose3& imuBetweenFactor,
        const gtsam::NavState& statePrevKf,
        const gtsam::imuBias::ConstantBias& biasPrevKf,
        const gtsam::NavState& initGuessStateCurrKf,
        const gtsam::imuBias::ConstantBias& initGuessBiasCurrKf,
        const gtsam::NavState& optimizedStateCurrKf,
        const gtsam::imuBias::ConstantBias& optimizedBiasCurrKf,
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
        record.imuBetweenFactor_ = imuBetweenFactor;
        record.statePrevKf_ = statePrevKf;
        record.biasPrevKf_ = biasPrevKf;
        record.initGuessStateCurrKf_ = initGuessStateCurrKf;
        record.initGuessBiasCurrKf_ = initGuessBiasCurrKf;
        record.optimizedStateCurrKf_ = optimizedStateCurrKf;
        record.optimizedBiasCurrKf_ = optimizedBiasCurrKf;
        record.numKeyframes_ = numKeyframes;
        record.isFactorGraphOn_ = isFactorGraphOn;

        records_.emplace_back(record);
    }

    void dumpIntoFile(const std::string &filePath)
    {
        std::string fileName;
        std::string time;
        time = std::to_string(executionTime_);

        fileName = filePath + "/factor_graph_log_" + time + ".txt";
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
        
        // write config parameters and 
        // ofs.setf(std::ios::fixed);
        // ofs << std::setprecision(std::numeric_limits<double>::max_digits10);
        ofs << "========== Executaion Time: " << time << std::endl;
        ofs << "========== Config Parameters ==========" << std::endl;
        ofs << "                lidar_correction_noise: " << cfgParams_.lidarCorrectionNoise_ << std::endl;
        ofs << "                                 acc_n: " << cfgParams_.imuAccNoise_ << std::endl;
        ofs << "                                 gyr_n: " << cfgParams_.imuGyrNoise_ << std::endl;
        ofs << "                                 acc_w: " << cfgParams_.imuAccNoise_BiasRandomWalk_ << std::endl;
        ofs << "                                 gyr_w: " << cfgParams_.imuGyrNoise_BiasRandomWalk_ << std::endl;
        ofs << "                                 acc_w: " << cfgParams_.imuAccNoise_BiasRandomWalk_ << std::endl;
        ofs << "                                g_norm: " << cfgParams_.gravityNorm_ << std::endl;
        ofs << "                                   lag: " << cfgParams_.lag_ << std::endl;
        ofs << " use_lo_prior_factor_wo_between_factor: " << cfgParams_.useLOPriorFactor_ << std::endl;
        ofs << "        factor_graph_init_guess_source: " << cfgParams_.fgStateInitGuessSource_ << std::endl;

        // lidarCorrectionNoise_ = lidarCorrectionNoise;
        // imuAccNoise_ = imuAccNoise;
        // imuGyrNoise_ = imuGyrNoise;
        // imuAccNoise_BiasRandomWalk_ = imuAccNoise_BiasRandomWalk;
        // imuGyrNoise_BiasRandomWalk_ = imuGyrNoise_BiasRandomWalk;
        // gravityNorm_ = gravityNorm;
        // lag_ = lag;
        // useLOPriorFactor_ = useLOPriorFactor;
        // fgStateInitGuessSource_ = fgStateInitGuessSource;

        // write config parameters

        // write all the records
        for (auto rec: records_){
            ofs << "====================" << std::endl;
            ofs << " Keyframe timestamp          : " << rec.keyframeTimestamp_ << std::endl;
            ofs << " Keyframe id                 : " << rec.keyFrameId_ << std::endl;
            ofs << " Num. of keyframes in window : " << rec.keyFrameId_ << std::endl;
            ofs << " Lidar scan beginning time   : " << rec.timeCurrScanBeg_ << std::endl;
            ofs << " Lidar scan ending time      : " << rec.timeCurrScanEnd_ << std::endl;
            ofs << " Imu beginning time          : " << rec.timeImuBeg_ << std::endl;
            ofs << " Imu ending time             : " << rec.timeImuEnd_ << std::endl;

            double x, y, z, roll, pitch, yaw, vx, vy, vz;
            ofs << "  ----- Factors (dx, dy, dz, droll, dpitch, dyaw)-----  " << std::endl;
            if (!rec.lidarBetweenFactor_.equals(gtsam::Pose3())){
            getStateVectorFromGtsam(rec.lidarBetweenFactor_, x, y, z, roll, pitch, yaw);
            ofs << "        Lidar Odom Between Factor  : " << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw << std::endl;
            }
            if (!rec.lidarPriorFactor_.equals(gtsam::Pose3())){
            getStateVectorFromGtsam(rec.lidarPriorFactor_, x, y, z, roll, pitch, yaw);
            ofs << "        Lidar Odom Prior Factor    : " << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw << std::endl;
            }
            getStateVectorFromGtsam(rec.imuBetweenFactor_, x, y, z, roll, pitch, yaw);
            ofs << "             Imu Between Factor    : " << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw << std::endl;

            Eigen::VectorXd bias;
            ofs << "  ----- States (x, y, z, roll, pitch, yaw, vx, vy, vz, bax, bay, baz, bgx, bgy, bgz)-----  " << std::endl;
            getStateVectorFromGtsam(rec.statePrevKf_, x, y, z, roll, pitch, yaw, vx, vy, vz);
            bias = rec.biasPrevKf_.vector();
            ofs << "        Previous Kf State           : " << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw << ", " << vx << ", " << vy << ", " << vz << ", " 
                << bias[0] << ", " << bias[1] << ", " << bias[2] << ", " << bias[3] << ", " << bias[4] << ", " << bias[5] << std::endl;

            getStateVectorFromGtsam(rec.initGuessStateCurrKf_, x, y, z, roll, pitch, yaw, vx, vy, vz);
            bias = rec.initGuessBiasCurrKf_.vector();
            ofs << "        Init Guess Current Kf State : " << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw << ", " << vx << ", " << vy << ", " << vz << ", " 
                << bias[0] << ", " << bias[1] << ", " << bias[2] << ", " << bias[3] << ", " << bias[4] << ", " << bias[5] << std::endl;

            getStateVectorFromGtsam(rec.optimizedStateCurrKf_, x, y, z, roll, pitch, yaw, vx, vy, vz);
            bias = rec.optimizedBiasCurrKf_.vector();
            ofs << "        Optimized Current Kf State  : " << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw << ", " << vx << ", " << vy << ", " << vz << ", " 
                << bias[0] << ", " << bias[1] << ", " << bias[2] << ", " << bias[3] << ", " << bias[4] << ", " << bias[5] << std::endl;
            
            ofs << std::endl;
            ofs << std::endl;
        }
        
        // double keyframeTimestamp_;
        // double timeCurrScanBeg_;
        // double timeCurrScanEnd_;
        // double timeImuBeg_;
        // double timeImuEnd_;
        // int keyFrameId_;
        // gtsam::Pose3 lidarBetweenFactor_;
        // gtsam::Pose3 lidarPriorFactor_;
        // gtsam::Pose3 imuBetweenFactor_;
        // gtsam::NavState statePrevKf_;
        // gtsam::imuBias::ConstantBias biasPrevKf_;
        // gtsam::NavState initGuessStateCurrKf_;
        // gtsam::imuBias::ConstantBias initGuessBiasCurrKf_;
        // gtsam::NavState optimizedStateCurrKf_;
        // gtsam::imuBias::ConstantBias optimizedBiasCurrKf_;
        // int numKeyframes_;
        // bool isFactorGraphOn_;

        // for (size_t i=0; i<timeLine_.size(); i++)
        // {
        //     auto time = timeLine_[i];
        //     // write time
        //     ofs.setf(std::ios::fixed);
        //     ofs << std::setprecision(std::numeric_limits<double>::max_digits10);
        //     ofs << "Time: " << time << std::endl;

        //     for (auto it: records_)
        //     {
        //         // auto itRecord = it->second.stateTimeLine_.find(time);
        //         auto itRecord = it.second.stateTimeLine_.find(time);
        //         if (itRecord != it.second.stateTimeLine_.end())
        //         {
        //             Eigen::Vector3d pos = itRecord->second.first;
        //             double roll, pitch, yaw;
        //             convertQuatToEuler(itRecord->second.second, roll, pitch, yaw);

        //             auto offset = it.first.length();

        //             // write down the state name
        //             ofs << std::setw(10) << "[" << it.first << "]:" ;
        //             ofs << std::setw(15-offset) << std::setprecision(6) << pos.x() << ", " << pos.y() << ", " << pos.z() << ", " << roll << ", " << pitch << ", " << yaw << std::endl;
        //         }
        //         // add an empty line for readability
        //     }
        //     ofs << std::endl;
        // }
        // ofs.close();

    }

private:
    rclcpp::Logger logger_;
    // std::map<std::string, StateRecord> records_;
    // std::vector<double> timeLine_;

    // std::map<double, FactorGraphRecord> records_;
    std::vector<FactorGraphRecord> records_;

    double executionTime_;
    Config cfgParams_;


};


#endif // COMMON_H