#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

typedef pcl::PointXYZI PointType;

// 传感器型号
enum class SensorType { VELODYNE, OUSTER, LIVOX };

class ParamServer {
 public:

  ros::NodeHandle nh;

  std::string robot_id;

  // 话题
  string pointCloudTopic; // points_raw 原始点云数据
  string imuTopic;        // imu_raw 对应park数据集，imu_correct对应outdoor数据集，都是原始imu数据，不同的坐标系表示
  string odomTopic;       // odometry/imu，imu里程计，imu积分计算得到
  string gpsTopic;        // odometry/gps，gps里程计

  // 坐标系
  string lidarFrame;      // 激光坐标系
  string baselinkFrame;   // 载体坐标系
  string odometryFrame;   // 里程计坐标系
  string mapFrame;        // 世界坐标系

  // GPS参数
  bool useImuHeadingInitialization;   //
  bool useGpsElevation;
  float gpsCovThreshold;
  float poseCovThreshold;

  // 保存PCD
  bool savePCD;               // 是否保存地图
  string savePCDDirectory;    // 保存路径

  // 激光传感器参数
  SensorType sensor;      // 传感器型号
  int N_SCAN;             // 扫描线数，例如16、64
  int Horizon_SCAN;       // 扫描一周计数，例如每隔0.2°扫描一次，一周360°可以扫描1800次
  int downsampleRate;     // 扫描线降采样，跳过一些扫描线
  float lidarMinRange;    // 最小范围
  float lidarMaxRange;    // 最大范围

  // IMU参数
  float imuAccNoise;          // 加速度噪声标准差
  float imuGyrNoise;          // 角速度噪声标准差
  float imuAccBiasN;          //
  float imuGyrBiasN;
  float imuGravity;           // 重力加速度
  float imuRPYWeight;
  vector<double> extRotV;
  vector<double> extRPYV;
  vector<double> extTransV;
  Eigen::Matrix3d extRot;     // xyz坐标系旋转
  Eigen::Matrix3d extRPY;     // RPY欧拉角的变换关系
  Eigen::Vector3d extTrans;   // xyz坐标系平移
  Eigen::Quaterniond extQRPY;

  // LOAM
  float edgeThreshold;
  float surfThreshold;
  int edgeFeatureMinValidNum;
  int surfFeatureMinValidNum;

  // voxel filter paprams
  float odometrySurfLeafSize;
  float mappingCornerLeafSize;
  float mappingSurfLeafSize;

  float z_tollerance;
  float rotation_tollerance;

  // CPU Params
  int numberOfCores;
  double mappingProcessInterval;

  // Surrounding map
  float surroundingkeyframeAddingDistThreshold;
  float surroundingkeyframeAddingAngleThreshold;
  float surroundingKeyframeDensity;
  float surroundingKeyframeSearchRadius;

  // Loop closure
  bool loopClosureEnableFlag;
  float loopClosureFrequency;
  int surroundingKeyframeSize;
  float historyKeyframeSearchRadius;
  float historyKeyframeSearchTimeDiff;
  int historyKeyframeSearchNum;
  float historyKeyframeFitnessScore;

  // global map visualization radius
  float globalMapVisualizationSearchRadius;
  float globalMapVisualizationPoseDensity;
  float globalMapVisualizationLeafSize;

  //保存数据
  std::ofstream imu_ofs_;

  ParamServer() {
    nh.param<std::string>("/robot_id", robot_id, "roboat");

    // 从param server中读取key为"lio_sam/pointCloudTopic"对应的参数，存pointCloudTopic，第三个参数是默认值
    // launch文件中定义<rosparam file="$(find lio_sam)/config/params.yaml" command="load" />，从yaml文件加载参数
    nh.param<std::string>("lio_sam/pointCloudTopic", pointCloudTopic, "points_raw");
    nh.param<std::string>("lio_sam/imuTopic", imuTopic, "imu_correct");
    nh.param<std::string>("lio_sam/odomTopic", odomTopic, "odometry/imu");
    nh.param<std::string>("lio_sam/gpsTopic", gpsTopic, "odometry/gps");

    nh.param<std::string>("lio_sam/lidarFrame", lidarFrame, "base_link");
    nh.param<std::string>("lio_sam/baselinkFrame", baselinkFrame, "base_link");
    nh.param<std::string>("lio_sam/odometryFrame", odometryFrame, "odom");
    nh.param<std::string>("lio_sam/mapFrame", mapFrame, "map");

    nh.param<bool>("lio_sam/useImuHeadingInitialization", useImuHeadingInitialization, false);
    nh.param<bool>("lio_sam/useGpsElevation", useGpsElevation, false);
    nh.param<float>("lio_sam/gpsCovThreshold", gpsCovThreshold, 2.0);
    nh.param<float>("lio_sam/poseCovThreshold", poseCovThreshold, 25.0);

    nh.param<bool>("lio_sam/savePCD", savePCD, false);
    nh.param<std::string>("lio_sam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

    std::string sensorStr;
    nh.param<std::string>("lio_sam/sensor", sensorStr, "");
    if (sensorStr == "velodyne") {
      sensor = SensorType::VELODYNE;
    } else if (sensorStr == "ouster") {
      sensor = SensorType::OUSTER;
    } else if (sensorStr == "livox") {
      sensor = SensorType::LIVOX;
    } else {
      ROS_ERROR_STREAM(
          "Invalid sensor type (must be either 'velodyne' or 'ouster'): " << sensorStr);
      ros::shutdown();
    }

    nh.param<int>("lio_sam/N_SCAN", N_SCAN, 16);
    nh.param<int>("lio_sam/Horizon_SCAN", Horizon_SCAN, 1800);
    nh.param<int>("lio_sam/downsampleRate", downsampleRate, 1);
    nh.param<float>("lio_sam/lidarMinRange", lidarMinRange, 1.0);
    nh.param<float>("lio_sam/lidarMaxRange", lidarMaxRange, 1000.0);

    nh.param<float>("lio_sam/imuAccNoise", imuAccNoise, 0.01);
    nh.param<float>("lio_sam/imuGyrNoise", imuGyrNoise, 0.001);
    nh.param<float>("lio_sam/imuAccBiasN", imuAccBiasN, 0.0002);
    nh.param<float>("lio_sam/imuGyrBiasN", imuGyrBiasN, 0.00003);
    nh.param<float>("lio_sam/imuGravity", imuGravity, 9.80511);
    nh.param<float>("lio_sam/imuRPYWeight", imuRPYWeight, 0.01);
    nh.param<vector<double>>("lio_sam/extrinsicRot", extRotV, vector<double>());
    nh.param<vector<double>>("lio_sam/extrinsicRPY", extRPYV, vector<double>());
    nh.param<vector<double>>("lio_sam/extrinsicTrans", extTransV, vector<double>());
    extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
    extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
    extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
    extQRPY = Eigen::Quaterniond(extRPY);

    nh.param<float>("lio_sam/edgeThreshold", edgeThreshold, 0.1);
    nh.param<float>("lio_sam/surfThreshold", surfThreshold, 0.1);
    nh.param<int>("lio_sam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
    nh.param<int>("lio_sam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

    nh.param<float>("lio_sam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
    nh.param<float>("lio_sam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
    nh.param<float>("lio_sam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

    nh.param<float>("lio_sam/z_tollerance", z_tollerance, FLT_MAX);
    nh.param<float>("lio_sam/rotation_tollerance", rotation_tollerance, FLT_MAX);

    nh.param<int>("lio_sam/numberOfCores", numberOfCores, 2);
    nh.param<double>("lio_sam/mappingProcessInterval", mappingProcessInterval, 0.15);

    nh.param<float>("lio_sam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
    nh.param<float>("lio_sam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
    nh.param<float>("lio_sam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
    nh.param<float>("lio_sam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

    nh.param<bool>("lio_sam/loopClosureEnableFlag", loopClosureEnableFlag, false);
    nh.param<float>("lio_sam/loopClosureFrequency", loopClosureFrequency, 1.0);
    nh.param<int>("lio_sam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
    nh.param<float>("lio_sam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
    nh.param<float>("lio_sam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
    nh.param<int>("lio_sam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
    nh.param<float>("lio_sam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

    nh.param<float>("lio_sam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
    nh.param<float>("lio_sam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
    nh.param<float>("lio_sam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

    //保存数据
    std::string data_path = "/home/he/slam_data/trajectory/imu.txt";;
    imu_ofs_.open(data_path.c_str(), std::ios::out);
    usleep(100);
  }

  /**
   * imu原始测量数据转换到lidar系，加速度、角速度、RPY
  */
  sensor_msgs::Imu imuConverter(const sensor_msgs::Imu &imu_in) {
    sensor_msgs::Imu imu_out = imu_in;
    // 加速度，只跟xyz坐标系的旋转有关系
//        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
    double w = imu_in.angular_velocity.z / 180 * M_PI;
    double acc_x = imu_in.linear_acceleration.x - 4.36 * w * w;
    double acc_y = imu_in.linear_acceleration.y - 0.16 * w * w;

    // 臂长标定 (后续可以加上角度标定)  3.36 1.16
//    static double count = 0, l_x = 0, l_y = 0;
//    l_x += imu_in.linear_acceleration.x / (w * w);
//    l_y += imu_in.linear_acceleration.y / (w * w);
//    count++;
//    std::cout << "l_x: " << l_x/count << " l_y: " << l_y/count << std::endl;

    Eigen::Vector3d acc(acc_x, acc_y, 0);
//    std::cout << "acc_X: " << acc.x() << " acc_Y: " << acc.y() << std::endl;
    acc = extRot * acc;
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();
//    imu_out.linear_acceleration.x = 0;
//    imu_out.linear_acceleration.y = 0;
//    imu_out.linear_acceleration.z = 0;
    // 角速度，只跟xyz坐标系的旋转有关系
//        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    Eigen::Vector3d gyr(imu_in.angular_velocity.x / 180 * M_PI,
                        imu_in.angular_velocity.y / 180 * M_PI,
                        imu_in.angular_velocity.z / 180 * M_PI);
    gyr = extRot * gyr;
    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();
//      imu_out.angular_velocity.x = 0;
//      imu_out.angular_velocity.y = 0;
//      imu_out.angular_velocity.z = 0;
    // RPY
//        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
    Eigen::Quaterniond q_from(0, 1, 0, 0);
    // 为什么是右乘，可以动手画一下看看
//        Eigen::Quaterniond q_final = q_from;
    Eigen::Quaterniond q_final = q_from * extQRPY;
    imu_out.orientation.x = q_final.x();
    imu_out.orientation.y = q_final.y();
    imu_out.orientation.z = q_final.z();
    imu_out.orientation.w = q_final.w();
//      imu_out.orientation.x = 0;
//      imu_out.orientation.y = 0;
//      imu_out.orientation.z = 0;
//      imu_out.orientation.w = 1;

    if (sqrt(
        q_final.x() * q_final.x() + q_final.y() * q_final.y() + q_final.z() * q_final.z() + q_final.w() * q_final.w())
        < 0.1) {
      ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
      ros::shutdown();
    }

    //中值积分
//    static sensor_msgs::Imu imu_last;
//    static sensor_msgs::Imu imu_now;
//    static Eigen::Quaterniond Qwb(1, 0, 0, 0);
//    static Eigen::Vector3d Pwb(0, 0, 0);
//    static Eigen::Vector3d Vw(0, 0, 0);
//    static Eigen::Vector3d gw(0, 0, 0);
//    static Eigen::Vector3d temp_a;
//    static Eigen::Vector3d theta;
//    static bool init = false;
//    if (!init) {
//      init = true;
//      imu_now = imu_out;
//    } else {
//      imu_last = imu_now;
//      imu_now = imu_out;
//      double dt = 0.01;
//      //delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
//      Eigen::Vector3d gyro(imu_now.angular_velocity.x, imu_now.angular_velocity.y, imu_now.angular_velocity.z);
//      Eigen::Vector3d gyro_(imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z);
//      Eigen::Quaterniond dq;
//      // i时刻与i-1时刻的gyro相加后除以2,得到中值角速度
//      Eigen::Vector3d dtheta_half = 0.5 * (gyro + gyro_) * dt / 2.0;
//      dq.w() = 1;
//      dq.x() = dtheta_half.x();
//      dq.y() = dtheta_half.y();
//      dq.z() = dtheta_half.z();
//      // 得到i时刻的旋转并归一化
//      Eigen::Quaterniond Qwb_i = Qwb*dq;
//      Qwb_i.normalize();
//      // Qwb对应i-1时刻, Qwb_i对应i时刻.
//      // i时刻与i-1时刻加速度相加后除以2,得到中值加速度
//      Eigen::Vector3d acc_i(imu_now.linear_acceleration.x, imu_now.linear_acceleration.y, imu_now.linear_acceleration.z);
//      Eigen::Vector3d acc_(imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z);
//      Eigen::Vector3d acc_w = (Qwb * acc_ + Qwb_i * acc_i) * 0.5 + gw;
//      // {body}系角速度转为{world}系角速度
//      Qwb = Qwb_i ;
//      // {body}系中点在{world}系中的坐标的刷新( p_w' = p_w + v_w*t + 0.5*t*t*a_w )
//      Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
//      // {body}系中的点在{world}系中的速度刷新( v_w' = v_w + a_w*t)
//      Vw = Vw + acc_w * dt;
//      //保存数据
//      imu_ofs_ << std::fixed << ros::Time::now().toSec() << " " << Pwb(0) << " " << Pwb(1) << " " << Pwb(2)
//               << " " << Qwb.x() << " " << Qwb.y() << " " << Qwb.z() << " " << Qwb.w() << std::endl;
//    }


    return imu_out;
  }
};

/**
 * 发布thisCloud，返回thisCloud对应msg格式
*/
sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub,
                                      pcl::PointCloud<PointType>::Ptr thisCloud,
                                      ros::Time thisStamp,
                                      std::string thisFrame) {
  sensor_msgs::PointCloud2 tempCloud;
  pcl::toROSMsg(*thisCloud, tempCloud);
  tempCloud.header.stamp = thisStamp;
  tempCloud.header.frame_id = thisFrame;
  if (thisPub->getNumSubscribers() != 0)
    thisPub->publish(tempCloud);
  return tempCloud;
}

/**
 * msg时间戳
*/
template<typename T>
double ROS_TIME(T msg) {
  return msg->header.stamp.toSec();
}

/**
 * 提取imu角速度
*/
template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z) {
  *angular_x = thisImuMsg->angular_velocity.x;
  *angular_y = thisImuMsg->angular_velocity.y;
  *angular_z = thisImuMsg->angular_velocity.z;
}

/**
 * 提取imu加速度
*/
template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z) {
  *acc_x = thisImuMsg->linear_acceleration.x;
  *acc_y = thisImuMsg->linear_acceleration.y;
  *acc_z = thisImuMsg->linear_acceleration.z;
}

/**
 * 提取imu姿态角RPY
*/
template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw) {
  double imuRoll, imuPitch, imuYaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

  *rosRoll = imuRoll;
  *rosPitch = imuPitch;
  *rosYaw = imuYaw;
}

/**
 * 点到坐标系原点距离
*/
float pointDistance(PointType p) {
  return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

/**
 * 两点之间距离
*/
float pointDistance(PointType p1, PointType p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

#endif
