// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include "loam_velodyne/LaserOdometry.h"
#include "loam_velodyne/common.h"
#include "math_utils.h"
#include <pcl/filters/filter.h>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>


namespace loam {

using std::sin;
using std::cos;
using std::asin;
using std::atan2;
using std::sqrt;
using std::fabs;
using std::pow;


LaserOdometry::LaserOdometry(const float& scanPeriod,
                             const uint16_t& ioRatio,
                             const size_t& maxIterations)
      : _scanPeriod(scanPeriod),
        _ioRatio(ioRatio),
        _systemInited(false),
        _frameCount(0),
        _maxIterations(maxIterations),
        _deltaTAbort(0.1),
        _deltaRAbort(0.1),
        _cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>()),
        _cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZI>()),
        _surfPointsFlat(new pcl::PointCloud<pcl::PointXYZI>()),
        _surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZI>()),
        _laserCloud(new pcl::PointCloud<pcl::PointXYZI>()),
        _lastCornerCloud(new pcl::PointCloud<pcl::PointXYZI>()),
        _lastSurfaceCloud(new pcl::PointCloud<pcl::PointXYZI>()),
        _laserCloudOri(new pcl::PointCloud<pcl::PointXYZI>()),
        _coeffSel(new pcl::PointCloud<pcl::PointXYZI>())
{
  // initialize odometry and odometry tf messages
  _laserOdometryMsg.header.frame_id = "/camera_init";
  _laserOdometryMsg.child_frame_id = "/laser_odom";

  _laserOdometryTrans.frame_id_ = "/camera_init";
  _laserOdometryTrans.child_frame_id_ = "/laser_odom";
}



bool LaserOdometry::setup(ros::NodeHandle &node,
                          ros::NodeHandle &privateNode)
{
  // fetch laser odometry params
  float fParam;
  int iParam;

  if (privateNode.getParam("scanPeriod", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("Invalid scanPeriod parameter: %f (expected > 0)", fParam);
      return false;
    } else {
      _scanPeriod = fParam;
      ROS_INFO("Set scanPeriod: %g", fParam);
    }
  }

  if (privateNode.getParam("ioRatio", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid ioRatio parameter: %d (expected > 0)", iParam);
      return false;
    } else {
      _ioRatio = iParam;
      ROS_INFO("Set ioRatio: %d", iParam);
    }
  }

  if (privateNode.getParam("maxIterations", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid maxIterations parameter: %d (expected > 0)", iParam);
      return false;
    } else {
      _maxIterations = iParam;
      ROS_INFO("Set maxIterations: %d", iParam);
    }
  }

  if (privateNode.getParam("deltaTAbort", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("Invalid deltaTAbort parameter: %f (expected > 0)", fParam);
      return false;
    } else {
      _deltaTAbort = fParam;
      ROS_INFO("Set deltaTAbort: %g", fParam);
    }
  }

  if (privateNode.getParam("deltaRAbort", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("Invalid deltaRAbort parameter: %f (expected > 0)", fParam);
      return false;
    } else {
      _deltaRAbort = fParam;
      ROS_INFO("Set deltaRAbort: %g", fParam);
    }
  }


  // advertise laser odometry topics
  _pubLaserCloudCornerLast = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);
  _pubLaserCloudSurfLast   = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);
  _pubLaserCloudFullRes    = node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 2);
  _pubLaserOdometry        = node.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 5);


  // subscribe to scan registration topics
  _subCornerPointsSharp = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_sharp", 2, &LaserOdometry::laserCloudSharpHandler, this);

  _subCornerPointsLessSharp = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_less_sharp", 2, &LaserOdometry::laserCloudLessSharpHandler, this);

  _subSurfPointsFlat = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_flat", 2, &LaserOdometry::laserCloudFlatHandler, this);

  _subSurfPointsLessFlat = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_less_flat", 2, &LaserOdometry::laserCloudLessFlatHandler, this);

  _subLaserCloudFullRes = node.subscribe<sensor_msgs::PointCloud2>
      ("/velodyne_cloud_2", 2, &LaserOdometry::laserCloudFullResHandler, this);

  _subImuTrans = node.subscribe<sensor_msgs::PointCloud2>
      ("/imu_trans", 5, &LaserOdometry::imuTransHandler, this);

  return true;
}



void LaserOdometry::transformToStart(const pcl::PointXYZI& pi, pcl::PointXYZI& po)
{
  float s = 10 * (pi.intensity - int(pi.intensity));

  po.x = pi.x - s * _transform.pos.x();
  po.y = pi.y - s * _transform.pos.y();
  po.z = pi.z - s * _transform.pos.z();
  po.intensity = pi.intensity;

  Angle rx = -s * _transform.rot_x.rad();
  Angle ry = -s * _transform.rot_y.rad();
  Angle rz = -s * _transform.rot_z.rad();
  rotateZXY(po, rz, rx, ry);
}



size_t LaserOdometry::transformToEnd(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  size_t cloudSize = cloud->points.size();

  for (size_t i = 0; i < cloudSize; i++) {
    pcl::PointXYZI& point = cloud->points[i];

    float s = 10 * (point.intensity - int(point.intensity));

    point.x -= s * _transform.pos.x();
    point.y -= s * _transform.pos.y();
    point.z -= s * _transform.pos.z();
    point.intensity = int(point.intensity);

    Angle rx = -s * _transform.rot_x.rad();
    Angle ry = -s * _transform.rot_y.rad();
    Angle rz = -s * _transform.rot_z.rad();
    rotateZXY(point, rz, rx, ry);
    rotateYXZ(point, _transform.rot_y, _transform.rot_x, _transform.rot_z);

    point.x += _transform.pos.x() - _imuShiftFromStart.x();
    point.y += _transform.pos.y() - _imuShiftFromStart.y();
    point.z += _transform.pos.z() - _imuShiftFromStart.z();

    rotateZXY(point, _imuRollStart, _imuPitchStart, _imuYawStart);
    rotateYXZ(point, -_imuYawEnd, -_imuPitchEnd, -_imuRollEnd);
  }

  return cloudSize;
}



void LaserOdometry::pluginIMURotation(const Angle& bcx, const Angle& bcy, const Angle& bcz,
                                      const Angle& blx, const Angle& bly, const Angle& blz,
                                      const Angle& alx, const Angle& aly, const Angle& alz,
                                      Angle &acx, Angle &acy, Angle &acz)
{
  float sbcx = bcx.sin();
  float cbcx = bcx.cos();
  float sbcy = bcy.sin();
  float cbcy = bcy.cos();
  float sbcz = bcz.sin();
  float cbcz = bcz.cos();

  float sblx = blx.sin();
  float cblx = blx.cos();
  float sbly = bly.sin();
  float cbly = bly.cos();
  float sblz = blz.sin();
  float cblz = blz.cos();

  float salx = alx.sin();
  float calx = alx.cos();
  float saly = aly.sin();
  float caly = aly.cos();
  float salz = alz.sin();
  float calz = alz.cos();

  float srx = -sbcx*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly)
            - cbcx*cbcz*(calx*saly*(cbly*sblz - cblz*sblx*sbly)
                         - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx)
            - cbcx*sbcz*(calx*caly*(cblz*sbly - cbly*sblx*sblz)
                         - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz);
  acx = -asin(srx);

  float srycrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz*sblx*sbly)
                                               - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx)
                 - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz)
                                                 - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz)
                 + cbcx*sbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
  float crycrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz)
                                               - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz)
                 - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz*sblx*sbly)
                                                 - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx)
                 + cbcx*cbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
  acy = atan2(srycrx / acx.cos(), crycrx / acx.cos());

  float srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz) - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx)
                 - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly)
                              + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx)
                              - calx*cblx*cblz*salz)
                 + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
                              + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz)
                              + calx*cblx*salz*sblz);
  float crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly) - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx)
                 + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                              + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly)
                              + calx*calz*cblx*cblz)
                 - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly - cbly*sblx*sblz)
                              + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz)
                              - calx*calz*cblx*sblz);
  acz = atan2(srzcrx / acx.cos(), crzcrx / acx.cos());
}



void LaserOdometry::accumulateRotation(Angle cx, Angle cy, Angle cz,
                                       Angle lx, Angle ly, Angle lz,
                                       Angle &ox, Angle &oy, Angle &oz)
{
  float srx = lx.cos()*cx.cos()*ly.sin()*cz.sin()
            - cx.cos()*cz.cos()*lx.sin()
            - lx.cos()*ly.cos()*cx.sin();
  ox = -asin(srx);

  float srycrx = lx.sin()*(cy.cos()*cz.sin() - cz.cos()*cx.sin()*cy.sin())
               + lx.cos()*ly.sin()*(cy.cos()*cz.cos() + cx.sin()*cy.sin()*cz.sin())
               + lx.cos()*ly.cos()*cx.cos()*cy.sin();
  float crycrx = lx.cos()*ly.cos()*cx.cos()*cy.cos()
               - lx.cos()*ly.sin()*(cz.cos()*cy.sin() - cy.cos()*cx.sin()*cz.sin())
               - lx.sin()*(cy.sin()*cz.sin() + cy.cos()*cz.cos()*cx.sin());
  oy = atan2(srycrx / ox.cos(), crycrx / ox.cos());

  float srzcrx = cx.sin()*(lz.cos()*ly.sin() - ly.cos()*lx.sin()*lz.sin())
               + cx.cos()*cz.sin()*(ly.cos()*lz.cos() + lx.sin()*ly.sin()*lz.sin())
               + lx.cos()*cx.cos()*cz.cos()*lz.sin();
  float crzcrx = lx.cos()*lz.cos()*cx.cos()*cz.cos()
               - cx.cos()*cz.sin()*(ly.cos()*lz.sin() - lz.cos()*lx.sin()*ly.sin())
               - cx.sin()*(ly.sin()*lz.sin() + ly.cos()*lz.cos()*lx.sin());
  oz = atan2(srzcrx / ox.cos(), crzcrx / ox.cos());
}



void LaserOdometry::laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsSharpMsg)
{
  _timeCornerPointsSharp = cornerPointsSharpMsg->header.stamp;

  _cornerPointsSharp->clear();
  pcl::fromROSMsg(*cornerPointsSharpMsg, *_cornerPointsSharp);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_cornerPointsSharp, *_cornerPointsSharp, indices);
  _newCornerPointsSharp = true;
}



void LaserOdometry::laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLessSharpMsg)
{
  _timeCornerPointsLessSharp = cornerPointsLessSharpMsg->header.stamp;

  _cornerPointsLessSharp->clear();
  pcl::fromROSMsg(*cornerPointsLessSharpMsg, *_cornerPointsLessSharp);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_cornerPointsLessSharp, *_cornerPointsLessSharp, indices);
  _newCornerPointsLessSharp = true;
}



void LaserOdometry::laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsFlatMsg)
{
  _timeSurfPointsFlat = surfPointsFlatMsg->header.stamp;

  _surfPointsFlat->clear();
  pcl::fromROSMsg(*surfPointsFlatMsg, *_surfPointsFlat);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_surfPointsFlat, *_surfPointsFlat, indices);
  _newSurfPointsFlat = true;
}



void LaserOdometry::laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsLessFlatMsg)
{
  _timeSurfPointsLessFlat = surfPointsLessFlatMsg->header.stamp;

  _surfPointsLessFlat->clear();
  pcl::fromROSMsg(*surfPointsLessFlatMsg, *_surfPointsLessFlat);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_surfPointsLessFlat, *_surfPointsLessFlat, indices);
  _newSurfPointsLessFlat = true;
}



void LaserOdometry::laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg)
{
  _timeLaserCloudFullRes = laserCloudFullResMsg->header.stamp;

  _laserCloud->clear();
  pcl::fromROSMsg(*laserCloudFullResMsg, *_laserCloud);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_laserCloud, *_laserCloud, indices);
  _newLaserCloudFullRes = true;
}



void LaserOdometry::imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTransMsg)
{
  _timeImuTrans = imuTransMsg->header.stamp;

  pcl::PointCloud<pcl::PointXYZ> imuTrans;
  pcl::fromROSMsg(*imuTransMsg, imuTrans);

  _imuPitchStart = imuTrans.points[0].x;
  _imuYawStart = imuTrans.points[0].y;
  _imuRollStart = imuTrans.points[0].z;

  _imuPitchEnd = imuTrans.points[1].x;
  _imuYawEnd = imuTrans.points[1].y;
  _imuRollEnd = imuTrans.points[1].z;

  _imuShiftFromStart = imuTrans.points[2];
  _imuVeloFromStart = imuTrans.points[3];

  _newImuTrans = true;
}



void LaserOdometry::spin()
{
  ros::Rate rate(100);
  bool status = ros::ok();

  // loop until shutdown
  while (status) {
    ros::spinOnce();

    // try processing new data
    process();

    status = ros::ok();
    rate.sleep();
  }
}



void LaserOdometry::reset()
{
  _newCornerPointsSharp = false;
  _newCornerPointsLessSharp = false;
  _newSurfPointsFlat = false;
  _newSurfPointsLessFlat = false;
  _newLaserCloudFullRes = false;
  _newImuTrans = false;
}



bool LaserOdometry::hasNewData()
{
  return _newCornerPointsSharp && _newCornerPointsLessSharp && _newSurfPointsFlat &&
         _newSurfPointsLessFlat && _newLaserCloudFullRes && _newImuTrans &&
         fabs((_timeCornerPointsSharp - _timeSurfPointsLessFlat).toSec()) < 0.005 &&
         fabs((_timeCornerPointsLessSharp - _timeSurfPointsLessFlat).toSec()) < 0.005 &&
         fabs((_timeSurfPointsFlat - _timeSurfPointsLessFlat).toSec()) < 0.005 &&
         fabs((_timeLaserCloudFullRes - _timeSurfPointsLessFlat).toSec()) < 0.005 &&
         fabs((_timeImuTrans - _timeSurfPointsLessFlat).toSec()) < 0.005;
}



void LaserOdometry::process()
{//判断数据是否到来
  if (!hasNewData()) {
    // waiting for new data to arrive...
    return;
  }

  // reset flags, etc.初始化topic是否到来的flag
  reset();

  if (!_systemInited){ // 将订阅的数据保存为上一时刻的数据
    _cornerPointsLessSharp.swap(_lastCornerCloud);
    _surfPointsLessFlat.swap(_lastSurfaceCloud);
//将边特征，平面特征点云存到kdtree
    _lastCornerKDTree.setInputCloud(_lastCornerCloud);
    _lastSurfaceKDTree.setInputCloud(_lastSurfaceCloud);

    _transformSum.rot_x += _imuPitchStart;
    _transformSum.rot_z += _imuRollStart;

    _systemInited = true;
    return;
  }

  pcl::PointXYZI coeff;//雅克比矩阵值
  bool isDegenerate = false;
  Eigen::Matrix<float,6,6> matP;

  _frameCount++;
  _transform.pos -= _imuVeloFromStart * _scanPeriod;


  size_t lastCornerCloudSize = _lastCornerCloud->points.size();
  size_t lastSurfaceCloudSize = _lastSurfaceCloud->points.size();
// 上一时刻特征边(曲率大)上的点云个数大于10， 特征面内的点云大于100 保证足够多的特征点可用于t+1时刻的匹配
  if (lastCornerCloudSize > 10 && lastSurfaceCloudSize > 100) {
    std::vector<int> pointSearchInd(1);
    std::vector<float> pointSearchSqDis(1);
    std::vector<int> indices;

    pcl::removeNaNFromPointCloud(*_cornerPointsSharp, *_cornerPointsSharp, indices);//剔除一些异常点
    size_t cornerPointsSharpNum = _cornerPointsSharp->points.size();//边特征个数
    size_t surfPointsFlatNum = _surfPointsFlat->points.size();//平面特征个数

    _pointSearchCornerInd1.resize(cornerPointsSharpNum);
    _pointSearchCornerInd2.resize(cornerPointsSharpNum);
    _pointSearchSurfInd1.resize(surfPointsFlatNum);
    _pointSearchSurfInd2.resize(surfPointsFlatNum);
    _pointSearchSurfInd3.resize(surfPointsFlatNum);
//接下来开始迭代配准
    for (size_t iterCount = 0; iterCount < _maxIterations; iterCount++) {
      pcl::PointXYZI pointSel, pointProj, tripod1, tripod2, tripod3;
      _laserCloudOri->clear();
      _coeffSel->clear();//两时刻点云配准协方差

      for (int i = 0; i < cornerPointsSharpNum; i++) {
        transformToStart(_cornerPointsSharp->points[i], pointSel);// 将点坐标转换到起始点云坐标系中

        if (iterCount % 5 == 0) {//降低匹配次数，每隔5次迭代一次
          pcl::removeNaNFromPointCloud(*_lastCornerCloud, *_lastCornerCloud, indices);
          _lastCornerKDTree.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
// 找到pointSel(当前时刻边特征中的某一点)在laserCloudCornerLast中的1个最邻近点返回pointSearchInd(点对应的索引)  pointSearchSqDis(pointSel与对应点的欧氏距离)
          int closestPointInd = -1, minPointInd2 = -1;
          if (pointSearchSqDis[0] < 25) {//两桢之间的变换不会太剧烈，25作为阈值
            closestPointInd = pointSearchInd[0];
            int closestPointScan = int(_lastCornerCloud->points[closestPointInd].intensity);//pointSel对应上一桢的那一线？closestPointScan
//找到closestPointInd后找次近邻点的索引,作者没有直接nearKSearch找最近的两个点，也为了考虑这两个点要有效的吧，下面是不同约束，不能差3线以上，不能距离过大。
            float pointSqDis, minPointSqDis2 = 25;
            for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
              if (int(_lastCornerCloud->points[j].intensity) > closestPointScan + 2.5) {// 找到与最邻近点相距3条线的特征点时跳出
                break;
              }

              pointSqDis = calcSquaredDiff(_lastCornerCloud->points[j], pointSel);//直接计算，没有用nearksearch查找

              if (int(_lastCornerCloud->points[j].intensity) > closestPointScan) {
                if (pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              }
            }
            for (int j = closestPointInd - 1; j >= 0; j--) {// 向下三条线，找次临近点
              if (int(_lastCornerCloud->points[j].intensity) < closestPointScan - 2.5) {
                break;
              }

              pointSqDis = calcSquaredDiff(_lastCornerCloud->points[j], pointSel);

              if (int(_lastCornerCloud->points[j].intensity) < closestPointScan) {
                if (pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              }
            }
          }

          _pointSearchCornerInd1[i] = closestPointInd;// 当前所有边特征点在上一时刻边特征点云中对应的最邻近点的索引
          _pointSearchCornerInd2[i] = minPointInd2;// 当前所有边特征点在上一时刻边特征点云中对应的次邻近点的索引
            }
        }
	/*Jaccobian*/
        if (_pointSearchCornerInd2[i] >= 0) {
          const pcl::PointXYZI  &A = _lastCornerCloud->points[_pointSearchCornerInd1[i]];
          const pcl::PointXYZI  &B = _lastCornerCloud->points[_pointSearchCornerInd2[i]];
          pcl::PointXYZI coefficients;
          if(getCornerFeatureCoefficients(A, B, pointSel, iterCount, coefficients)) {
            _laserCloudOri->push_back(_cornerPointsSharp->points[i]);
            _coeffSel->push_back(coefficients);//求得的雅克比矩阵
          }
        }
      }
//面特征采用类似思想，找最近邻，次近邻点。求雅克比矩阵值
      for (int i = 0; i < surfPointsFlatNum; i++) {
        transformToStart(_surfPointsFlat->points[i], pointSel);

        if (iterCount % 5 == 0) {
          _lastSurfaceKDTree.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
          int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
          if (pointSearchSqDis[0] < 25) {
            closestPointInd = pointSearchInd[0];
            int closestPointScan = int(_lastSurfaceCloud->points[closestPointInd].intensity);

            float pointSqDis, minPointSqDis2 = 25, minPointSqDis3 = 25;
            for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
              if (int(_lastSurfaceCloud->points[j].intensity) > closestPointScan + 2.5) {
                break;
              }

              pointSqDis = calcSquaredDiff(_lastSurfaceCloud->points[j], pointSel);

              if (int(_lastSurfaceCloud->points[j].intensity) <= closestPointScan) {
                if (pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              } else {
                if (pointSqDis < minPointSqDis3) {
                  minPointSqDis3 = pointSqDis;
                  minPointInd3 = j;
                }
              }
            }
            for (int j = closestPointInd - 1; j >= 0; j--) {
              if (int(_lastSurfaceCloud->points[j].intensity) < closestPointScan - 2.5) {
                break;
              }

              pointSqDis = calcSquaredDiff(_lastSurfaceCloud->points[j], pointSel);

              if (int(_lastSurfaceCloud->points[j].intensity) >= closestPointScan) {
                if (pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              } else {
                if (pointSqDis < minPointSqDis3) {
                  minPointSqDis3 = pointSqDis;
                  minPointInd3 = j;
                }
              }
            }
          }

          _pointSearchSurfInd1[i] = closestPointInd;
          _pointSearchSurfInd2[i] = minPointInd2;
          _pointSearchSurfInd3[i] = minPointInd3;
        }
	/*Jaccobian*/
        if (_pointSearchSurfInd2[i] >= 0 && _pointSearchSurfInd3[i] >= 0) {
          const pcl::PointXYZI &A = _lastSurfaceCloud->points[_pointSearchSurfInd1[i]];
          const pcl::PointXYZI &B = _lastSurfaceCloud->points[_pointSearchSurfInd2[i]];
          const pcl::PointXYZI &C = _lastSurfaceCloud->points[_pointSearchSurfInd3[i]];

          pcl::PointXYZI coefficients;
          if(getSurfaceFeatureCoefficients(A, B, C, pointSel, iterCount, coefficients)) {
            _laserCloudOri->push_back(_surfPointsFlat->points[i]);
            _coeffSel->push_back(coefficients);
          }
        }      
      }
// 匹配到的点的个数(即存在多少个约束),小于10则继续寻找 
      int pointSelNum = _laserCloudOri->points.size();
      if (pointSelNum < 10) {
        continue;
      }
//LM进行运动估计，matA是雅克比矩阵，matAt*matA*matX = matAt*matB;其中matX是步长,（roll , ptich ,yaw,x,y,z）
      Eigen::Matrix<float,Eigen::Dynamic,6> matA(pointSelNum, 6);
      Eigen::Matrix<float,6,Eigen::Dynamic> matAt(6,pointSelNum);
      Eigen::Matrix<float,6,6> matAtA;
      Eigen::VectorXf matB(pointSelNum);
      Eigen::Matrix<float,6,1> matAtB;
      Eigen::Matrix<float,6,1> matX;

      for (int i = 0; i < pointSelNum; i++) {
        const pcl::PointXYZI& pointOri = _laserCloudOri->points[i];
        coeff = _coeffSel->points[i];

        float s = 1;

        float srx = sin(s * _transform.rot_x.rad());
        float crx = cos(s * _transform.rot_x.rad());
        float sry = sin(s * _transform.rot_y.rad());
        float cry = cos(s * _transform.rot_y.rad());
        float srz = sin(s * _transform.rot_z.rad());
        float crz = cos(s * _transform.rot_z.rad());
        float tx = s * _transform.pos.x();
        float ty = s * _transform.pos.y();
        float tz = s * _transform.pos.z();

        float arx = (-s*crx*sry*srz*pointOri.x + s*crx*crz*sry*pointOri.y + s*srx*sry*pointOri.z
                     + s*tx*crx*sry*srz - s*ty*crx*crz*sry - s*tz*srx*sry) * coeff.x
                    + (s*srx*srz*pointOri.x - s*crz*srx*pointOri.y + s*crx*pointOri.z
                       + s*ty*crz*srx - s*tz*crx - s*tx*srx*srz) * coeff.y
                    + (s*crx*cry*srz*pointOri.x - s*crx*cry*crz*pointOri.y - s*cry*srx*pointOri.z
                       + s*tz*cry*srx + s*ty*crx*cry*crz - s*tx*crx*cry*srz) * coeff.z;

        float ary = ((-s*crz*sry - s*cry*srx*srz)*pointOri.x
                     + (s*cry*crz*srx - s*sry*srz)*pointOri.y - s*crx*cry*pointOri.z
                     + tx*(s*crz*sry + s*cry*srx*srz) + ty*(s*sry*srz - s*cry*crz*srx)
                     + s*tz*crx*cry) * coeff.x
                    + ((s*cry*crz - s*srx*sry*srz)*pointOri.x
                       + (s*cry*srz + s*crz*srx*sry)*pointOri.y - s*crx*sry*pointOri.z
                       + s*tz*crx*sry - ty*(s*cry*srz + s*crz*srx*sry)
                       - tx*(s*cry*crz - s*srx*sry*srz)) * coeff.z;

        float arz = ((-s*cry*srz - s*crz*srx*sry)*pointOri.x + (s*cry*crz - s*srx*sry*srz)*pointOri.y
                     + tx*(s*cry*srz + s*crz*srx*sry) - ty*(s*cry*crz - s*srx*sry*srz)) * coeff.x
                    + (-s*crx*crz*pointOri.x - s*crx*srz*pointOri.y
                       + s*ty*crx*srz + s*tx*crx*crz) * coeff.y
                    + ((s*cry*crz*srx - s*sry*srz)*pointOri.x + (s*crz*sry + s*cry*srx*srz)*pointOri.y
                       + tx*(s*sry*srz - s*cry*crz*srx) - ty*(s*crz*sry + s*cry*srx*srz)) * coeff.z;

        float atx = -s*(cry*crz - srx*sry*srz) * coeff.x + s*crx*srz * coeff.y
                    - s*(crz*sry + cry*srx*srz) * coeff.z;

        float aty = -s*(cry*srz + crz*srx*sry) * coeff.x - s*crx*crz * coeff.y
                    - s*(sry*srz - cry*crz*srx) * coeff.z;

        float atz = s*crx*sry * coeff.x - s*srx * coeff.y - s*crx*cry * coeff.z;

        float d2 = coeff.intensity;

        matA(i, 0) = arx;
        matA(i, 1) = ary;
        matA(i, 2) = arz;
        matA(i, 3) = atx;
        matA(i, 4) = aty;
        matA(i, 5) = atz;
        matB(i, 0) = -0.05 * d2;
      }
      //QR
      matAt = matA.transpose();
      matAtA = matAt * matA;
      matAtB = matAt * matB;

      matX = matAtA.colPivHouseholderQr().solve(matAtB);

      if (iterCount == 0) {
        Eigen::Matrix<float,1,6> matE;
        Eigen::Matrix<float,6,6> matV;
        Eigen::Matrix<float,6,6> matV2;

        Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float,6, 6> > esolver(matAtA);
        matE = esolver.eigenvalues().real();
        matV = esolver.eigenvectors().real();

        matV2 = matV;

        isDegenerate = false;
        float eignThre[6] = {10, 10, 10, 10, 10, 10};
        for (int i = 5; i >= 0; i--) {
          if (matE(0, i) < eignThre[i]) {
            for (int j = 0; j < 6; j++) {
              matV2(i, j) = 0;
            }
            isDegenerate = true;
          } else {
            break;
          }
        }
        matP = matV.inverse() * matV2;
      }

      if (isDegenerate) {
        Eigen::Matrix<float,6,1> matX2;
        matX2 = matX;
        matX = matP * matX2;
      }

      _transform.rot_x = _transform.rot_x.rad() + matX(0, 0);
      _transform.rot_y = _transform.rot_y.rad() + matX(1, 0);
      _transform.rot_z = _transform.rot_z.rad() + matX(2, 0);
      _transform.pos.x() += matX(3, 0);
      _transform.pos.y() += matX(4, 0);
      _transform.pos.z() += matX(5, 0);

      if( !pcl_isfinite(_transform.rot_x.rad()) ) _transform.rot_x = Angle();
      if( !pcl_isfinite(_transform.rot_y.rad()) ) _transform.rot_y = Angle();
      if( !pcl_isfinite(_transform.rot_z.rad()) ) _transform.rot_z = Angle();

      if( !pcl_isfinite(_transform.pos.x()) ) _transform.pos.x() = 0.0;
      if( !pcl_isfinite(_transform.pos.y()) ) _transform.pos.y() = 0.0;
      if( !pcl_isfinite(_transform.pos.z()) ) _transform.pos.z() = 0.0;

      float deltaR = sqrt(pow(rad2deg(matX(0, 0)), 2) +
                          pow(rad2deg(matX(1, 0)), 2) +
                          pow(rad2deg(matX(2, 0)), 2));
      float deltaT = sqrt(pow(matX(3, 0) * 100, 2) +
                          pow(matX(4, 0) * 100, 2) +
                          pow(matX(5, 0) * 100, 2));

      if (deltaR < _deltaRAbort && deltaT < _deltaTAbort) {
        break;
      }
    }
  }
/*Transformation **/
  Angle rx, ry, rz;
  accumulateRotation(_transformSum.rot_x,
                     _transformSum.rot_y,
                     _transformSum.rot_z,
                     -_transform.rot_x,
                     -_transform.rot_y.rad() * 1.05,
                     -_transform.rot_z,
                     rx, ry, rz);

  Vector3 v( _transform.pos.x()        - _imuShiftFromStart.x(),
             _transform.pos.y()        - _imuShiftFromStart.y(),
             _transform.pos.z() * 1.05 - _imuShiftFromStart.z() );
  rotateZXY(v, rz, rx, ry);
  Vector3 trans = _transformSum.pos - v;

  pluginIMURotation(rx, ry, rz,
                    _imuPitchStart, _imuYawStart, _imuRollStart,
                    _imuPitchEnd, _imuYawEnd, _imuRollEnd,
                    rx, ry, rz);

  _transformSum.rot_x = rx;
  _transformSum.rot_y = ry;
  _transformSum.rot_z = rz;
  _transformSum.pos = trans;

  transformToEnd(_cornerPointsLessSharp);
  transformToEnd(_surfPointsLessFlat);

  _cornerPointsLessSharp.swap(_lastCornerCloud);
  _surfPointsLessFlat.swap(_lastSurfaceCloud);

  lastCornerCloudSize = _lastCornerCloud->points.size();
  lastSurfaceCloudSize = _lastSurfaceCloud->points.size();

  if (lastCornerCloudSize > 10 && lastSurfaceCloudSize > 100) {
    _lastCornerKDTree.setInputCloud(_lastCornerCloud);
    _lastSurfaceKDTree.setInputCloud(_lastSurfaceCloud);
  }

  publishResult();
}



void LaserOdometry::publishResult()
{
  // publish odometry tranformations
  geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(_transformSum.rot_z.rad(),
                                                                              -_transformSum.rot_x.rad(),
                                                                              -_transformSum.rot_y.rad());

  _laserOdometryMsg.header.stamp = _timeSurfPointsLessFlat;
  _laserOdometryMsg.pose.pose.orientation.x = -geoQuat.y;
  _laserOdometryMsg.pose.pose.orientation.y = -geoQuat.z;
  _laserOdometryMsg.pose.pose.orientation.z = geoQuat.x;
  _laserOdometryMsg.pose.pose.orientation.w = geoQuat.w;
  _laserOdometryMsg.pose.pose.position.x = _transformSum.pos.x();
  _laserOdometryMsg.pose.pose.position.y = _transformSum.pos.y();
  _laserOdometryMsg.pose.pose.position.z = _transformSum.pos.z();
  _pubLaserOdometry.publish(_laserOdometryMsg);

  _laserOdometryTrans.stamp_ = _timeSurfPointsLessFlat;
  _laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  _laserOdometryTrans.setOrigin(tf::Vector3( _transformSum.pos.x(), _transformSum.pos.y(), _transformSum.pos.z()) );
  _tfBroadcaster.sendTransform(_laserOdometryTrans);


  // publish cloud results according to the input output ratio
  if (_ioRatio < 2 || _frameCount % _ioRatio == 1) {
    ros::Time sweepTime = _timeSurfPointsLessFlat;
    transformToEnd(_laserCloud);  // transform full resolution cloud to sweep end before sending it

    publishCloudMsg(_pubLaserCloudCornerLast, *_lastCornerCloud, sweepTime, "/camera");
    publishCloudMsg(_pubLaserCloudSurfLast, *_lastSurfaceCloud, sweepTime, "/camera");
    publishCloudMsg(_pubLaserCloudFullRes, *_laserCloud, sweepTime, "/camera");
  }
}

} // end namespace loam
