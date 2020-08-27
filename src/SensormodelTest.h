/*
 * SensormodelTest.h
 *
 *  Created on: Aug 20, 2020
 *      Author: jasmin
 */

#include "obvision/reconstruct/space/SensorVelodyne3DNew.h"
#include "obvision/reconstruct/space/TsdSpace.h"
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf/tf.h>

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > stdVecEig3d;

class SensormodelTest
{
public:
  SensormodelTest(const float dimX, const float dimY, const float dimZ, const float cellSize);
  virtual ~SensormodelTest();
  void callbackPointcloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  bool pubAxisAlignedRaycaster(void);

private:
  void init(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  void redBlueRenderSpace(pcl::PointCloud<pcl::PointXYZRGB>& cloud);

  std::unique_ptr<obvious::TsdSpace>            _space;
  std::unique_ptr<obvious::SensorVelodyne3DNew> _sensor;
  std::string                                   _tfBaseFrame;

  ros::Subscriber _subPointcloud;
  ros::NodeHandle _nh;
  ros::Publisher  _pubAxisAlignedCloud;
  ros::Publisher  _pubRedBlueRendered;

  float        _dimX;
  float        _dimY;
  float        _dimZ;
  float        _cellSize;
  unsigned int _cellsX;
  unsigned int _cellsY;
  unsigned int _cellsZ;
};