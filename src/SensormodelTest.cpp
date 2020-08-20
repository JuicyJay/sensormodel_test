/*
 * SensormodelTest.h
 *
 *  Created on: Aug 20, 2020
 *      Author: jasmin
 */

#include "SensormodelTest.h"
#include <iostream>

SensormodelTest::SensormodelTest(const float dimX, const float dimY, const float dimZ, const float cellSize)
    : _dimX(dimX), _dimY(dimY), _dimZ(dimZ), _cellSize(cellSize), _cellsX(0), _cellsY(0), _cellsZ(0)
{
  std::cout << "constructor" << std::endl;
}

SensormodelTest::~SensormodelTest() {}

void SensormodelTest::callbackPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  // if neither sensor nor space are initialized yet -> do so in init routine
  if(!_sensor || !_space)
    this->init(cloud);

  /////////////////////////////////////////////////////
  // den ganzen tf listener Kram lass ich erstmal weg//
  ////////////////////////////////////////////////////

  obvious::Matrix TransMat(4, 4);
  TransMat.setIdentity();
  obvious::obfloat center[3];
  _space->getCentroid(center);
  tf::Vector3 tfVec(0.0, 0.0, 0.0);
}
