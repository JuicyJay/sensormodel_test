/*
 * SensormodelTest.h
 *
 *  Created on: Aug 20, 2020
 *      Author: jasmin
 */

class SensormodelTest
{
public:
  SensormodelTest(const float dimX, const float dimY, const float dimZ, const float cellSize);
  virtual ~SensormodelTest();
  void callbackPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);

private:
  float        _dimX;
  float        _dimY;
  float        _dimZ;
  float        _cellSize;
  unsigned int _cellsX;
  unsigned int _cellsY;
  unsigned int _cellsZ;
};