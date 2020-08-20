/*
 * main.cpp
 *
 *  Created on: Aug 20, 2020
 *      Author: jasmin
 */

#include "SensormodelTest.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensormodelTest_node");
  SensormodelTest model(10.0, 10.0, 10.0, 0.025);
  ros::spin();
}