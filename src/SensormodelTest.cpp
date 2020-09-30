/*
 * SensormodelTest.h
 *
 *  Created on: Aug 20, 2020
 *      Author: jasmin
 */

#include "SensormodelTest.h"
#include "obcore/math/mathbase.h"
#include "obvision/reconstruct/space/RayCast3D.h"
#include "obvision/reconstruct/space/RayCastAxisAligned3D.h"
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>

SensormodelTest::SensormodelTest(const float dimX, const float dimY, const float dimZ, const float cellSize)
    : _dimX(dimX), _dimY(dimY), _dimZ(dimZ), _cellSize(cellSize), _listener(std::make_unique<tf::TransformListener>()), _cellsX(0), _cellsY(0), _cellsZ(0)
{
  std::cout << "Constructor. Hey gorgeous, you can do this." << std::endl;
  _subPointcloud = _nh.subscribe("transformed_cloud", 1, &SensormodelTest::callbackPointcloud, this); // bagfile chamber
  // _subPointcloud = _nh.subscribe("puck_rear/velodyne_points", 1, &SensormodelTest::callbackPointcloud, this); // office bagfile, georg bags
  // _subPointcloud =
  //     _nh.subscribe("horizontal/velodyne_points", 1, &SensormodelTest::callbackPointcloud, this); // bagfile gazebo_puck_tf_lehrpfad.bag
  //  _subPointcloud         = _nh.subscribe("velodyne_points", 1, &SensormodelTest::callbackPointcloud, this);            //
  // bagfile organizePCL_Halle.bag
  _pubAxisAlignedCloud   = _nh.advertise<pcl::PointCloud<pcl::PointXYZRGBNormal> >("axisAlignedCloud", 1);
  _pubRedBlueRendered    = _nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("redBlueRendered_space", 1);
  _pubSensorRaycastCloud = _nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("sensorRaycastCloud", 1);
  _tfBaseFrame           = "map"; // changed to map for artificial box data from cloud_factory
  _virginPush            = false;
  _withTf                = false; // if this is false, only mapping of first cloud without tf
}

SensormodelTest::~SensormodelTest() {}

void SensormodelTest::init(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  std::cout << "init Yeah! Keep going!" << std::endl;
  if(_sensor || _space)
    return;

  // 1. INITIALIZE SPACE
  unsigned int cellsX = static_cast<unsigned int>(std::round(_dimX / _cellSize));
  unsigned int cellsY = static_cast<unsigned int>(std::round(_dimY / _cellSize));
  unsigned int cellsZ = static_cast<unsigned int>(std::round(_dimZ / _cellSize));

  const unsigned int xFac = cellsX / 64;
  const unsigned int yFac = cellsY / 64;
  const unsigned int zFac = cellsZ / 64;

  _cellsX = xFac * 64;
  _cellsY = yFac * 64;
  _cellsZ = zFac * 64;

  std::cout << "_cellsX = " << _cellsX << ", _cellsY = " << _cellsY << ", cellsZ = " << _cellsZ << std::endl;

  _space = std::make_unique<obvious::TsdSpace>(_cellSize, obvious::LAYOUT_64x64x64, _cellsX, _cellsY, _cellsZ);
  _space->setMaxTruncation(3.0 * _cellSize);
  // initial sensor pose: middle of space
  double tr[3];
  _space->getCentroid(tr);
  double          tf[16] = {1, 0, 0, tr[0], 0, 1, 0, tr[1], 0, 0, 1, tr[2], 0, 0, 0, 1};
  obvious::Matrix Tinit(4, 4);
  Tinit.setIdentity();
  Tinit.setData(tf);

  // 2. INITIALIZE SENSOR
  unsigned int raysIncl = 16;
  double       inclMin  = obvious::deg2rad(-15.0);
  double       inclMax  = obvious::deg2rad(15.0);
  double       inclRes  = obvious::deg2rad(2.0);
  double       azimMin  = obvious::deg2rad(0.0);
  double       azimMax  = obvious::deg2rad(360.0);
  double       azimRes  = obvious::deg2rad(0.2); // CAREFUL this should be 0.2, just 4 debugging
  _sensor               = std::make_unique<obvious::SensorVelodyne3DNew>(raysIncl, inclMin, inclMax, inclRes, azimMin, azimMax, azimRes);

  _sensor->setTransformation(Tinit);
  std::cout << "You go girl! Sensor pose after transforming to the middle of TSD space: " << std::endl;
  _sensor->getTransformation().print();
}

bool SensormodelTest::pubAxisAlignedRaycaster(void)
{
  static obvious::obfloat*      coords  = new obvious::obfloat[_cellsX * _cellsY * _cellsZ * 3];
  static obvious::obfloat*      normals = new obvious::obfloat[_cellsX * _cellsY * _cellsZ * 3];
  static unsigned char*         rgb     = new unsigned char[_cellsX * _cellsY * _cellsZ * 3];
  static unsigned int           seq     = 0;
  obvious::RayCastAxisAligned3D raycasterAxisAligned;
  unsigned int                  cnt = 0;

  raycasterAxisAligned.calcCoords(_space.get(), coords, normals, rgb, &cnt);
  if(cnt == 0)
    return false;

  pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
  obvious::obfloat                        tr[3];
  _space->getCentroid(tr);

  for(unsigned int i = 0; i < cnt; i += 3)
  {
    pcl::PointXYZRGBNormal p;
    p.x = coords[i] - tr[0];
    p.y = coords[i + 1] - tr[1];
    p.z = coords[i + 2] - tr[2];
    // p.x = -p.x; // warum macht er das?

    p.normal_x = normals[i] - tr[0];
    p.normal_y = normals[i + 1] - tr[1];
    p.normal_z = normals[i + 2] - tr[2];

    p.r = rgb[i];
    p.g = rgb[i + 1];
    p.b = rgb[i + 2];

    cloud.push_back(p);
  }
  // transform pointcloud
  //   Eigen::Affine3d rot  = Eigen::Affine3f::Identity();
  //   Eigen::Affine3f rotY = Eigen::Affine3f::Identity();
  //   Eigen::Affine3f rotX = Eigen::Affine3f::Identity();
  //   rotY.rotate(Eigen::AngleAxisf(M_PI / 2.0f, Eigen::Vector3f::UnityY()));
  //   rotX.rotate(Eigen::AngleAxisf(M_PI / 2.0f, Eigen::Vector3f::UnitX()));
  //   rot = rotX * rotY;
  //   pcl::transformPointCloud(cloud, cloud, rot);

  cloud.header.frame_id = _tfBaseFrame;
  cloud.header.seq      = seq++;
  std::cout << __PRETTY_FUNCTION__ << " publishing " << cloud.size() << " points" << std::endl;
  _pubAxisAlignedCloud.publish(cloud);
  return true;
}

void SensormodelTest::redBlueRenderSpace(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
  static unsigned int seq = 0;
  struct Color
  {
    Color(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}
    Color() : r(0), g(0), b(0) {}
    void    red(uint8_t val) { r = val; }
    void    blue(uint8_t val) { b = val; }
    uint8_t r;
    uint8_t g;
    uint8_t b;
  };

  obvious::Matrix*   cellCoordsHom = obvious::TsdSpacePartition::getCellCoordsHom();
  obvious::Matrix*   partCoords    = obvious::TsdSpacePartition::getPartitionCoords();
  unsigned int       partSize      = (_space->getPartitions())[0][0][0]->getSize();
  stdVecEig3d        centers;
  std::vector<Color> colors;
  obvious::obfloat   tr[3];
  _space->getCentroid(tr);
  for(unsigned int pz = 0; pz < _space->getPartitionsInZ(); pz++)
  {
    for(unsigned int py = 0; py < _space->getPartitionsInY(); py++)
    {
      for(unsigned int px = 0; px < _space->getPartitionsInX(); px++)
      {
        obvious::TsdSpacePartition* part = _space->getPartitions()[pz][py][px];
        if(part->isInitialized() && !part->isEmpty())
        {
          obvious::obfloat t[3];
          part->getCellCoordsOffset(t);
          for(unsigned int c = 0; c < partSize; c++)
          {
            Eigen::Vector3d center;
            center(0) = (*cellCoordsHom)(c, 0) + t[0];
            center(1) = (*cellCoordsHom)(c, 1) + t[1];
            center(2) = (*cellCoordsHom)(c, 2) + t[2];
            // if(center(1) > _space->getMaxY() / 2.0)
            //   continue;
            obvious::obfloat tsd = part->getTsd((*partCoords)(c, 0), (*partCoords)(c, 1), (*partCoords)(c, 2));
            if((isnan(tsd)) || (tsd > std::abs(1.1)))
              continue;

            Color tsdColor; //(0, 0, 0);
            if(tsd < 0.0)   // red
              tsdColor.red(static_cast<unsigned char>(-1.0 * tsd * 255.0));
            else
              tsdColor.blue(static_cast<unsigned char>(tsd * 255.0));

            centers.push_back(center);
            colors.push_back(tsdColor);
          }
        }
      }
    }
  }
  if((centers.size() == colors.size()) && (centers.size() != 0))
  {
    //_gui->drawGlyphs(colors, centers, space.getVoxelSize());
    for(unsigned int i = 0; i < centers.size(); i++)
    {
      pcl::PointXYZRGB p;
      p.x = centers[i].x() - tr[0];
      p.y = centers[i].y() - tr[1];
      p.z = centers[i].z() - tr[2];
      p.r = colors[i].r;
      p.g = colors[i].g;
      p.b = colors[i].b;
      cloud.push_back(p);
    }
  }
  else
    std::cout << __PRETTY_FUNCTION__ << "nuttingham found " << centers.size() << " " << colors.size() << std::endl;
  cloud.header.frame_id = _tfBaseFrame;
  cloud.header.seq      = seq++;
  std::cout << __PRETTY_FUNCTION__ << " publishing " << cloud.size() << " points" << std::endl;
}

void SensormodelTest::pubSensorRaycast(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  static obvious::obfloat* coords  = new obvious::obfloat[_cellsX * _cellsY * _cellsZ * 3];
  static obvious::obfloat* normals = new obvious::obfloat[_cellsX * _cellsY * _cellsZ * 3];
  static unsigned char*    rgb     = new unsigned char[_cellsX * _cellsY * _cellsZ * 3];
  static unsigned int      seq     = 0;

  // WAS MACHT DAS HIER
  // obvious::Matrix T = _sensor->getTransformation();
  // _filterBounds->setPose(&T);

  unsigned int width  = _sensor->getWidth();
  unsigned int height = _sensor->getHeight();
  unsigned int size   = 0;

  obvious::RayCast3D raycasterSensor;
  raycasterSensor.calcCoordsFromCurrentPose(_space.get(), _sensor.get(), coords, normals, rgb, &size);
  std::cout << __PRETTY_FUNCTION__ << "AAAAAAAAAAAA calcCoordsFromCurrentPose size = " << size << std::endl;
  obvious::obfloat tr[3];
  _space->getCentroid(tr);

  for(unsigned int i = 0; i < size; i += 3)
  {
    pcl::PointXYZ p;

    // p.x = coords[i] - tr[0];
    // p.z = coords[i + 1] - tr[1];
    // p.y = coords[i + 2] - tr[2];
    // p.x = -p.x;
    // cloud.push_back(p);

    // y u z vertauschen
    p.x = coords[i] - tr[0];
    p.y = coords[i + 1] - tr[1];
    p.z = coords[i + 2] - tr[2];
    // p.x = -p.x;
    cloud.push_back(p);
  }

  cloud.header.frame_id = _tfBaseFrame;
  cloud.header.seq      = seq++;
  std::cout << __PRETTY_FUNCTION__ << " publishing " << cloud.size() << " points" << std::endl;
}

void SensormodelTest::callbackPointcloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  // if neither sensor nor space are initialized yet -> do so in init routine
  if(!_sensor || !_space)
    this->init(cloud);
  std::cout << __PRETTY_FUNCTION__ << " cloud.header.frame_id = " << cloud.header.frame_id << std::endl;

  if(_withTf)
  {
    std::cout << __PRETTY_FUNCTION__ << " WITH TF " << std::endl;

    ros::Time            elapsedTimer = ros::Time::now();
    tf::StampedTransform tfSensor;
    // look up transform from cloud base frame_id to _tfBaseFrame=map for my sensor to transform sensor!
    try
    {
      _listener->waitForTransform("ground_truth", _tfBaseFrame, ros::Time(0), ros::Duration(10.0));
      _listener->lookupTransform("ground_truth", _tfBaseFrame, ros::Time(0), tfSensor);
    }
    catch(const tf::TransformException& e)
    {
      std::cout << __PRETTY_FUNCTION__ << "e: " << e.what() << std::endl;
      return;
    }

    // example matrix
    // tf::Transform carmen_pose
    // double yaw, pitch, roll;
    // 	tf::Matrix3x3(carmen_pose.getRotation()).getRPY(roll, pitch, yaw);

    // tf_echo ground_truth map beim ersten bag timestamp: (-5.185, -8,285, 0.310)

    // nur first Push
    if(!_virginPush) // muss das dann nicht weg?
    {
      std::cout << __PRETTY_FUNCTION__ << " virgin push, first point cloud in" << std::endl;
      std::cout << __PRETTY_FUNCTION__ << "tfSensorBefore:  translation: " << tfSensor.getOrigin().getX() << " , " << tfSensor.getOrigin().getY() << " , "
                << tfSensor.getOrigin().getZ() << std::endl;

      /*** hier später weiter
          static tf::TransformBroadcaster broadcaster;
          // tf::StampedTransform            transformInit;
          // transformInit.setOrigin(
          //     tf::Vector3(5.185, 8.285, 0.310)); // ausgelesen mit tfEcho und *(-1) gerechnet um zu map zu kommen. kommt auch oben beim cout so raus
          // tf::Quaternion qInit;
          // qInit.setRPY(0.0, 0.0, 0.0);
          // transformInit.setRotation(qInit);
          // std::cout << __PRETTY_FUNCTION__ << "StampedTransform transformInit:  translation: " << transformInit.getOrigin().getX() << " , "
          //           << transformInit.getOrigin().getY() << " , " << transformInit.getOrigin().getZ() << std::endl;

          // add transformInit to tfSensor
          // double x = tfSensor.getOrigin().getX() + 5.185;
          // double y = tfSensor.getOrigin().getY() + 8.285;
          // double z = tfSensor.getOrigin().getZ() + 0.310;
          double x = tfSensor.getOrigin().getX() + (-1) * tfSensor.getOrigin().getX();
          double y = tfSensor.getOrigin().getY() + (-1) * tfSensor.getOrigin().getY();
          double z = tfSensor.getOrigin().getZ() + (-1) * tfSensor.getOrigin().getZ();
          tfSensor.setOrigin(tf::Vector3(x, y, z));

          std::cout << __PRETTY_FUNCTION__ << "!!!!tfSensorAfter:  translation: " << tfSensor.getOrigin().getX() << " , " << tfSensor.getOrigin().getY() <<
    ", " << tfSensor.getOrigin().getZ() << std::endl;

          broadcaster.sendTransform(tf::StampedTransform(tfSensor, ros::Time::now(), "map", "ground_truth")); // oder groundtruth map vertauschen?
    ***/

      // FIRST INITIAL GROUND TRUTH
      // erste translation von groundtruth nach map rausziehen & const abspeichern. jede tf meiner punktwolke damit verschieben
      // bei der ersten reicht translation, wenn ich dann die neue tf immer wieder im callback verwerte, brauch ich auch die rotation
      /**
      const tf::Vector3 translGroundTrInitial = tfSensor.getOrigin();
      const obvious::Matrix InitGroundTruth(4, 4);
      InitGroundTruth = obvious::MatrixFactory::TransformationMatrix44(0.0, 0.0, 0.0, translGroundTrInitial.getX(), translGroundTrInitial.getY(),
      translGroundTrInitial.getZ()); const obvious::Matrix InverseInitGroundTruth = InitGroundTruth.getInverse();
      //muss ich jetzt hier die Inverse noch mit dem Space Mittelpunkt multiplizieren?
      //Space Mittelpunkt
      obvious::Matrix SpaceMid(4, 4);
      SpaceMid.setIdentity();
      obvious::obfloat mid[3];
      _space->getCentroid(mid);
      SpaceMid = obvious::MatrixFactory::TransformationMatrix44(0.0, 0.0, 0.0, mid[0], mid[1], mid[2]);
      **/

      // CURRENT TF --> kann mein tfListener hier auch auf groundtruth bleiben oder muss ich auf horizontal_velodyne gehen?
      // virgin push muss weg?

      // Eigen::Vector3f t(tf.getOrigin().getX(), tf.getOrigin().getY(), tf.getOrigin().getZ());
      tf::Vector3    tfVec = tfSensor.getOrigin();
      tf::Quaternion quat  = tfSensor.getRotation();
      tf::Matrix3x3  RotMat(quat);
      double         roll, pitch, yaw = 0.0;
      RotMat.getRPY(roll, pitch, yaw);

      // transform sensor
      obvious::Matrix TransMat(4, 4);
      TransMat.setIdentity();
      obvious::obfloat center[3];
      _space->getCentroid(center);
      TransMat =
          obvious::MatrixFactory::TransformationMatrix44(yaw, pitch, roll, center[0] + tfVec.getX(), center[1] + tfVec.getY(), center[2] + tfVec.getZ());
      _sensor->setTransformation(TransMat);
      std::cout << __PRETTY_FUNCTION__ << std::endl;
      std::cout << "You're getting there, love! Current Transformation of sensor : " << std::endl;
      _sensor->getTransformation().print();

      // extract data from pointcloud and write it to a vector of size height*weight, calculate depth value from xyz
      std::vector<double> depthData(cloud.height * cloud.width, 0.0);
      bool*               mask = new bool[cloud.height * cloud.width];

      std::cout << "Pointcloud height = " << cloud.height << " , width = " << cloud.width << std::endl;

      unsigned int valid = 0;

      // HEIGHT u WIDTH UMDREHEN WIE IN KÜNSTLICHEN DATEN UND SENSOR SORTIERT -> kein unterschied. ist ja klar. verzweifelter depp
      for(unsigned int i = 0; i < cloud.width; i++)
      {
        for(unsigned int j = 0; j < cloud.height; j++)
        {
          // NUR für künstliche Daten richtig --> SONST J NEHMEN weil HEIGHT = 1 IN VELOROSPOINTCLOUD
          const unsigned int idx = i * cloud.height + j; // das würde doch auch einfach mit ++ bis height*width gehen

          Eigen::Vector3f point(cloud.points[idx].x, cloud.points[idx].y, cloud.points[idx].z);

          double abs = static_cast<double>(point.norm());
          if(abs > 0.0)
          {
            depthData[idx] = abs;
            // depthData[idx] = 1.0; // artificial data with uniform ray length -- wird n Thorus
            mask[idx] = true;
            valid++;
          }
          else
          {
            mask[idx] = false;
          }
        }
      }

      if(!valid)
      {
        std::cout << __PRETTY_FUNCTION__ << " no valid points with depth value > 0.0 in data " << std::endl;
        return;
      }
      std::cout << __PRETTY_FUNCTION__ << " pushing " << valid << " valid points " << std::endl;
      _sensor->setRealMeasurementData(depthData.data());
      _sensor->setRealMeasurementMask(mask);

      delete mask;
      _space->push(_sensor.get());
      _virginPush = true;
    } // end if !_virginPush
    else
    {
      this->pubAxisAlignedRaycaster();

      pcl::PointCloud<pcl::PointXYZRGB> redBlueRenderedCloud;
      this->redBlueRenderSpace(redBlueRenderedCloud);
      _pubRedBlueRendered.publish(redBlueRenderedCloud);

      pcl::PointCloud<pcl::PointXYZ> sensorRaycastCloud;
      this->pubSensorRaycast(sensorRaycastCloud);
      _pubSensorRaycastCloud.publish(sensorRaycastCloud);
    }
  }    // end if with tf
  else // ohne tf
  {
    std::cout << __PRETTY_FUNCTION__ << " WITHOUT TF " << std::endl;

    if(!_virginPush) // muss das dann nicht weg?
    {
      std::cout << __PRETTY_FUNCTION__ << " virgin push, first point cloud in" << std::endl;

      // extract data from pointcloud and write it to a vector of size height*weight, calculate depth value from xyz
      std::vector<double> depthData(cloud.height * cloud.width, 0.0);
      bool*               mask = new bool[cloud.height * cloud.width];

      std::cout << "Pointcloud height = " << cloud.height << " , width = " << cloud.width << std::endl;

      unsigned int valid = 0;

      for(unsigned int i = 0; i < cloud.width; i++)
      {
        for(unsigned int j = 0; j < cloud.height; j++)
        {
          // NUR für künstliche Daten richtig --> SONST J NEHMEN weil HEIGHT = 1 IN VELOROSPOINTCLOUD
          const unsigned int idx = i * cloud.height + j; // das würde doch auch einfach mit ++ bis height*width gehen

          Eigen::Vector3f point(cloud.points[idx].x, cloud.points[idx].y, cloud.points[idx].z);

          double abs = static_cast<double>(point.norm());
          if(abs > 0.0)
          {
            depthData[idx] = abs;
            // depthData[idx] = 1.0; // artificial data with uniform ray length -- wird n Thorus
            mask[idx] = true;
            valid++;
          }
          else
          {
            mask[idx] = false;
          }
        }
      }

      if(!valid)
      {
        std::cout << __PRETTY_FUNCTION__ << " no valid points with depth value > 0.0 in data " << std::endl;
        return;
      }
      std::cout << __PRETTY_FUNCTION__ << " pushing " << valid << " valid points " << std::endl;
      _sensor->setRealMeasurementData(depthData.data());
      _sensor->setRealMeasurementMask(mask);

      delete mask;
      _space->push(_sensor.get());
      _virginPush = true;
    }
    else
    {
      this->pubAxisAlignedRaycaster();

      pcl::PointCloud<pcl::PointXYZRGB> redBlueRenderedCloud;
      this->redBlueRenderSpace(redBlueRenderedCloud);
      _pubRedBlueRendered.publish(redBlueRenderedCloud);

      pcl::PointCloud<pcl::PointXYZ> sensorRaycastCloud;
      this->pubSensorRaycast(sensorRaycastCloud);
      _pubSensorRaycastCloud.publish(sensorRaycastCloud);
    }
  } // end else ohne tf
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //   /***OHNE TF

  //   ros::Time            elapsedTimer = ros::Time::now();
  //   tf::StampedTransform tfSensor;
  //   // look up transform from cloud base frame_id to _tfBaseFrame=map for my sensor to transform sensor!
  //   try
  //   {
  //     _listener->waitForTransform("ground_truth", _tfBaseFrame, ros::Time(0), ros::Duration(10.0));
  //     _listener->lookupTransform("ground_truth", _tfBaseFrame, ros::Time(0), tfSensor);
  //   }
  //   catch(const tf::TransformException& e)
  //   {
  //     std::cout << __PRETTY_FUNCTION__ << "e: " << e.what() << std::endl;
  //     return;
  //   }

  // //example matrix
  //   // tf::Transform carmen_pose
  //   // double yaw, pitch, roll;
  //   // 	tf::Matrix3x3(carmen_pose.getRotation()).getRPY(roll, pitch, yaw);

  //   // tf_echo ground_truth map beim ersten bag timestamp: (-5.185, -8,285, 0.310)

  //   OHNE TF***/

  //   // nur first Push
  //   if(!_virginPush) // muss das dann nicht weg?
  //   {
  //     std::cout << __PRETTY_FUNCTION__ << " virgin push, first point cloud in" << std::endl;
  //     // std::cout << __PRETTY_FUNCTION__ << "tfSensorBefore:  translation: " << tfSensor.getOrigin().getX() << " , " << tfSensor.getOrigin().getY() << "
  //     , "
  //     //           << tfSensor.getOrigin().getZ() << std::endl;

  //     /*** hier später weiter
  //         static tf::TransformBroadcaster broadcaster;
  //         // tf::StampedTransform            transformInit;
  //         // transformInit.setOrigin(
  //         //     tf::Vector3(5.185, 8.285, 0.310)); // ausgelesen mit tfEcho und *(-1) gerechnet um zu map zu kommen. kommt auch oben beim cout so raus
  //         // tf::Quaternion qInit;
  //         // qInit.setRPY(0.0, 0.0, 0.0);
  //         // transformInit.setRotation(qInit);
  //         // std::cout << __PRETTY_FUNCTION__ << "StampedTransform transformInit:  translation: " << transformInit.getOrigin().getX() << " , "
  //         //           << transformInit.getOrigin().getY() << " , " << transformInit.getOrigin().getZ() << std::endl;

  //         // add transformInit to tfSensor
  //         // double x = tfSensor.getOrigin().getX() + 5.185;
  //         // double y = tfSensor.getOrigin().getY() + 8.285;
  //         // double z = tfSensor.getOrigin().getZ() + 0.310;
  //         double x = tfSensor.getOrigin().getX() + (-1) * tfSensor.getOrigin().getX();
  //         double y = tfSensor.getOrigin().getY() + (-1) * tfSensor.getOrigin().getY();
  //         double z = tfSensor.getOrigin().getZ() + (-1) * tfSensor.getOrigin().getZ();
  //         tfSensor.setOrigin(tf::Vector3(x, y, z));

  //         std::cout << __PRETTY_FUNCTION__ << "!!!!tfSensorAfter:  translation: " << tfSensor.getOrigin().getX() << " , " << tfSensor.getOrigin().getY()
  //         << " ,
  //     "
  //                   << tfSensor.getOrigin().getZ() << std::endl;

  //         broadcaster.sendTransform(tf::StampedTransform(tfSensor, ros::Time::now(), "map", "ground_truth")); // oder groundtruth map vertauschen?
  //   ***/

  //     // FIRST INITIAL GROUND TRUTH
  //     // erste translation von groundtruth nach map rausziehen & const abspeichern. jede tf meiner punktwolke damit verschieben
  //     // bei der ersten reicht translation, wenn ich dann die neue tf immer wieder im callback verwerte, brauch ich auch die rotation
  //     /**
  //     const tf::Vector3 translGroundTrInitial = tfSensor.getOrigin();
  //     const obvious::Matrix InitGroundTruth(4, 4);
  //     InitGroundTruth = obvious::MatrixFactory::TransformationMatrix44(0.0, 0.0, 0.0, translGroundTrInitial.getX(), translGroundTrInitial.getY(),
  //     translGroundTrInitial.getZ()); const obvious::Matrix InverseInitGroundTruth = InitGroundTruth.getInverse();
  //     //muss ich jetzt hier die Inverse noch mit dem Space Mittelpunkt multiplizieren?
  //     //Space Mittelpunkt
  //     obvious::Matrix SpaceMid(4, 4);
  //     SpaceMid.setIdentity();
  //     obvious::obfloat mid[3];
  //     _space->getCentroid(mid);
  //     SpaceMid = obvious::MatrixFactory::TransformationMatrix44(0.0, 0.0, 0.0, mid[0], mid[1], mid[2]);
  //     **/

  //     /*** OHNE TF
  //       /////////////////////////////////////////////////////////////////////////////////////////////////////////
  //       // CURRENT TF --> kann mein tfListener hier auch auf groundtruth bleiben oder muss ich auf horizontal_velodyne gehen?
  //       // virgin push muss weg?

  //       // Eigen::Vector3f t(tf.getOrigin().getX(), tf.getOrigin().getY(), tf.getOrigin().getZ());
  //       tf::Vector3    tfVec = tfSensor.getOrigin();
  //       tf::Quaternion quat  = tfSensor.getRotation();
  //       tf::Matrix3x3  RotMat(quat);
  //       double         roll, pitch, yaw = 0.0;
  //       RotMat.getRPY(roll, pitch, yaw);

  //       // transform sensor
  //       obvious::Matrix TransMat(4, 4);
  //       TransMat.setIdentity();
  //       obvious::obfloat center[3];
  //       _space->getCentroid(center);
  //       // obvious::obfloat yaw   = 0.0;
  //       // obvious::obfloat pitch = 0.0;
  //       // obvious::obfloat roll  = 0.0;
  //       TransMat = obvious::MatrixFactory::TransformationMatrix44(yaw, pitch, roll, center[0] + tfVec.getX(), center[1] + tfVec.getY(), center[2] +
  //       tfVec.getZ()); _sensor->setTransformation(TransMat); std::cout << __PRETTY_FUNCTION__ << std::endl; std::cout << "You're getting there, love!
  //       Current Transformation of sensor: " << std::endl; _sensor->getTransformation().print();
  //     //////////////////////////////////////////////////////////////////////////////////////////////////////
  //     OHNE TF***/

  //     // extract data from pointcloud and write it to a vector of size height*weight, calculate depth value from xyz
  //     std::vector<double> depthData(cloud.height * cloud.width, 0.0);
  //     bool*               mask = new bool[cloud.height * cloud.width];

  //     std::cout << "Pointcloud height = " << cloud.height << " , width = " << cloud.width << std::endl;

  //     unsigned int valid = 0;

  //     // HEIGHT u WIDTH UMDREHEN WIE IN KÜNSTLICHEN DATEN UND SENSOR SORTIERT -> kein unterschied. ist ja klar. verzweifelter depp
  //     for(unsigned int i = 0; i < cloud.width; i++)
  //     {
  //       for(unsigned int j = 0; j < cloud.height; j++)
  //       {
  //         // NUR für künstliche Daten richtig --> SONST J NEHMEN weil HEIGHT = 1 IN VELOROSPOINTCLOUD
  //         const unsigned int idx = i * cloud.height + j; // das würde doch auch einfach mit ++ bis height*width gehen
  //         // std::cout << __PRETTY_FUNCTION__ << "idx = i * cloud.height + j = " << i << " * " << cloud.height << " + " << j << " = " << idx << std::endl;

  //         Eigen::Vector3f point(cloud.points[idx].x, cloud.points[idx].y, cloud.points[idx].z);

  //         // Y u Z VERTAUSCHEN --> ändert aber nichts an abs --> bringt nichts
  //         // Eigen::Vector3f point(cloud.points[idx].x, cloud.points[idx].z, cloud.points[idx].y);

  //         double abs = static_cast<double>(point.norm());
  //         if(abs > 0.0)
  //         {
  //           depthData[idx] = abs;
  //           // depthData[idx] = 1.0; // artificial data with uniform ray length -- wird n Thorus
  //           mask[idx] = true;
  //           valid++;
  //         }
  //         else
  //         {
  //           mask[idx] = false;
  //         }
  //       }
  //     }

  //     if(!valid)
  //     {
  //       std::cout << __PRETTY_FUNCTION__ << " no valid points with depth value > 0.0 in data " << std::endl;
  //       return;
  //     }
  //     std::cout << __PRETTY_FUNCTION__ << " pushing " << valid << " valid points " << std::endl;
  //     _sensor->setRealMeasurementData(depthData.data());
  //     _sensor->setRealMeasurementMask(mask);

  //     delete mask;
  //     _space->push(_sensor.get());
  //     _virginPush = true;
  //   }
  //   else
  //   {
  //     this->pubAxisAlignedRaycaster();

  //     pcl::PointCloud<pcl::PointXYZRGB> redBlueRenderedCloud;
  //     this->redBlueRenderSpace(redBlueRenderedCloud);
  //     _pubRedBlueRendered.publish(redBlueRenderedCloud);

  //     pcl::PointCloud<pcl::PointXYZ> sensorRaycastCloud;
  //     this->pubSensorRaycast(sensorRaycastCloud);
  //     _pubSensorRaycastCloud.publish(sensorRaycastCloud);
  //   }
}
