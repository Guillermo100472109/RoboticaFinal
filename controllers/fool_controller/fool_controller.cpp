// File:          full_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Compass.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/GPS.hpp>
#include <webots/Camera.hpp>
#include <math.h>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // initializing and testing motors
  Motor *_leftMotor = robot->getMotor("left wheel motor");
  Motor *_rightMotor = robot->getMotor("right wheel motor");
  
  _leftMotor->setPosition(INFINITY);
  _rightMotor->setPosition(INFINITY);
  
  _leftMotor->setVelocity(3);
  _rightMotor->setVelocity(5);
  
  // initializing encoders
  PositionSensor * _left_wheel_sensor = robot->getPositionSensor("left wheel sensor");
  PositionSensor * _right_wheel_sensor = robot->getPositionSensor("right wheel sensor");
  _left_wheel_sensor->enable(timeStep);
  _right_wheel_sensor->enable(timeStep);
    
  // initializing distance sensors
  DistanceSensor*_distance_sensor[16];
  const char *ds_name[16] = {"ds0", "ds1" ,"ds2", "ds3", "ds4", "ds5", "ds6", "ds7", "ds8", "ds9", "ds10", "ds11", "ds12","ds13", "ds14", "ds15"};
  for (int ind = 0; ind < 16; ind++){
    _distance_sensor[ind] = robot->getDistanceSensor(ds_name[ind]); 
    _distance_sensor[ind]->enable(timeStep);
  }
  
  // initializing compass
  Compass *_compass = robot->getCompass("compass");
  _compass->enable(timeStep);
  
  // initializing GPS
  GPS * _gps = robot->getGPS("gps");
  _gps->enable(timeStep);
  
  // initializing and testing cameras
  Camera *_front_cam = robot->getCamera("camera_f");
  _front_cam->enable(timeStep);
  std::cout << "Front images are " << _front_cam->getWidth() << " x " << _front_cam->getHeight() << std::endl;
  Camera *_spher_cam = robot->getCamera("camera_s");
  _spher_cam->enable(timeStep);
  std::cout << "Spherical images are " << _spher_cam->getWidth() << " x " << _spher_cam->getHeight() << std::endl;  
  

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    
    // testing encoders
    std::cout << "Left encoder: " << _left_wheel_sensor->getValue() << std::endl;
    std::cout << "Right encoder: " << _right_wheel_sensor->getValue() << std::endl;
    
    // testing distance sensors
    for (int ind = 0; ind < 16; ind++){
      std::cout << "DS" << ind << ": " << _distance_sensor[ind]->getValue() << std::endl;
    }

    // testing compass
    const double * compass_vals = _compass->getValues();
    if (compass_vals != NULL)
      std::cout << "Compass values: " << compass_vals [0] << "," << compass_vals[1] << "," << compass_vals[2] << std::endl;
    else
      std::cout << "Compass values are NULL" << std::endl;

    // testing GPS
    const double * gps_vals = _gps->getValues();
    if (gps_vals != NULL)
    {
      std::cout << "GPS values: " << gps_vals[0] << "," << gps_vals[1] << "," << gps_vals[2] << std::endl;
      std::cout << "GPS speed [m/s]: " << _gps->getSpeed() << std::endl;
    } else
      std::cout << "GPS values are NULL" << std::endl;


    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.
  _leftMotor->setVelocity(0);
  _rightMotor->setVelocity(0);
  _left_wheel_sensor->disable();
  _right_wheel_sensor->disable();  
  for (int ind = 0; ind < 16; ind++){
    _distance_sensor[ind]->disable();
  }
  _compass->disable();
  _gps->disable();
  _front_cam->disable();
  _spher_cam->disable();
  delete robot;
  
  return 0;
}
