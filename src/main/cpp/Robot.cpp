// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdio>
#include <span>
#include <sstream>
#include <string>
#include <thread>
#include <frc/TimedRobot.h>
#include <frc/apriltag/AprilTagDetection.h>
#include <frc/apriltag/AprilTagDetector.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>
#include <frc/geometry/Transform3d.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <units/angle.h>
#include <units/length.h>
#include <cameraserver/CameraServer.h>
#include <fmt/format.h>

#include "Robot.h"
#include <fmt/core.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include <frc2/command/CommandScheduler.h>
#include <iostream>
#include <frc/Joystick.h>

void Robot::RobotInit() {

  m_MotorRight.SetInverted(true);
  m_MotorRightFollow.SetInverted(true);
  m_MotorLeft.SetInverted(false);
  m_MotorLeftFollow.SetInverted(false);

  m_MotorRightFollow.Follow(m_MotorRight);
  m_MotorLeftFollow.Follow(m_MotorLeft);

  m_MotorRight.ConfigVoltageCompSaturation(12);
  m_MotorRightFollow.ConfigVoltageCompSaturation(12);
  m_MotorLeft.ConfigVoltageCompSaturation(12);
  m_MotorLeftFollow.ConfigVoltageCompSaturation(12);
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  
    // We need to run our vision program in a separate thread. If not, our robot
    // program will not run.
#if defined(__linux__) || defined(_WIN32)
    std::thread visionThread(VisionThread);
    visionThread.detach();
#else
    std::fputs("Vision only available on Linux or Windows.\n", stderr);
    std::fflush(stderr);
#endif
  

}

void Robot::RobotPeriodic()
{
  
  frc2::CommandScheduler::GetInstance().Run();
}
void Robot::setDriveMotors(double forward, double turn){
  double left = forward - turn;
  double right = forward + turn;
  m_MotorRight.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, right);
  m_MotorLeft.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, left);
}


void Robot::AutonomousInit() {
   m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
    setDriveMotors(0.0, 0.0);
    m_count=0;
    m_timer.Reset();
    m_timer.Start();
  } else {
    // Default Auto goes here
    setDriveMotors(0.0, 0.0);
    m_count=0;
    m_timer.Reset();
    m_timer.Start();
  }
}


void Robot::AutonomousPeriodic() {
  std::cout << "count: " << m_count << std::endl;
 
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
    //while (m_count < 20000) {
   if  (m_timer.Get() < 2_s) {
    std::cout << "on est dans le if" << std::endl;
    setDriveMotors(-0.4, 0.0);
    m_count++;
   }  
    //else if  (m_timer.Get() > 1_s && m_timer.Get() < 2_s) {
    //setDriveMotors(0.0, -0.3);
    //m_count++;
   //}  
    else {
      setDriveMotors(0.0, 0.0);
   }
  
  }else {
    // Default Auto goes here
     //while (m_count < 20000) {
  if  (m_timer.Get() < 2_s) {
    std::cout << "on est dans le if" << std::endl;
    setDriveMotors(0.0, 0.4);
    m_count++;
   } 
   
  else  {
    setDriveMotors(0.0, 0.0);
   
  }  
   
 }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
double driveSpeed;
double driveTurn;


 //m_robotDrive.ArcadeDrive(-m_joystick.GetY(), -m_joystick.GetX());
//driveSpeed = 0.2;
//driveTurn = 0.2;

/*if(m_joystick.GetY()> 0.5){
   setDriveMotors(driveSpeed, driveTurn);

 }else if(m_joystick.GetY() < -0.5){
  setDriveMotors(-driveSpeed, driveTurn);
 }
 else{
  setDriveMotors(0.0, 0.0);
}

if(m_joystick.GetZ()> 0.5){
   setDriveMotors(0.0, driveTurn);

 }else if(m_joystick.GetZ() < -0.5){
  setDriveMotors(0.0, -driveTurn);
 }
 else{
  setDriveMotors(0.0, 0.0);
}*/


  
  /*double forward = utils::Deadband(m_Forward());
  double turn = utils::Deadband(m_Turn());
  double slide = std::abs(m_Slide());
  forward = slide;
  turn= 0.5;


  setDriveMotors(forward, turn);*/
  

/*if (m_joystick.GetRawButton(1)) {
    std::cout << "drivespeed non nulle" << std::endl;
    driveSpeed = 0.5;

  }
 else{
    driveSpeed = 0.0;
 }

  setDriveMotors(driveSpeed,0.0);*/



}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}

#endif