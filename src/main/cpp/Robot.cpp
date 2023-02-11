// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
   if  (m_timer.Get() < 10_s) {
    std::cout << "on est dans le if" << std::endl;
    setDriveMotors(0.4, 0.0);
    m_count++;
   }  
    //else if  (m_timer.Get() > 1_s && m_timer.Get() < 2_s) {
    //setDriveMotors(0.0, -0.3);
    //m_count++;
   //}  
    else {
      setDriveMotors(0.0, 0.0);
   }
  }
  else {
    // Default Auto goes here
     //while (m_count < 20000) {
    if  (m_timer.Get() < 10_s) {
    std::cout << "on est dans le if" << std::endl;
    setDriveMotors(0.4, 0.0);
    m_count++;
   } 
  } 
    //else if (m_timer.Get() > 1_s && m_timer.Get() < 2_s) {
    //setDriveMotors(-0.3, 0.0);
    //m_count++;
   //}  
    if (m_timer.Get() > 1_s && m_timer.Get() < 2_s) {
    setDriveMotors(0.0, 0.4);   
   }
  }
void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
 
  setDriveMotors(-m_joystick.GetY(), -m_joystick.GetZ());

  /*double forward = utils::Deadband(m_Forward());
  double turn = utils::Deadband(m_Turn());
  double slide = std::abs(m_Slide());
  forward = slide;
  turn= 0.5;


  setDriveMotors(forward, turn);*/
  double driveSpeed;

 if (m_joystick.GetRawButton(1)) {
    std::cout << "drivespeed non nulle" << std::endl;
    driveSpeed = 0.5;

  }
 else{
    driveSpeed = 0.0;
 }

setDriveMotors(driveSpeed,0.0);



}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}

#endif