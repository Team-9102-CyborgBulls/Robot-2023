// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

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

}
void Robot::setDriveMotors(double forward, double turn){
  double left = forward - turn;
  double right = forward + turn;
  m_MotorRight.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, right);
  m_MotorLeft.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, left);
}


void Robot::AutonomousInit() {
  m_timer.Reset();
  m_timer.Start();
}
void Robot::AutonomousPeriodic() {
  double drivespeed;
  if(m_timer.Get()< 5_s) {
    drivespeed = 0.5;

  }
  else{
    drivespeed = 0.0; 
  }
  setDriveMotors(drivespeed,0.0);
}
void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
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