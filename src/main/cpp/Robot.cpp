// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdio>
#include <span>
#include <sstream>
#include <string>
#include <thread>
#include <frc/TimedRobot.h>


#include <networktables/IntegerArrayTopic.h>
#include <networktables/NetworkTableInstance.h>

#include <cameraserver/CameraServer.h>
#include <fmt/format.h>
#include "Constants.h"
#include "Robot.h"
#include "lib/Utils.h"
#include <fmt/core.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include <frc2/command/CommandScheduler.h>
#include <iostream>
#include <frc/Joystick.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/Talon.h>
#include <frc/controller/PIDController.h>
#include <frc/Encoder.h>
#include <frc/XboxController.h>


void Robot::RobotInit() {

  m_MotorRight.SetInverted(false);
  m_MotorRightFollow.SetInverted(false);
  m_MotorLeft.SetInverted(true);
  m_MotorLeftFollow.SetInverted(true);

  m_MotorRightFollow.Follow(m_MotorRight);
  m_MotorLeftFollow.Follow(m_MotorLeft);

  m_MotorRight.ConfigVoltageCompSaturation(12);
  m_MotorRightFollow.ConfigVoltageCompSaturation(12);
  m_MotorLeft.ConfigVoltageCompSaturation(12);
  m_MotorLeftFollow.ConfigVoltageCompSaturation(12);
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // Get the USB camera from CameraServer
    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
    // Set the resolution
    camera.SetResolution(640/2, 480/2);
    camera.SetFPS(120);
  
   // Resets the encoder to read a distance of zero
   encoder.Reset();

   encoder.SetDistancePerPulse(1.0/256.0);
   encoder.GetDistance();
   // Gets whether the encoder is stopped
   encoder.GetStopped();
   // Gets the last direction in which the encoder moved
   encoder.GetDirection();
   // Gets the current period of the encoder
   encoder.GetPeriod();



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

//double setpoint = pid.GetSetpoint();
// Calculates the output of the PID algorithm based on the sensor reading
// and sends it to a motor
//m_ArmMotor.Set(pid.Calculate(encoder.GetDistance(), setpoint));

  //double y = -m_joystick.GetY();
  //double z = m_joystick.GetZ();
  double y = -m_XboxController.GetLeftY();
  double x = m_XboxController.GetRightX();
  double forward = utils::Deadband(y);
  double turn = utils::Deadband(x);
  double v = forward * VMAX;
  double w = turn * WMAX;

  double left_wheel = v + (w * HALF_TRACKWIDTH);
  double right_wheel = v - (w * HALF_TRACKWIDTH);

  double k;
  k = 1.0 / (NMAX(VMAX, NMAX(NABS(left_wheel), NABS(right_wheel))));
  left_wheel *= k;
  right_wheel *= k;

  m_MotorRight.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, left_wheel); //
  m_MotorLeft.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, right_wheel); //

double ArmPower;
if (m_XboxController.GetRightBumper()==true) {
  ArmPower = -0.1;
}
else if (m_XboxController.GetLeftBumper()==true){
  ArmPower = 0.1;
}
else {
  ArmPower = 0.0;
}
setArmMotor(ArmPower);
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}

#endif