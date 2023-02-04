#include "Robot.h"

#include <fmt/core.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <cameraserver/CameraServer.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
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
  cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();

  camera.SetResolution(320, 180);
  camera.SetFPS(120);
}

void Robot::setDriveMotors(double forward, double turn){
  //valeur positive de turn fait tourner à gauche (antihoraire)
  //test
  double left = forward - turn;
  double right = forward + turn;
  m_MotorRight.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, right);
  m_MotorLeft.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, left);
}

void Robot::RobotPeriodic()
{
  
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
setDriveMotors(0.0, 0.0);
    m_count=0;
    m_timer.Reset();
    m_timer.Start();
  }



}
//changement test
//test 2
void Robot::AutonomousPeriodic() {

  std::cout << "count: " << m_count << std::endl;
 
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
    //while (m_count < 20000) {
   if  (m_timer.Get() < 10_s) {
    std::cout << "on est dans le if" << std::endl;
    setDriveMotors(0.9, 0.0);
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
    setDriveMotors(0.9, 0.0);
    m_count++;
   }
    //else if (m_timer.Get() > 1_s && m_timer.Get() < 2_s) {
    //setDriveMotors(-0.3, 0.0);
    //m_count++;
   //}
    else {
    setDriveMotors(0.0, 0.9);
   }
  }
}
 



void Robot::TeleopInit() {}

void Robot::TeleopPeriodic()
{
  if (m_joystick.GetRawButtonPressed(1)) {
    setDriveMotors(0.5, 0.0);
} else {
   setDriveMotors(0.0, 0.0);
}

}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif