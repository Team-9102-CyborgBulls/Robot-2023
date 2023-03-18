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
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include <frc2/command/CommandScheduler.h>
#include <iostream>
#include <frc/Joystick.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/Talon.h>
#include <frc/controller/PIDController.h>
#include <frc/XboxController.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <rev/SparkMaxAbsoluteEncoder.h> 



void Robot::RobotInit() {

  m_MotorRight.SetInverted(true);
  m_MotorRightFollow.SetInverted(true);
  m_MotorLeft.SetInverted(false);
  m_MotorLeftFollow.SetInverted(false);

  m_MotorRightFollow.Follow(m_MotorRight);
  m_MotorLeftFollow.Follow(m_MotorLeft);

  m_MotorRight.ConfigVoltageCompSaturation(VOLTAGE_COMPENSATION_DRIVETRAIN_MOTOR);
  m_MotorRightFollow.ConfigVoltageCompSaturation(VOLTAGE_COMPENSATION_DRIVETRAIN_MOTOR);
  m_MotorLeft.ConfigVoltageCompSaturation(VOLTAGE_COMPENSATION_DRIVETRAIN_MOTOR);
  m_MotorLeftFollow.ConfigVoltageCompSaturation(VOLTAGE_COMPENSATION_DRIVETRAIN_MOTOR);
 // m_ArmMotor.GetVoltageCompensationNominalVoltage(VOLTAGE_COMPENSATION_ARM_MOTOR);
  //m_IntakeRotor.GetVoltageCompensationNominalVoltage(VOLTAGE_COMPENSATION_ARM_MOTOR);
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  m_chooser.AddOption(kAutoNameTest, kAutoNameTest);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // Get the USB camera from CameraServer
  cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
  //Set the resolution
  camera.SetResolution(640/2, 480/2);
  camera.SetFPS(120);

    

    m_pidController.SetP(kP);
    m_pidController.SetI(kI);
    m_pidController.SetD(kD);
  
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    gyroX.InitGyro();
    gyroX.Calibrate();
    

   // Resets the encoder to read a distance of zero
   /*encoder.Reset();

   encoder.SetDistancePerPulse(1.0/256.0);
   encoder.GetDistance();
   // Gets whether the encoder is stopped
   encoder.GetStopped();
   // Gets the last direction in which the encoder moved
   encoder.GetDirection();
   // Gets the current period of the encoder
   encoder.GetPeriod();*/

   //void frc2::PIDController::SetP(0);
   //void frc2::PIDController::SetI(0);
   //void frc2::PIDController::SetD(0);	
   //void frc2::PIDController::SetPID (0, 0, 0);

}

void Robot::RobotPeriodic()
{
  std::cout<<gyroX.GetRate()<<std::endl;
  std::cout<<gyroY.GetValue()<<std::endl;
  std::cout<<gyroZ.GetValue()<<std::endl;
}

void Robot::setDriveMotors(double forward, double turn){
  
  double left = forward - turn;
  double right = forward + turn;
  m_MotorRight.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, right);
  m_MotorLeft.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, left);
}


void Robot::AutonomousInit() {
  
  m_autoSelected = m_chooser.GetSelected();
  fmt::print("Auto selected: {}\n", m_autoSelected);

 if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
    setDriveMotors(0.0, 0.0);
    //m_count=0;
    m_timer.Reset();
    m_timer.Start();
  } 
  if (m_autoSelected == kAutoNameTest) {
    setDriveMotors(0.0, 0.0);
    //m_count=0;
    m_timer.Reset();
    m_timer.Start();
  }

  else {
    // Default Auto goes here
    setDriveMotors(0.0, 0.0);
    //m_count=0;
    m_timer.Reset();
    m_timer.Start();
  }
 
}


void Robot::AutonomousPeriodic() {
  
 
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
   if  (m_timer.Get() < 2_s) {
    std::cout << "on est dans le if" << std::endl;
    setDriveMotors(-0.4, 0.0);
    
   }  
    //else if  (m_timer.Get() > 1_s && m_timer.Get() < 2_s) {
    //setDriveMotors(0.0, -0.3);
    //m_count++;
   //}  
   else {
    setDriveMotors(0.0, 0.0);
   }
  
  }if (m_autoSelected == kAutoNameTest){
    // Custom Auto goes here
   if  (m_timer.Get() < 2.1_s) {
 
    setDriveMotors(0.4, 0.0);
    //m_count++;
   } 
   else if (m_timer.Get() > 2.1_s && m_timer.Get() < 2.15_s){
        setDriveMotors(0.0, -0.1);
   }
   else {
    setDriveMotors(0.0, 0.0);
   }
  
  }else {
    // Default Auto goes here
     //while (m_count < 20000) {
   if  (m_timer.Get() < 2_s) {
    
    setDriveMotors(0.0, 0.4);
    
   }else{
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

  double p = frc::SmartDashboard::GetNumber("P Gain", 0);
  double i = frc::SmartDashboard::GetNumber("I Gain", 0);
  double d = frc::SmartDashboard::GetNumber("D Gain", 0);
//  frc::SmartDashboard::PutNumber("Encoder Position", m_encoder().GetPosition());
//  frc::SmartDashboard::PutNumber("Encoder Velocity", m_encoder().GetVelocity());

  if((p != kP)) { m_pidController.SetP(p); kP = p; }
  if((i != kI)) { m_pidController.SetI(i); kI = i; }
  if((d != kD)) { m_pidController.SetD(d); kD = d; }

  //m_pidController.SetReference(rotations, rev::CANSparkMax::ControlType::kPosition);
  //frc::SmartDashboard::PutNumber("SetPoint", rotations);

  double y = -m_joystick.GetY();
  double z = m_joystick.GetZ();
  double forward = utils::Deadband(y);
  double turn = utils::Deadband(z);
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
if (m_joystick.GetRawButton(5)==true) {
  ArmPower = -0.54;
}
else if (m_joystick.GetRawButton(6)==true){
  ArmPower = 0.54;
}
else {
  ArmPower = -0.02;
}
setArmMotor(ArmPower);

double IntakeRotorPower;

if(m_joystick.GetRawButton(8)==true){
 IntakeRotorPower = -0.3;
} else if(m_joystick.GetRawButton(7)==true){
 IntakeRotorPower = 0.3;
}else{
  IntakeRotorPower = 0.0;
}
setIntakeRotor(IntakeRotorPower, 40);
/*
if(m_joystick.GetRawButton(4)==true){
DoublePH.Set(frc::DoubleSolenoid::Value::kForward);
}else if(m_joystick.GetRawButton(5)==true){

DoublePH.Set(frc::DoubleSolenoid::Value::kReverse);

}else{
DoublePH.Set(frc::DoubleSolenoid::Value::kOff);
}
*/



}


#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif