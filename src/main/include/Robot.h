// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <frc/Joystick.h>
#include "Constants.h"
#include <frc/TimedRobot.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/smartdashboard/SendableChooser.h>

class Robot : public frc::TimedRobot
{
public:
  void setDriveMotors(double forward, double turn);
  void RobotInit() override;
  void RobotPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
private:
 double m_count;
 frc::Joystick m_joystick{0};
 frc::Timer m_timer;
 frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  ctre::phoenix::motorcontrol::can::TalonSRX m_MotorRight{CAN_ID_DRIVETRAIN_MOTOR_RIGHT};
  ctre::phoenix::motorcontrol::can::TalonSRX m_MotorRightFollow{CAN_ID_DRIVETRAIN_MOTOR_RIGHT_FOLLOW};
  ctre::phoenix::motorcontrol::can::TalonSRX m_MotorLeft{CAN_ID_DRIVETRAIN_MOTOR_LEFT};
  ctre::phoenix::motorcontrol::can::TalonSRX m_MotorLeftFollow{CAN_ID_DRIVETRAIN_MOTOR_LEFT_FOLLOW};

};