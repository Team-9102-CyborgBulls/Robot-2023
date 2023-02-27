// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

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

#include <frc/Joystick.h>
#include "Constants.h"

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/Talon.h>
#include "rev/CANSparkMax.h"
#include <frc/controller/PIDController.h>
#include <frc/Encoder.h>
#include <frc/XboxController.h>



class Robot : public frc::TimedRobot
{
  //#if defined(__linux__) || defined(_WIN32)
 
public:
  void setDriveMotors(double forward, double turn);
  void RobotInit() override;
  void RobotPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void setArmMotor(double percent){
    m_ArmMotor.Set(percent);
  }

  
private:
 double m_count;
 frc::Joystick m_joystick{0};
 frc::XboxController m_XboxController{0};
 frc::Timer m_timer;
 frc::SendableChooser<std::string> m_chooser;
 //frc2::CommandScheduler::CommandScheduler 
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  rev::CANSparkMax m_ArmMotor{6, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_RotationIntake{7, rev::CANSparkMax::MotorType::kBrushless};
  ctre::phoenix::motorcontrol::can::TalonSRX m_MotorRight{CAN_ID_DRIVETRAIN_MOTOR_RIGHT};
  ctre::phoenix::motorcontrol::can::TalonSRX m_MotorRightFollow{CAN_ID_DRIVETRAIN_MOTOR_RIGHT_FOLLOW};
  ctre::phoenix::motorcontrol::can::TalonSRX m_MotorLeft{CAN_ID_DRIVETRAIN_MOTOR_LEFT};
  ctre::phoenix::motorcontrol::can::TalonSRX m_MotorLeftFollow{CAN_ID_DRIVETRAIN_MOTOR_LEFT_FOLLOW};
  std::function<double()> m_Forward;
  std::function<double()> m_Turn;
  std::function<double()> m_Slide;
 
// Initializes an encoder on DIO pins 0 and 1
// Defaults to 4X decoding and non-inverted
frc::Encoder encoder{0, 1};

// Creates a PIDController with gains kP, kI, and kD
//frc2::PIDController pid(kP, kI, kD);
//void frc2::PIDController::SetPID(double kP, double kI, double kD);	


};
   