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
#include <frc/XboxController.h>
#include <frc2/command/PIDCommand.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <iostream>
#include <rev/SparkMaxAbsoluteEncoder.h>
#include <math.h>
#include <frc/Encoder.h>
#include <units/pressure.h>



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
  void setArmMotor(double percent, int amps){
    m_ArmMotor.Set(percent);
     m_ArmMotor.SetSmartCurrentLimit(amps);
  }
  void setIntakeRotor(double percent, int amps) {
    m_IntakeRotor.Set(percent);
    m_IntakeRotor.SetSmartCurrentLimit(amps);
  }

  
private:
 
 frc::Joystick m_joystick{0};
 frc::XboxController m_XboxController{1};
 frc::Timer m_timer;
 frc::SendableChooser<std::string> m_chooser;
 //frc2::CommandScheduler::CommandScheduler 
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto"; 
  std::string m_autoSelected;
  rev::CANSparkMax m_ArmMotor{CAN_ID_ARM_MOTOR, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_IntakeRotor{CAN_ID_INTAKE_ROTOR, rev::CANSparkMax::MotorType::kBrushed};
  ctre::phoenix::motorcontrol::can::TalonSRX m_MotorRight{CAN_ID_DRIVETRAIN_MOTOR_RIGHT};
  ctre::phoenix::motorcontrol::can::TalonSRX m_MotorRightFollow{CAN_ID_DRIVETRAIN_MOTOR_RIGHT_FOLLOW};
  ctre::phoenix::motorcontrol::can::TalonSRX m_MotorLeft{CAN_ID_DRIVETRAIN_MOTOR_LEFT};
  ctre::phoenix::motorcontrol::can::TalonSRX m_MotorLeftFollow{CAN_ID_DRIVETRAIN_MOTOR_LEFT_FOLLOW};
  std::function<double()> m_Forward;
  std::function<double()> m_Turn;
  std::function<double()> m_Slide;
  rev::SparkMaxAbsoluteEncoder m_encoder();

 frc::Compressor phCompressor{0, frc::PneumaticsModuleType::REVPH};
 frc::DoubleSolenoid DoublePH{1, frc::PneumaticsModuleType::REVPH, 8, 9};	
 bool enabled = phCompressor.Enabled();
 bool pressureSwitch = phCompressor.GetPressureSwitchValue();

 
// Initializes an encoder on DIO pins 0 and 1
// Defaults to 4X decoding and non-inverted
//frc::Encoder encoder{0, 1};

// Creates a PIDController with gains kP, kI, and kD

rev::SparkMaxPIDController m_pidController = m_ArmMotor.GetPIDController();

double kP = 0, kI = 0, kD = 0;



/*double Kp;
double Ki;
double Kd;
double targetCurrent = 0;//le setpoint
double error = targetCurrent;
double errorChange = error-lastError;
double errorSum = error + errorChange;
double correction = Kp*error + Ki*errorSum + Kd*errorChange;
double lastError = error;
cout << "correction: " << correction << endl;*/
};