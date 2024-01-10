// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
//For the Xbox Controller
#include <frc/XboxController.h>
//For the motor controllers, both for motor and arm
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
//For grouping the left and right sides of the tank drive together
#include <frc/motorcontrol/MotorControllerGroup.h>
//For differential drive
#include <frc/drive/DifferentialDrive.h>
//Create a new thread for the cameras
#include <thread>
//For the intake and scoring
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>

ctre::phoenix::motorcontrol::can::WPI_TalonSRX FRMotor{0};
ctre::phoenix::motorcontrol::can::WPI_TalonSRX FLMotor{1};
ctre::phoenix::motorcontrol::can::WPI_TalonSRX BRMotor{2};
ctre::phoenix::motorcontrol::can::WPI_TalonSRX BLMotor{3};

frc::MotorControllerGroup lMotorGroup(FLMotor,BLMotor);
frc::MotorControllerGroup rMotorGroup(FRMotor,BRMotor);

frc::DifferentialDrive m_drive{lMotorGroup, rMotorGroup}; 

// Drive controller
frc::XboxController driveController{4};

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  //Begin new thread for camera processing
  std::jthread visionThread(VisionThread);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  // Left trigger
  double LTrigger = driveController.GetLeftTriggerAxis();
  // Right trigger
  double RTrigger = driveController.GetRightTriggerAxis();
  // Right joystick
  double RJoystick = driveController.GetRightX();
  // Right bumper
  bool RBumper = driveController.GetRightBumper();

  // Foward to the right & left
  if (RTrigger > 0 && (RJoystick > 0.05 || RJoystick < -0.05))
  {
    m_drive.ArcadeDrive(RTrigger, RJoystick, true);
  }
  // Backwards to the left & right
  else if (LTrigger > 0 && (RJoystick > 0.05 || RJoystick < -0.05))
  {
    m_drive.ArcadeDrive(LTrigger * -1, RJoystick, true);
  }
  // Forwards
  else if (RTrigger > 0)
  {
    m_drive.ArcadeDrive(RTrigger, 0, true);
  }
  // Backwards
  else if (LTrigger > 0)
  {
    m_drive.ArcadeDrive(LTrigger * -1, 0, true);
  }
  // Still
  else
  {
    m_drive.ArcadeDrive(0, 0, true);
  }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
