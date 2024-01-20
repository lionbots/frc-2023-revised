// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/format.h>
//For the Xbox Controller
#include <frc/XboxController.h>
//For regular Joystick
#include <frc/Joystick.h>
//#include <frc/GenericHID.h>
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
//For the IMU
#include <frc/ADIS16470_IMU.h>
//Units Library
#include <units/base.h>

// Drive system
ctre::phoenix::motorcontrol::can::WPI_TalonSRX FRMotor{4};
ctre::phoenix::motorcontrol::can::WPI_TalonSRX FLMotor{0};
ctre::phoenix::motorcontrol::can::WPI_TalonSRX BRMotor{3};
ctre::phoenix::motorcontrol::can::WPI_TalonSRX BLMotor{1};

frc::MotorControllerGroup lMotorGroup(FLMotor,BLMotor);
frc::MotorControllerGroup rMotorGroup(FRMotor,BRMotor);

frc::DifferentialDrive m_drive{lMotorGroup, rMotorGroup}; 

// Manipulator
rev::CANSparkMax intakeL{5, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
rev::CANSparkMax intakeR{6, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
ctre::phoenix::motorcontrol::can::WPI_TalonSRX arm{2};
double armSp = 0.0;

// Basic controls
frc::Joystick joystickController{0};

// Pro controls
frc::XboxController driveController{1};
frc::XboxController manipulatorController{2};

//IMU
frc::ADIS16470_IMU imu{};
/* Acceleration X-Axis */ double accelerationX = imu.GetAccelX().value() * 3.280839895;
/* Acceleration Y-Axis */ double accelerationY = imu.GetAccelY().value() * 3.280839895;
/* Velocity X-Axis */ double velocityX = velocityX + accelerationX;
/* Velocty Y-Axis */ double velocityY = velocityY + accelerationY;
/* Position X-Axis */ double positionX = positionX + velocityX;
/* Position Y-Axis */ double positionY = positionY + velocityY;
/* Ticks since last print */ int ticks = 0;


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
  // CONTROL DEFINITION
  
  // PRO CONTROLS
  // Drive System
  /* GO BACK    - L2 */double driveLTrigger = driveController.GetLeftTriggerAxis();
  /* GO FORWARD - R2*/double driveRTrigger = driveController.GetRightTriggerAxis();
  /* DIRECTION  - L STICK*/double driveLJoystick = driveController.GetLeftX();

  // Manipulator Arm System
  /* RELEASE ARM - A*/bool manipAButton = manipulatorController.GetAButton();
  /* RETRACT ARM - B*/bool manipBButton = manipulatorController.GetBButton();
  /* INTAKE      - L1*/bool manipLBumper = manipulatorController.GetLeftBumper();
  /* EJECT       - R1*/bool manipRBumper = manipulatorController.GetRightBumper();
  /* MAX EJECT   - R1*/double manipRTrigger = manipulatorController.GetRightTriggerAxis();

  // JOYSTICK CONTROLS
  // Drive System
  /* Forward and Backwards - Y AXIS*/ double joyYAxis = joystickController.GetY();
  /* Turning               - Z AXIS*/ double joyZAxis = joystickController.GetZ();

  // Manipulator Arm System
  /* RELEASE ARM - #5*/ bool joyButtonFive = joystickController.GetRawButton(5);
  /* RETRACT ARM - #10*/ bool joyButtonTen = joystickController.GetRawButton(10);
  /* INTAKE      - #2*/ bool joyButtonTwo = joystickController.GetRawButton(2);
  /* EJECT       - #1*/ bool joyButtonOne = joystickController.GetRawButton(1);
  /* MAX EJECT   - #3*/ bool joyButtonThree = joystickController.GetRawButton(3);

  //IMU
  /* Acceleration X-Axis */ accelerationX = imu.GetAccelX().value() * 3.280839895;
  /* Acceleration Y-Axis */ accelerationY = imu.GetAccelY().value() * 3.280839895;
  /* Velocity X-Axis */ velocityX = velocityX + accelerationX;
  /* Velocty Y-Axis */ velocityY = velocityY + accelerationY;
  /* Position X-Axis */ positionX = positionX + velocityX;
  /* Position Y-Axis */ positionY = positionY + velocityY;

  //Increases ticks by 1 every 20ms
  ticks++;
  //Every 10 ticks it will print acceleration, velocity, and position and resets ticks
  if(ticks == 10) {
    fmt::print("[{}, {}, {}, {}, {}, {}]\n", accelerationX, accelerationY, velocityX, velocityY, positionX, positionY);
    ticks = 0;
  }

  //DRIVE SYSTEM
  if (!(joyZAxis > 0.05 || joyZAxis < 0.05 || joyYAxis > 0.05 || joyYAxis < 0.)) {
    // PRO CONTROL SCHEME
    // Drive System
    // Foward to the right & left
    if (driveRTrigger > 0 && (driveLJoystick > 0.05 || driveLJoystick < -0.05)) {
      m_drive.ArcadeDrive(driveLJoystick, driveRTrigger * 0.7, true);
    }
    // Backwards to the left & right
    else if (driveLTrigger > 0 && (driveLJoystick > 0.05 || driveLJoystick < -0.05)) {
      m_drive.ArcadeDrive(driveLJoystick, driveLTrigger * -0.7, true);
    }
    // Forwards
    else if (driveRTrigger > 0) {
      m_drive.ArcadeDrive(0, driveRTrigger, true);
    }
    // Backwards
    else if (driveLTrigger > 0) {
      m_drive.ArcadeDrive(0, driveLTrigger * -1, true);
    }
    // Still
    else {
      m_drive.ArcadeDrive(0, 0, true);
  }
  } else {
    // JOYSTICK CONTROLS
    // Drive System
    m_drive.ArcadeDrive(joyZAxis * -0.7, joyYAxis, true);
  }

  // Manipulator System
  // Arm
  if (manipAButton || joyButtonFive) {
    if (armSp < 0.3) {
        armSp += 0.005;
        arm.Set(armSp);
      } else {
        arm.Set(armSp);
      }
  } else if (manipBButton || joyButtonThree) {
    if (armSp > -0.3) {
        armSp -= 0.005;
        arm.Set(armSp);
      } else {
        arm.Set(armSp);
      }
  } else {
    arm.Set(0);
  }
  // Grippers
  if (manipRBumper || joyButtonOne) { //eject
    intakeL.Set(0.7);
    intakeR.Set(-0.7);
  } else if (manipLBumper || joyButtonTwo) { //intake
    intakeL.Set(-0.3);
    intakeR.Set(0.3);
  } else if (manipRTrigger > 0.2 || joyButtonThree) { //max eject
    int manipEjectSpeed = manipRTrigger;
    int joyEjectSpeed = joyButtonThree ? 1 : 0;
    if (manipEjectSpeed > joyButtonThree) {
      intakeL.Set(manipRTrigger);
      intakeR.Set(manipRTrigger * -1);
    } else {
      intakeL.Set(1);
      intakeR.Set(-1);
    }
  } else {
    intakeL.Set(-0.02);
    intakeR.Set(0.02);
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
