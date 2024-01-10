// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <cscore_oo.h>
#include <cameraserver/CameraServer.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  //Function for cameras (called by new thread)
  static void VisionThread()
  {
    cs::UsbCamera cameraF{"FrontCamera", 0};
    cs::UsbCamera cameraB{"BackCamera", 1};
    cameraF.SetResolution(320, 240);
    cameraB.SetResolution(160, 120);
    cameraF.SetFPS(30);
    cameraB.SetFPS(20);
    cameraF = frc::CameraServer::StartAutomaticCapture(0);
    cameraB = frc::CameraServer::StartAutomaticCapture(1);
    cs::CvSink cvSink = frc::CameraServer::GetVideo();
    cs::CvSource outputStream = frc::CameraServer::PutVideo("Rectangle", 800, 700);
  }
};
