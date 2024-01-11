// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();
  m_drive.ZeroHeading();

  // Set up default drive command
  m_Actuator.SetDefaultCommand(frc2::RunCommand(
  [this]{
    m_driverController.GetPOV();
    frc::SmartDashboard::PutNumber("My POV Value is ", m_driverController.GetPOV());
  if (m_driverController.GetPOV()==0){
    m_Actuator.Extend();
  } else if (m_driverController.GetPOV()==180){
    m_Actuator.Retract();
  } else if (m_driverController.GetPOV()==-1){
    m_Actuator.Neutral();
  }
  },
  {&m_Actuator}));
  m_compressor.SetDefaultCommand(BeginCompressor(m_compressor));
  m_Shooter.SetDefaultCommand(ShooterSafe(&m_Shooter));
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
      bool noJoystick = false;
      double safeX = Deadzone(m_driverController.GetLeftX());
      double safeY =  Deadzone(m_driverController.GetLeftY());
      double safeRot = Deadzone(m_driverController.GetRightX());
      bool fieldOrientated;
      if (m_driverController.GetRawAxis(3)> 0.15){
        fieldOrientated = false;
      }
      if (m_driverController.GetRawAxis(3)< 0.15){
        fieldOrientated = true;
      }
      if((safeX == 0) && (safeY == 0) && (safeRot == 0)) {
        noJoystick = true;
      }
      m_drive.Drive(units::meters_per_second_t(
                    -safeY * AutoConstants::kMaxSpeed),
                    units::meters_per_second_t(
                    -safeX * AutoConstants::kMaxSpeed),
                    units::radians_per_second_t(
                    -safeRot * std::numbers::pi * 1.5),
                    fieldOrientated,
                    noJoystick);
      },
      {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
    frc2::JoystickButton(&m_driverController, 5).OnTrue(m_drive.ZeroHeading());
    frc2::JoystickButton(&m_driverController, 1).OnTrue(shootCmd);
//       [this] {
//         m_drive.ZeroHeading();
    //   },
//       {&m_drive}));
      
}

float RobotContainer::Deadzone(float x){
  if ((x < 0.1) &&  (x > -0.1)){
    x=0;
  }
  else if(x >= 0.1){
    x = x - 0.1;
  }
  else if(x <= -0.1){
    x = x + 0.1;
  }
  return(x);
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d{3_m, 0_m, 0_deg},
      // Pass the config
      config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::CommandPtr swerveControllerCommand =
      frc2::SwerveControllerCommand<4>(
          exampleTrajectory, [this]() { return m_drive.GetPose(); },

          m_drive.kDriveKinematics,

          frc::PIDController{AutoConstants::kPXController, 0, 0},
          frc::PIDController{AutoConstants::kPYController, 0, 0},
          thetaController,

          [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

          {&m_drive})
          .ToPtr();

  // Reset odometry to the initial pose of the trajectory, run path following
  // command, then stop at the end.
  return frc2::cmd::Sequence(
      frc2::InstantCommand(
          [this, &exampleTrajectory]() {
            m_drive.ResetOdometry(exampleTrajectory.InitialPose());
          },
          {})
          .ToPtr(),
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this] { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); }, {})
          .ToPtr());
}
