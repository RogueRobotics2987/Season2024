#pragma once
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/SwerveModuleSubsystem.h"
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/geometry/Translation2d.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <iostream>
#include "subsystems/LimelightPose.h"
#include "commands/LimeLightCmd.h"


/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();
  frc2::Command* GetAutonomousCommand();


  float Deadzone(float x);


 private:
  //replace with frc::Joystick if using a joystick instead of an xbox controller
  frc::XboxController m_driverController{0};


  // The robot's subsystems are defined here...
  DriveSubsystem m_drive;
  LimelightPose m_limePose;


  void ConfigureButtonBindings();
};
