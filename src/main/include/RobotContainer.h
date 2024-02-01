#pragma once

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/Joystick.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/XboxController.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <iostream>
#include <pathplanner/lib/path/PathPlannerPath.h> 
#include <pathplanner/lib/commands/FollowPathHolonomic.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

#include "subsystems/LimelightPose.h"
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/SwerveModuleSubsystem.h"
#include "commands/NoteFollower.h"
#include "commands/AprilTagFollower.h"
#include "commands/AutoAprilTag.h"
#include "commands/followWaypoints.h"

using namespace pathplanner;

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer
{
  public:
    RobotContainer();
    frc2::CommandPtr GetAutonomousCommand();
    frc2::CommandPtr GetPath(std::vector<frc::Pose2d> waypoints);
    float Deadzone(float x); 
    frc2::CommandPtr onFlyGeneration();

  private:
    //replace with frc::Joystick if using a joystick instead of an xbox controller
    frc::XboxController m_driverController{0};
    // The robot's subsystems are defined here...
    DriveSubsystem m_drive;
    LimelightPose m_limePose;
    // frc::SendableChooser<frc2::Command*> m_chooser;
    void ConfigureButtonBindings();
    std::unique_ptr<frc2::Command> onTheFly;
    std::unique_ptr<frc2::Command> followOnTheFly;
};
