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
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <iostream>
#include <string>
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
#include "commands/FollowWaypoints.h"

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
    
  private:
    //replace with frc::Joystick if using a joystick instead of an xbox controller
    frc::XboxController m_driverController{0};
    // The robot's subsystems are defined here...
    DriveSubsystem m_drive;
    LimelightPose m_limePose;
    frc::SendableChooser<std::string> m_chooser;
    void ConfigureButtonBindings();
    std::string chosenAuto;

    //Blue auto Paths







    std::vector<frc::Pose2d>  B_1Waypoints{
      frc::Pose2d(1.45_m, 7_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.90_m, 7_m, frc::Rotation2d(180_deg))
    };

    std::vector<units::meters_per_second_t> B_1PointSpeed{
      0_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_1CruiseSpeed{
      1_mps,
      1.5_mps
    };

    std::vector<frc::Pose2d>  B_2Waypoints{
      frc::Pose2d(1.45_m, 5.55_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.90_m, 5.55_m, frc::Rotation2d(180_deg))
    };

    std::vector<units::meters_per_second_t> B_2PointSpeed{
      0_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_2CruiseSpeed{
      1_mps,
      1.5_mps
    };

    std::vector<frc::Pose2d>  B_3Waypoints{
      frc::Pose2d(1.45_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.90_m, 4.1_m, frc::Rotation2d(180_deg))
    };

    std::vector<units::meters_per_second_t> B_3PointSpeed{
      0_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_3CruiseSpeed{
      1_mps,
      1.5_mps
    };

    std::vector<frc::Pose2d>  B_1_2_3Waypoints{
      frc::Pose2d(1.45_m, 7.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 7.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(1.45_m, 5.55_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 5.55_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(1.45_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 4.1_m, frc::Rotation2d(180_deg))
    };

    std::vector<units::meters_per_second_t> B_1_2_3PointSpeed{
      0_mps,
      0_mps,
      1_mps,
      0_mps,
      1_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_1_2_3CruiseSpeed{
      1_mps,
      2_mps,
      2_mps,
      2_mps,
      2_mps,
      2_mps
    };


    std::vector<frc::Pose2d> B_3_6Waypoints{
      frc::Pose2d(1.45_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.7_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.45_m, 4.1_m, frc::Rotation2d(180_deg)), // auto rotate TODO change later
      frc::Pose2d(2.45_m, 3_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(4.6_m, 3.45_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.6_m, 4.15_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(6.6_m, 4.15_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(8.15_m, 4.15_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.6_m, 4.15_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(4.6_m, 3.5_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(3_m, 3_m, frc::Rotation2d(145_deg)),
    };

    //point speed is the speed you want to be going at the specific waypoint
    std::vector<units::meters_per_second_t> B_3_6PointSpeed{
      0_mps,
      0_mps,
      0_mps,
      2.25_mps,
      2.25_mps,
      2_mps,
      2_mps,
      0.75_mps,
      2.25_mps,
      2.25_mps,
      0_mps,
    };

    //The cruise speed is the speed to get to that waypoint, so for the last item it would be the speed to get to that item.
    std::vector<units::meters_per_second_t> B_3_6CruiseSpeed{
      1_mps,
      1.25_mps,
      1.25_mps,
      2.25_mps,
      2.25_mps,
      2.25_mps,
      3.25_mps,
      2.25_mps,
      2.25_mps,
      2_mps,
      1.5_mps
    };

    std::vector<frc::Pose2d> B_3_7Waypoints{
      frc::Pose2d(1.45_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.5_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.15_m, 1.8_m, frc::Rotation2d(180_deg)), // auto rotate TODO change later
      frc::Pose2d(5.15_m, 1.8_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(6.9_m, 2.35_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(7.7_m, 2.45_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.15_m, 1.8_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.5_m, 3_m, frc::Rotation2d(145_deg)),
      frc::Pose2d(2.35_m, 4_m, frc::Rotation2d(145_deg))
    };

    std::vector<units::meters_per_second_t> B_3_7PointSpeed{
      0_mps,
      0_mps,
      2_mps,
      2_mps,
      1_mps,
      0_mps,
      2_mps,
      1_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_3_7CruiseSpeed{
      1_mps,
      1_mps,
      2_mps,
      2_mps,
      1_mps,
      2_mps,
      2_mps,
      2_mps,
      1.5_mps
    };

    std::vector<frc::Pose2d> B_2_6Waypoints{
      frc::Pose2d(1.45_m, 5.55_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.4_m, 5.55_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(4.0_m, 5.15_m, frc::Rotation2d(180_deg)), // auto rotate TODO change later
      frc::Pose2d(6.25_m, 4.25_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(7.7_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.5_m, 4.25_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(4.1_m, 5.4_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.4_m, 5.55_m, frc::Rotation2d(145_deg)),
      frc::Pose2d(1.4_m, 5.55_m, frc::Rotation2d(145_deg))
    };

    std::vector<units::meters_per_second_t> B_2_6PointSpeed{
      0_mps,
      0_mps,
      2_mps,
      2_mps,
      0_mps,
      2_mps,
      2_mps,
      1_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_2_6CruiseSpeed{
      1_mps,
      2_mps,
      2_mps,
      2_mps,
      1_mps,
      2_mps,
      2_mps,
      1_mps,
      1_mps
    };

std::vector<frc::Pose2d> B_2_7Waypoints{
      frc::Pose2d(1.45_m, 5.55_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.4_m, 5.55_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(4.4_m, 4.9_m, frc::Rotation2d(180_deg)), // auto rotate TODO change later
      frc::Pose2d(5.4_m, 4.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(7.2_m, 2.4_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(7.8_m, 2.4_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.9_m, 4.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(4.4_m, 4.8_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 5.6_m, frc::Rotation2d(145_deg)),
      frc::Pose2d(2.3_m, 5.6_m, frc::Rotation2d(145_deg))
    };

    std::vector<units::meters_per_second_t> B_2_7PointSpeed{
      0_mps,
      0_mps,
      2_mps,
      2_mps,
      1_mps,
      0_mps,
      2_mps,
      2_mps,
      1_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_2_7CruiseSpeed{
      1_mps,
      1_mps,
      2_mps,
      2_mps,
      1_mps,
      2_mps,
      2_mps,
      2_mps,
      1_mps,
      1_mps
    };
    
    std::vector<frc::Pose2d> B_1_4Waypoints{
      frc::Pose2d(1.45_m, 7.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 7.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(6.4_m, 7.5_m, frc::Rotation2d(180_deg)), // auto rotate TODO change later
      frc::Pose2d(7.35_m, 7.5_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.4_m, 7.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.3_m, 7.0_m, frc::Rotation2d(145_deg))
    };

    std::vector<units::meters_per_second_t> B_1_4PointSpeed{
      0_mps,
      0_mps,
      1.5_mps,
      0_mps,
      1_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_1_4CruiseSpeed{
      1_mps,
      1.5_mps,
      4_mps,
      1.5_mps,
      2_mps,
      1.5_mps
    };

    std::vector<frc::Pose2d> B_3_8Waypoints{
      frc::Pose2d(1.45_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.5_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.5_m, 3_m, frc::Rotation2d(180_deg)), // auto rotate TODO change later
      frc::Pose2d(5.2_m, 1.2_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(7.2_m, 0.8_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(7.9_m, 0.8_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.2_m, 1.2_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.5_m, 3_m, frc::Rotation2d(145_deg)),
      frc::Pose2d(2.4_m, 3.6_m, frc::Rotation2d(145_deg))
    };

    std::vector<units::meters_per_second_t> B_3_8PointSpeed{
      0_mps,
      0_mps,
      2_mps,
      2_mps,
      1_mps,
      0_mps,
      2_mps,
      1_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_3_8CruiseSpeed{
      1_mps,
      1_mps,
      2_mps,
      2_mps,
      1_mps,
      1_mps,
      2_mps,
      1_mps,
      1_mps
    };

    std::vector<frc::Pose2d> B_2_5Waypoints{
      frc::Pose2d(1.45_m, 5.55_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.4_m, 5.55_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.25_m, 6.2_m, frc::Rotation2d(180_deg)), // auto rotate TODO change later
      frc::Pose2d(7.3_m, 5.8_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(7.8_m, 5.8_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.8_m, 6.3_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(3.1_m, 5.8_m, frc::Rotation2d(145_deg)),
      frc::Pose2d(2.3_m, 5.6_m, frc::Rotation2d(145_deg))
    };

    std::vector<units::meters_per_second_t> B_2_5PointSpeed{
      0_mps,
      0_mps,
      2_mps,
      1_mps,
      0_mps,
      2_mps,
      1_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_2_5CruiseSpeed{
      1_mps,
      2_mps,
      2_mps,
      1_mps,
      2_mps,
      2_mps,
      1_mps,
      1_mps,
    };

    std::vector<frc::Pose2d> B_3_2_1Waypoints{
      frc::Pose2d(1.45_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(1.5_m, 5.55_m, frc::Rotation2d(180_deg)), // auto rotate TODO change later
      frc::Pose2d(2.9_m, 5.55_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(1.5_m, 7.00_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 7.00_m, frc::Rotation2d(180_deg))
    };

    std::vector<units::meters_per_second_t> B_3_2_1PointSpeed{
      0_mps,
      0_mps,
      1_mps,
      0_mps,
      1_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_3_2_1CruiseSpeed{
      1_mps,
      2_mps,
      2_mps,
      2_mps,
      1_mps
    };

    std::vector<frc::Pose2d> B_1_5Waypoints{
      frc::Pose2d(1.45_m, 7.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 7.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.9_m, 6.4_m, frc::Rotation2d(180_deg)), // auto rotate TODO change later
      frc::Pose2d(8.3_m, 5.8_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.9_m, 6.4_m, frc::Rotation2d(145_deg)),
      frc::Pose2d(1.9_m, 6.3_m, frc::Rotation2d(145_deg))
    };

    std::vector<units::meters_per_second_t> B_1_5PointSpeed{
      0_mps,
      0_mps,
      2_mps,
      0_mps,
      2_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_1_5CruiseSpeed{
      1_mps,
      2_mps,
      2_mps,
      2_mps,
      2_mps,
      2_mps
    };

    std::vector<frc::Pose2d> B_3_7_8Waypoints{
      frc::Pose2d(1.45_m, 4.10_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 4.10_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.9_m, 3.0_m, frc::Rotation2d(180_deg)), // auto rotate TODO change later
      frc::Pose2d(5.30_m, 1.75_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(8.3_m, 2.4_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(8.3_m, 0.80_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.3_m, 1.70_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 3.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(1.9_m, 5.10_m, frc::Rotation2d(180_deg))
    };

    std::vector<units::meters_per_second_t> B_3_7_8PointSpeed{
      0_mps,
      0_mps,
      1_mps,
      2_mps,
      1_mps,
      0_mps,
      2_mps,
      1.5_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_3_7_8CruiseSpeed{
      1_mps,
      1_mps,
      2_mps,
      2_mps,
      1_mps,
      2_mps,
      2_mps,
      1.5_mps,
      1.5_mps
    };

std::vector<frc::Pose2d> B_1_4_5Waypoints{
      frc::Pose2d(1.45_m, 7.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 7.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(8.3_m, 7.4_m, frc::Rotation2d(180_deg)), // auto rotate TODO change later
      frc::Pose2d(5.80_m, 6.5_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(8.3_m, 5.8_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.8_m, 6.5_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(1.9_m, 6.2_m, frc::Rotation2d(180_deg))
    };

    std::vector<units::meters_per_second_t> B_1_4_5PointSpeed{
      0_mps,
      0_mps,
      0_mps,
      2_mps,
      0_mps,
      2_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_1_4_5CruiseSpeed{
      1_mps,
      2_mps,
      1.5_mps,
      2_mps,
      2_mps,
      2_mps,
      2_mps
    };

std::vector<frc::Pose2d> B_2_6_7Waypoints{
      frc::Pose2d(1.45_m, 5.55_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 5.55_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(4.2_m, 4.8_m, frc::Rotation2d(180_deg)), // auto rotate TODO change later
      frc::Pose2d(5.9_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(8.3_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.8_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(4.2_m, 4.9_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.9_m, 3.8_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(8.3_m, 2.4_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.9_m, 4.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(4.3_m, 4.9_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(1.9_m, 5.5_m, frc::Rotation2d(180_deg))
    };

    std::vector<units::meters_per_second_t> B_2_6_7PointSpeed{
      0_mps,
      0_mps,
      2_mps,
      2_mps,
      0_mps,
      2_mps,
      1_mps,
      1.5_mps,
      0_mps,
      2_mps,
      2_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_2_6_7CruiseSpeed{
      1_mps,
      2_mps,
      2_mps,
      1.5_mps,
      2_mps,
      2_mps,
      2_mps,
      2_mps,
      2_mps,
      1.5_mps
    };

std::vector<frc::Pose2d> B_3_6_7Waypoints{
      frc::Pose2d(1.45_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 3.1_m, frc::Rotation2d(180_deg)), // auto rotate TODO change later
      frc::Pose2d(4.3_m, 3.3_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.9_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(8.3_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.9_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(4.3_m, 4.9_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.9_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(8.3_m, 2.5_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.9_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(4.3_m, 4.9_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(1.9_m, 5.1_m, frc::Rotation2d(180_deg))
    };

    std::vector<units::meters_per_second_t> B_3_6_7PointSpeed{
      0_mps,
      0_mps,
      1_mps,
      1_mps,
      2_mps,
      0_mps,
      1.5_mps,
      1_mps,
      1.5_mps,
      0_mps,
      2_mps,
      2_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_3_6_7CruiseSpeed{
      1_mps,
      1_mps,
      1.5_mps,
      2_mps,
      1_mps,
      2_mps,
      1.5_mps,
      2_mps,
      1_mps,
      2_mps,
      2_mps,
      1.5_mps
    };

std::vector<frc::Pose2d> B_2_6_5Waypoints{
      frc::Pose2d(1.45_m, 5.55_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 5.55_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(4.4_m, 4.9_m, frc::Rotation2d(180_deg)), // auto rotate TODO change later
      frc::Pose2d(5.9_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(8.3_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.9_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(4.3_m, 5.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.2_m, 6.4_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(6.8_m, 5.9_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(8.3_m, 5.8_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.8_m, 6.3_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(4.2_m, 6.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(1.9_m, 5.55_m, frc::Rotation2d(180_deg))
    };

    std::vector<units::meters_per_second_t> B_2_6_5PointSpeed{
      0_mps,
      0_mps,
      1_mps,
      2_mps,
      0_mps,
      2_mps,
      1_mps,
      2_mps,
      1.5_mps,
      0_mps,
      2_mps,
      2_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_2_6_5CruiseSpeed{
      1_mps,
      1.5_mps,
      2_mps,
      2_mps,
      2_mps,
      1.5_mps,
      2_mps,
      2_mps,
      1.5_mps,
      2_mps,
      2_mps,
      2_mps
    };

std::vector<frc::Pose2d> B_2_5_6Waypoints{
      frc::Pose2d(1.45_m, 5.55_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 5.55_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.8_m, 6.3_m, frc::Rotation2d(180_deg)), // auto rotate TODO change later
      frc::Pose2d(8.3_m, 5.8_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.8_m, 6.3_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(8.3_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.9_m, 4.2_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(4.2_m, 4.9_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(1.9_m, 5.5_m, frc::Rotation2d(180_deg)),
    };

    std::vector<units::meters_per_second_t> B_2_5_6PointSpeed{
      0_mps,
      0_mps,
      2_mps,
      0_mps,
      2_mps,
      0_mps,
      1.5_mps,
      2_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_2_5_6CruiseSpeed{
      1_mps,
      2_mps,
      2_mps,
      2_mps,
      2_mps,
      1.5_mps,
      2_mps,
      2_mps
    };

std::vector<frc::Pose2d> B_3_2_1_4_5Waypoints{
      frc::Pose2d(1.45_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(1.5_m, 5.55_m, frc::Rotation2d(180_deg)), // auto rotate TODO change later
      frc::Pose2d(2.9_m, 5.55_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(1.5_m, 7.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 7.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(8.3_m, 7.50_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(4.3_m, 6.2_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(8.3_m, 5.8_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(5.9_m, 6.4_m, frc::Rotation2d(180_deg)),   
      frc::Pose2d(4.3_m, 6.4_m, frc::Rotation2d(180_deg)),   
      frc::Pose2d(1.9_m, 5.55_m, frc::Rotation2d(180_deg))
    };

    std::vector<units::meters_per_second_t> B_3_2_1_4_5PointSpeed{
      0_mps,
      0_mps,
      0_mps,
      0_mps,
      0_mps,
      0_mps,
      0_mps,
      1_mps,
      0_mps,
      2_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_3_2_1_4_5CruiseSpeed{
      1_mps,
      2_mps,
      1_mps,
      2_mps,
      1_mps,
      2_mps,
      1.5_mps,
      2_mps,
      2_mps,
      2_mps
    };

    //Red Auto Paths










    std::vector<frc::Pose2d>  R_1Waypoints{
      frc::Pose2d(15.1_m, 7_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.65_m, 7_m, frc::Rotation2d(0_deg))
    };

    std::vector<units::meters_per_second_t> R_1PointSpeed{
      0_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> R_1CruiseSpeed{
      2_mps,
      2_mps
    };

    std::vector<frc::Pose2d>  R_2Waypoints{
      frc::Pose2d(15.1_m, 5.55_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.65_m, 5.55_m, frc::Rotation2d(0_deg))
    };

    std::vector<units::meters_per_second_t> R_2PointSpeed{
      0_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> R_2CruiseSpeed{
      2_mps,
      2_mps
    };

    std::vector<frc::Pose2d>  R_3Waypoints{
      frc::Pose2d(15.1_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.65_m, 4.1_m, frc::Rotation2d(0_deg))
    };

    std::vector<units::meters_per_second_t> R_3PointSpeed{
      0_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> R_3CruiseSpeed{
      2_mps,
      2_mps
    };

    std::vector<frc::Pose2d> R_3_6Waypoints{
      frc::Pose2d(15.1_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.0_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.0_m, 3_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(12.3_m, 3.4_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.75_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(9.5_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(8.9_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.75_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(11.6_m, 3.85_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(12.1_m, 3.2_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.3_m, 3_m, frc::Rotation2d(35_deg)),
      frc::Pose2d(14.0_m, 3_m, frc::Rotation2d(35_deg)) //rotation will be limelights auto adjust
    };

    std::vector<units::meters_per_second_t> R_3_6PointSpeed{
      0_mps,
      0_mps,
      2_mps,
      2_mps,
      2_mps,
      1_mps,
      0_mps,
      2_mps,
      2_mps,
      2_mps,
      1_mps,
      0_mps,
    };

    std::vector<units::meters_per_second_t> R_3_6CruiseSpeed{
      1_mps,
      2_mps,
      2_mps,
      2_mps,
      2_mps,
      1_mps,
      2_mps,
      2_mps,
      2_mps,
      2_mps,
      2_mps,
      1_mps,
    };

    std::vector<frc::Pose2d> R_3_7Waypoints{
      frc::Pose2d(15.1_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.0_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.0_m, 3_m, frc::Rotation2d(0_deg)), // auto rotate TODO change later
      frc::Pose2d(12.25_m, 1.9_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(9.7_m, 2.4_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(8.9_m, 2.4_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(12.25_m, 1.9_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.0_m, 3_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.25_m, 3.6_m, frc::Rotation2d(0_deg))
    };

    std::vector<units::meters_per_second_t> R_3_7PointSpeed{
      0_mps,
      0_mps,
      2_mps,
      2_mps,
      1_mps,
      0_mps,
      2_mps,
      1.5_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> R_3_7CruiseSpeed{
      1_mps,
      2_mps,
      2_mps,
      2_mps,
      1_mps,
      2_mps,
      2_mps,
      2_mps,
      1_mps
    };

    std::vector<frc::Pose2d> R_1_2_3Waypoints{
      frc::Pose2d(15.1_m, 7.0_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.65_m, 7.0_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(15.1_m, 5.55_m, frc::Rotation2d(0_deg)), // auto rotate TODO change later
      frc::Pose2d(13.65_m, 5.55_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(15.1_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.65_m, 4.1_m, frc::Rotation2d(0_deg))
    };

    std::vector<units::meters_per_second_t> R_1_2_3PointSpeed{
      0_mps,
      0_mps,
      0_mps,
      0_mps,
      0_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> R_1_2_3CruiseSpeed{
      1_mps,
      2_mps,
      1_mps,
      2_mps,
      1_mps,
      2_mps
    };

    std::vector<frc::Pose2d> R_2_6Waypoints{
      frc::Pose2d(15.1_m, 5.55_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.15_m, 5.55_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(12.55_m, 5.15_m, frc::Rotation2d(0_deg)), // auto rotate TODO change later
      frc::Pose2d(10.3_m, 4.25_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(8.85_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(11.05_m, 4.25_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(12.45_m, 5.4_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.15_m, 5.55_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(15.15_m, 5.55_m, frc::Rotation2d(0_deg))
    };

    std::vector<units::meters_per_second_t> R_2_6PointSpeed{
      0_mps,
      0_mps,
      2_mps,
      2_mps,
      0_mps,
      2_mps,
      2_mps,
      1_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> R_2_6CruiseSpeed{
      1_mps, 
      2_mps,
      2_mps,
      2_mps,
      1_mps,
      2_mps,
      2_mps,
      1_mps,
      1_mps
    };

    std::vector<frc::Pose2d> R_2_7Waypoints{
      frc::Pose2d(15.1_m, 5.55_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.15_m, 5.55_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(12.15_m, 4.9_m, frc::Rotation2d(0_deg)), // auto rotate TODO change later
      frc::Pose2d(11.15_m, 4.0_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(9.35_m, 2.4_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(8.75_m, 2.4_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.65_m, 4.0_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(12.15_m, 4.8_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.65_m, 5.6_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.25_m, 5.6_m, frc::Rotation2d(0_deg))
    };

    std::vector<units::meters_per_second_t> R_2_7PointSpeed{
      0_mps,
      0_mps,
      2_mps,
      2_mps,
      1_mps,
      0_mps,
      2_mps,
      2_mps,
      1_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> R_2_7CruiseSpeed{
      1_mps,
      1_mps,
      2_mps,
      2_mps,
      1_mps,
      2_mps,
      2_mps,
      2_mps,
      1_mps,
      1_mps
    };

    std::vector<frc::Pose2d> R_3_8Waypoints{
      frc::Pose2d(15.1_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.05_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.05_m, 3_m, frc::Rotation2d(0_deg)), // auto rotate TODO change later
      frc::Pose2d(11.35_m, 1.2_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(9.35_m, 0.8_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(8.65_m, 0.8_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(11.35_m, 1.2_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.05_m, 3_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.15_m, 3.6_m, frc::Rotation2d(0_deg))
    };

    std::vector<units::meters_per_second_t> R_3_8PointSpeed{
      0_mps,
      0_mps,
      2_mps,
      2_mps,
      1_mps,
      0_mps,
      2_mps,
      1_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> R_3_8CruiseSpeed{
      1_mps,
      1_mps,
      2_mps,
      2_mps,
      1_mps,
      1_mps,
      2_mps,
      1_mps,
      1_mps
    };

    std::vector<frc::Pose2d> R_1_4Waypoints{
      frc::Pose2d(15.1_m, 7.0_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.65_m, 7.0_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.15_m, 7.5_m, frc::Rotation2d(0_deg)), // auto rotate TODO change later
      frc::Pose2d(9.2_m, 7.5_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.15_m, 7.0_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.25_m, 7.0_m, frc::Rotation2d(0_deg))
    };

    std::vector<units::meters_per_second_t> R_1_4PointSpeed{
      0_mps,
      0_mps,
      1.5_mps,
      0_mps,
      1_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> R_1_4CruiseSpeed{
      1_mps,
      1.5_mps,
      4_mps,
      1.5_mps,
      2_mps,
      1.5_mps
    };

    std::vector<frc::Pose2d> R_2_5Waypoints{
      frc::Pose2d(15.1_m, 5.55_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.15_m, 5.55_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(11.3_m, 6.2_m, frc::Rotation2d(0_deg)), // auto rotate TODO change later
      frc::Pose2d(9.25_m, 5.8_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(8.75_m, 5.8_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.75_m, 6.3_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.45_m, 5.8_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.25_m, 5.6_m, frc::Rotation2d(0_deg))
    };

    std::vector<units::meters_per_second_t> R_2_5PointSpeed{
      0_mps,
      0_mps,
      2_mps,
      1_mps,
      0_mps,
      2_mps,
      1_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> R_2_5CruiseSpeed{
      1_mps,
      2_mps,
      2_mps,
      1_mps,
      2_mps,
      2_mps,
      1_mps,
      1_mps,
    };

    std::vector<frc::Pose2d> R_3_2_1Waypoints{
      frc::Pose2d(15.1_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.65_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(15.1_m, 5.55_m, frc::Rotation2d(0_deg)), // auto rotate TODO change later
      frc::Pose2d(13.65_m, 5.55_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(15.1_m, 7.00_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.65_m, 7.00_m, frc::Rotation2d(0_deg))
    };

    std::vector<units::meters_per_second_t> R_3_2_1PointSpeed{
      0_mps,
      0_mps,
      1_mps,
      0_mps,
      1_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> R_3_2_1CruiseSpeed{
      1_mps,
      2_mps,
      2_mps,
      2_mps,
      1_mps
    };

    std::vector<frc::Pose2d> R_1_5Waypoints{
      frc::Pose2d(15.1_m, 7.0_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.65_m, 7.0_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.65_m, 6.4_m, frc::Rotation2d(0_deg)), // auto rotate TODO change later
      frc::Pose2d(8.25_m, 5.8_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.65_m, 6.4_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.65_m, 6.3_m, frc::Rotation2d(0_deg))
    };

    std::vector<units::meters_per_second_t> R_1_5PointSpeed{
      0_mps,
      0_mps,
      2_mps,
      0_mps,
      2_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> R_1_5CruiseSpeed{
      1_mps,
      2_mps,
      2_mps,
      2_mps,
      2_mps,
      2_mps
    };

std::vector<frc::Pose2d> R_3_7_8Waypoints{
      frc::Pose2d(15.1_m, 4.10_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.65_m, 4.10_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.65_m, 3.0_m, frc::Rotation2d(0_deg)), // auto rotate TODO change later
      frc::Pose2d(11.25_m, 1.75_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(8.25_m, 2.4_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(8.25_m, 0.80_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(11.25_m, 1.70_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.65_m, 3.0_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.65_m, 5.10_m, frc::Rotation2d(0_deg))
    };

    std::vector<units::meters_per_second_t> R_3_7_8PointSpeed{
      0_mps,
      0_mps,
      1_mps,
      2_mps,
      1_mps,
      0_mps,
      2_mps,
      1.5_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> R_3_7_8CruiseSpeed{
      1_mps,
      1_mps,
      2_mps,
      2_mps,
      1_mps,
      2_mps,
      2_mps,
      1.5_mps,
      1.5_mps
    };

    std::vector<frc::Pose2d> R_1_4_5Waypoints{
      frc::Pose2d(15.1_m, 7.0_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.65_m, 7.0_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(8.25_m, 7.4_m, frc::Rotation2d(0_deg)), // auto rotate TODO change later
      frc::Pose2d(10.75_m, 6.5_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(8.25_m, 5.8_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.75_m, 6.5_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.65_m, 6.2_m, frc::Rotation2d(0_deg))
    };

    std::vector<units::meters_per_second_t> R_1_4_5PointSpeed{
      0_mps,
      0_mps,
      0_mps,
      2_mps,
      0_mps,
      2_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> R_1_4_5CruiseSpeed{
      1_mps,
      2_mps,
      1.5_mps,
      2_mps,
      2_mps,
      2_mps,
      2_mps
    };

    std::vector<frc::Pose2d> R_2_6_7Waypoints{
      frc::Pose2d(15.1_m, 5.55_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.65_m, 5.55_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(12.35_m, 4.8_m, frc::Rotation2d(0_deg)), // auto rotate TODO change later
      frc::Pose2d(10.65_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(8.25_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.75_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(12.35_m, 4.9_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.65_m, 3.8_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(8.25_m, 2.4_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.65_m, 4.0_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(12.25_m, 4.9_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.65_m, 5.5_m, frc::Rotation2d(0_deg))
    };

    std::vector<units::meters_per_second_t> R_2_6_7PointSpeed{
      0_mps,
      0_mps,
      2_mps,
      2_mps,
      0_mps,
      2_mps,
      1_mps,
      1.5_mps,
      0_mps,
      2_mps,
      2_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> R_2_6_7CruiseSpeed{
      1_mps,
      2_mps,
      2_mps,
      1.5_mps,
      2_mps,
      2_mps,
      2_mps,
      2_mps,
      2_mps,
      1.5_mps
    };

    std::vector<frc::Pose2d> R_3_6_7Waypoints{
      frc::Pose2d(15.1_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.65_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.65_m, 3.1_m, frc::Rotation2d(0_deg)), // auto rotate TODO change later
      frc::Pose2d(12.25_m, 3.3_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.65_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(8.25_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.65_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(12.25_m, 4.9_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.65_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(8.25_m, 2.5_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.65_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(12.25_m, 4.9_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.65_m, 5.1_m, frc::Rotation2d(0_deg))
    };

    std::vector<units::meters_per_second_t> R_3_6_7PointSpeed{
      0_mps,
      0_mps,
      1_mps,
      1_mps,
      2_mps,
      0_mps,
      1.5_mps,
      1_mps,
      1.5_mps,
      0_mps,
      2_mps,
      2_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> R_3_6_7CruiseSpeed{
      1_mps,
      1_mps,
      1.5_mps,
      2_mps,
      1_mps,
      2_mps,
      1.5_mps,
      2_mps,
      1_mps,
      2_mps,
      2_mps,
      1.5_mps
    };

    std::vector<frc::Pose2d> R_2_6_5Waypoints{
      frc::Pose2d(15.1_m, 5.55_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.65_m, 5.55_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(12.15_m, 4.9_m, frc::Rotation2d(0_deg)), // auto rotate TODO change later
      frc::Pose2d(10.65_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(8.25_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.65_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(12.25_m, 5.0_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(11.35_m, 6.4_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(9.75_m, 5.9_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(8.25_m, 5.8_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.75_m, 6.3_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(12.35_m, 6.0_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.65_m, 5.55_m, frc::Rotation2d(0_deg))
    };

    std::vector<units::meters_per_second_t> R_2_6_5PointSpeed{
      0_mps,
      0_mps,
      1_mps,
      2_mps,
      0_mps,
      2_mps,
      1_mps,
      2_mps,
      1.5_mps,
      0_mps,
      2_mps,
      2_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> R_2_6_5CruiseSpeed{
      1_mps,
      1.5_mps,
      2_mps,
      2_mps,
      2_mps,
      1.5_mps,
      2_mps,
      2_mps,
      1.5_mps,
      2_mps,
      2_mps,
      2_mps
    };

    std::vector<frc::Pose2d> R_2_5_6Waypoints{
      frc::Pose2d(15.1_m, 5.55_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.65_m, 5.55_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.75_m, 6.3_m, frc::Rotation2d(0_deg)), // auto rotate TODO change later
      frc::Pose2d(8.25_m, 5.8_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.75_m, 6.3_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(8.25_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.65_m, 4.2_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(12.35_m, 4.9_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(14.65_m, 5.5_m, frc::Rotation2d(0_deg)),
    };

    std::vector<units::meters_per_second_t> R_2_5_6PointSpeed{
      0_mps,
      0_mps,
      2_mps,
      0_mps,
      2_mps,
      0_mps,
      1.5_mps,
      2_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> R_2_5_6CruiseSpeed{
      1_mps,
      2_mps,
      2_mps,
      2_mps,
      2_mps,
      1.5_mps,
      2_mps,
      2_mps
    };

    std::vector<frc::Pose2d> R_3_2_1_4_5Waypoints{
      frc::Pose2d(15.1_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.65_m, 4.1_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(15.1_m, 5.55_m, frc::Rotation2d(0_deg)), // auto rotate TODO change later
      frc::Pose2d(13.65_m, 5.55_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(15.1_m, 7.0_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(13.65_m, 7.0_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(8.25_m, 7.50_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(12.25_m, 6.2_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(8.25_m, 5.8_m, frc::Rotation2d(0_deg)),
      frc::Pose2d(10.65_m, 6.4_m, frc::Rotation2d(0_deg)),   
      frc::Pose2d(12.25_m, 6.4_m, frc::Rotation2d(0_deg)),   
      frc::Pose2d(14.65_m, 5.55_m, frc::Rotation2d(0_deg))
    };

    std::vector<units::meters_per_second_t> R_3_2_1_4_5PointSpeed{
      0_mps,
      0_mps,
      0_mps,
      0_mps,
      0_mps,
      0_mps,
      0_mps,
      1_mps,
      0_mps,
      2_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> R_3_2_1_4_5CruiseSpeed{
      1_mps,
      2_mps,
      1_mps,
      2_mps,
      1_mps,
      2_mps,
      1.5_mps,
      2_mps,
      2_mps,
      2_mps
    };
};