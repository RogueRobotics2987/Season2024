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
    // frc::SendableChooser<frc2::Command*> m_chooser;
    void ConfigureButtonBindings();
    std::unique_ptr<frc2::Command> onTheFly;
    std::unique_ptr<frc2::Command> followOnTheFly;

    //Blue auto Paths

    std::vector<frc::Pose2d> B_3_6Waypoints{
      frc::Pose2d(1.5_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.7_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.45_m, 4.1_m, frc::Rotation2d(180_deg)), // auto rotate TODO change later
      frc::Pose2d(2.45_m, 3_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(4.6_m, 3.35_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(6.6_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(8.15_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(4.6_m, 3.35_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.45_m, 3_m, frc::Rotation2d(145_deg)),
      frc::Pose2d(2.35_m, 3.6_m, frc::Rotation2d(145_deg)),
      frc::Pose2d(2.2_m, 4.1_m, frc::Rotation2d(145_deg)) // auto rotate
    };

    //point speed is the speed you want to be going at the specific waypoint
    std::vector<units::meters_per_second_t> B_3_6PointSpeed{
      0_mps,
      0_mps,
      0_mps,
      1_mps,
      2_mps,
      0.75_mps,
      0_mps,
      2_mps,
      1.5_mps,
      1_mps,
      0_mps
    };

    //The cruise speed is the speed to get to that waypoint, so for the last item it would be the speed to get to that item.
    std::vector<units::meters_per_second_t> B_3_6CruiseSpeed{
      1_mps,
      1_mps,
      2_mps,
      2_mps,
      2_mps,
      2_mps,
      1_mps,
      2_mps,
      2_mps,
      2_mps,
      1_mps
    };

    std::vector<frc::Pose2d> B_3_7Waypoints{
      frc::Pose2d(1.5_m, 4.1_m, frc::Rotation2d(180_deg)),
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
      frc::Pose2d(1.5_m, 5.55_m, frc::Rotation2d(180_deg)),
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
      frc::Pose2d(1.5_m, 5.55_m, frc::Rotation2d(180_deg)),
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
      frc::Pose2d(1.5_m, 7.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 7.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(6.4_m, 7.5_m, frc::Rotation2d(180_deg)), // auto rotate TODO change later
      frc::Pose2d(7.35_m, 7.5_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.4_m, 7.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.3_m, 7.0_m, frc::Rotation2d(145_deg))
    };

    std::vector<units::meters_per_second_t> B_1_4PointSpeed{
      0_mps,
      0_mps,
      3_mps,
      0_mps,
      1_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_1_4CruiseSpeed{
      1_mps,
      1.5_mps,
      3_mps,
      1.5_mps,
      3_mps,
      1_mps
    };

    std::vector<frc::Pose2d> B_3_8Waypoints{
      frc::Pose2d(1.5_m, 4.1_m, frc::Rotation2d(180_deg)),
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
      frc::Pose2d(1.5_m, 5.55_m, frc::Rotation2d(180_deg)),
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

    //Red Auto Paths

    std::vector<frc::Pose2d> R_3_6Waypoints{
      frc::Pose2d(15.05_m, 4.1_m, frc::Rotation2d(0_deg)),
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
      frc::Pose2d(15.05_m, 4.1_m, frc::Rotation2d(0_deg)),
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
};
