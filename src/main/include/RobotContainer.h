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
#include <frc2/command/button/POVButton.h>
// #include <frc2/command/button/CommandXboxController.h> //might be something to look into
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <iostream>
#include <string>

#include "subsystems/LimelightSubsystem.h"
#include "../cpp/CommandMessenger.cpp"
#include "../cpp/AutoPaths.cpp"
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/SwerveModuleSubsystem.h"
#include "commands/NoteFollower.h"
#include "commands/AprilTagAim.h"
#include "commands/AutoAprilTag.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "commands/FollowWaypoints.h"
#include "commands/StateMachine.h"
#include "commands/DriveStateMachine.h"
#include "commands/AutoDriveStateMachine.h"
#include "commands/AutoAuxilaryStateMachine.h"
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/ColorSensorSubsystem.h"
#include "commands/DriveStateMachine.h"
#include "commands/IntakeCmd.h"
#include "commands/SpitOutCmd.h"
#include "commands/ManualAim.h"
#include "commands/ShootCommand.h"
#include "commands/AutoShootCommand.h"
#include "subsystems/ShooterWheelsSubsystem.h"

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

    void SetRanAuto(bool ranAuto); 
    float Deadzone(float x); 
    frc2::CommandPtr onFlyGeneration();

    float DeadzoneCubed(float x);
    void ConfigureButtonBindings();
    frc2::CommandPtr GetAuxilaryStateMachine();
    //frc2::CommandPtr GetDriveStateMachine();
    frc2::CommandPtr GetAutoAuxilaryStateMachine();
    frc2::CommandPtr GetAutoDriveStateMachine();

  private:
    //replace with frc::Joystick if using a joystick instead of an xbox controller
    frc::XboxController m_driverController{0};
    frc::XboxController m_auxController{1};

    // The robot's subsystems are defined here...
    DriveSubsystem m_drive;
    LimelightSubsystem m_limelight;
    ShooterSubsystem m_shooter;
    ShooterWheelsSubsystem m_shooterWheels;
    IntakeSubsystem m_intake;
    // ArmSubsystem m_arm;
    ClimberSubsystem m_climb;
    ColorSensorSubsystem m_color;

    frc::SendableChooser<std::string> m_chooser;
    std::string chosenAuto;

    CommandMessenger driveShooterMessager;

    std::vector<AutoPaths::AutoPath> path;


    //Blue auto Paths

    std::vector<frc::Pose2d>  B_1Waypoints{
      // frc::Pose2d(1.45_m, 7_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(0.7_m, 6.65_m, frc::Rotation2d(120_deg)), 
      frc::Pose2d(2_m, 7_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.90_m, 7_m, frc::Rotation2d(180_deg))
    };

    std::vector<units::meters_per_second_t> B_1PointSpeed{
      0_mps,
      1_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_1CruiseSpeed{
      1_mps,
      1_mps,
      1.5_mps
    };

    std::vector<std::string> B_1Command{
      "Null",
      "Null",
      "Intake"
    };

    std::vector<bool> B_1LimelightPath{
      false,
      false,
      false
    };

    std::string B_1EndCommand = "Shoot";
    std::string B_12EndCommand = "NoteFollow"; //temp string to force a state when testing

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

  std::vector<frc::Pose2d>  sideBackupWaypointsLeft{
      frc::Pose2d(1.45_m, 5.55_m, frc::Rotation2d(120_deg)),
      frc::Pose2d(3.5_m, 5.55_m, frc::Rotation2d(120_deg))
    };

    std::vector<units::meters_per_second_t> sideBackupPointSpeedLeft{
      0_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> sideBackupCruiseSpeedLeft{
      1_mps,
      1.5_mps
    };


    std::vector<frc::Pose2d>  sideBackupWaypointsRight{
      frc::Pose2d(1.45_m, 5.55_m, frc::Rotation2d(120_deg)),
      frc::Pose2d(3.5_m, 5.55_m, frc::Rotation2d(120_deg))
    };

    std::vector<units::meters_per_second_t> sideBackupPointSpeedRight{
      0_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> sideBackupCruiseSpeedRight{
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


    std::vector<frc::Pose2d> B_3_6Waypoints1{
      // frc::Pose2d(1.45_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(0.7_m, 4.4_m, frc::Rotation2d(120_deg)), //if this works needs to be changed on all B_3_.... statements.
      frc::Pose2d(2.7_m, 4.1_m, frc::Rotation2d(-180_deg)),
      frc::Pose2d(2.45_m, 4.1_m, frc::Rotation2d(-180_deg))
    };

    std::vector<frc::Pose2d> B_3_6Waypoints2{
      frc::Pose2d(2.45_m, 3_m, frc::Rotation2d(-180_deg)),
      frc::Pose2d(4.6_m, 3.45_m, frc::Rotation2d(-180_deg)),
      frc::Pose2d(6.6_m, 4.1_m, frc::Rotation2d(-180_deg)),
      frc::Pose2d(8.15_m, 4.1_m, frc::Rotation2d(-180_deg))
    };

    std::vector<frc::Pose2d> B_3_6Waypoints3{
      frc::Pose2d(5.6_m, 4.1_m, frc::Rotation2d(-180_deg)),
      frc::Pose2d(4.6_m, 3.5_m, frc::Rotation2d(-180_deg)),
      frc::Pose2d(3_m, 3_m, frc::Rotation2d(-145_deg)),
      frc::Pose2d(2.5_m, 3.5_m, frc::Rotation2d(-145_deg)),
      frc::Pose2d(2.3_m, 3.5_m, frc::Rotation2d(-145_deg)) // auto rotate
    };

    //point speed is the speed you want to be going at the specific waypoint
    std::vector<units::meters_per_second_t> B_3_6PointSpeed1{
      0_mps,
      0_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_3_6PointSpeed2{
      2_mps,
      2_mps,
      1.75_mps,
      0.5_mps
    };

    std::vector<units::meters_per_second_t> B_3_6PointSpeed3{
      2_mps,
      2_mps,
      2_mps,
      1.5_mps,
      0_mps
    };

    //The cruise speed is the speed to get to that waypoint, so for the last item it would be the speed to get to that item.
    std::vector<units::meters_per_second_t> B_3_6CruiseSpeed1{
      1_mps,
      1_mps,
      1_mps
    };

    std::vector<units::meters_per_second_t> B_3_6CruiseSpeed2{
      2_mps,
      2_mps,
      3_mps,
      2_mps
    };

    std::vector<units::meters_per_second_t> B_3_6CruiseSpeed3{
      2_mps,
      2_mps,
      2_mps,
      2_mps,
      1.5_mps
    };

    std::vector<frc::Pose2d> close4Waypoint1{
      frc::Pose2d(1.3_m, 5.55_m, 180_deg),
      frc::Pose2d(2.65_m, 5.55_m, 180_deg)
    };

    std::vector<frc::Pose2d> close4Waypoint2{
      frc::Pose2d(2.65_m, 5.55_m, 180_deg),
      frc::Pose2d(2_m, 6.8_m, 180_deg),
      frc::Pose2d(2.65_m, 7_m, 180_deg)
    };

    std::vector<frc::Pose2d> close4Waypoint3{
      frc::Pose2d(2.65_m, 7_m, 180_deg),
      frc::Pose2d(2_m, 4.3_m, 180_deg),
      frc::Pose2d(2.6_m, 4.1_m, 180_deg),
      frc::Pose2d(2.35_m, 4.1_m, 180_deg)
    };

    std::vector<units::meters_per_second_t> close4PointSpeed1{
      0_mps,
      2_mps
    };

    std::vector<units::meters_per_second_t> close4PointSpeed2{
      1_mps,
      2_mps,
      1.5_mps
    };

    std::vector<units::meters_per_second_t> close4PointSpeed3{
      1_mps,
      1.5_mps,
      0_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> close4CruiseSpeed1{
      2_mps,
      1.5_mps
    };

    std::vector<units::meters_per_second_t> close4CruiseSpeed2{
      1_mps,
      2_mps,
      1.5_mps
    };

    std::vector<units::meters_per_second_t> close4CruiseSpeed3{
      1_mps,
      2_mps,
      1_mps,
      1_mps
    };
};
