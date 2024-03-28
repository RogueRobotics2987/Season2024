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
#include "subsystems/ClimberSubsystem.h"
#include "commands/IntakeCmd.h"
#include "commands/SpitOutCmd.h"
#include "commands/ManualAim.h"
#include "commands/ShootCommand.h"
#include "commands/AutoShootCommand.h"
#include "subsystems/ShooterWheelsSubsystem.h"
#include "commands/AmpCommand.h"
#include "commands/AmpShooter.h"
#include "commands/AmpLineup.h"
#include "commands/AutoShooterWarmupCmd.h"
#include "commands/AutoSubAim.h"
#include "commands/ShooterLobCommand.h"

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

    float Deadzone(float x); 
    float DeadzoneCubed(float x);
    void ConfigureButtonBindings();
    void ZeroHeading();
    void ShooterOff();

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
    ArmSubsystem m_arm;
    ClimberSubsystem m_climb;

    frc::SendableChooser<std::string> m_chooser;
    std::string chosenAuto;
    std::vector<AutoPaths::AutoPath> path;

    //CURRENT PATHS
    //RED - 4 NOTE
    //RED - SOURCE
    //RED - AMP

    //BLUE - 4 NOTE
    //BLUE - SOURCE
    //BLUE - AMP


    /*
    //Blue 1
    std::vector<frc::Pose2d>  B_1Waypoints
    {
      frc::Pose2d(0.7_m, 6.65_m, frc::Rotation2d(120_deg)), 
      frc::Pose2d(2_m, 7_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.90_m, 7_m, frc::Rotation2d(180_deg))
    };

    std::vector<units::meters_per_second_t> B_1PointSpeed
    {
      0_mps,
      1_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_1CruiseSpeed
    {
      1_mps,
      1_mps,
      1.5_mps
    };

    std::vector<std::string> B_1Command
    {
      "Null",
      "Null",
      "Intake"
    };

    std::vector<bool> B_1LimelightPath
    {
      false,
      false,
      false
    };

    std::string B_1EndCommand = "Shoot";
    std::string B_12EndCommand = "NoteFollow"; //temp string to force a state when testing

    */

   /*BLUE - 2

    std::vector<frc::Pose2d>  B_2Waypoints
    {
      frc::Pose2d(1.45_m, 5.55_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.90_m, 5.55_m, frc::Rotation2d(180_deg))
    };

    std::vector<units::meters_per_second_t> B_2PointSpeed
    {
      0_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_2CruiseSpeed
    {
      1_mps,
      1.5_mps
    };

    */

   /* SIDE BACKUP

    std::vector<frc::Pose2d>  sideBackupWaypointsLeft
    {
      frc::Pose2d(1.45_m, 5.55_m, frc::Rotation2d(120_deg)),
      frc::Pose2d(3.5_m, 5.55_m, frc::Rotation2d(120_deg))
    };

    std::vector<units::meters_per_second_t> sideBackupPointSpeedLeft
    {
      0_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> sideBackupCruiseSpeedLeft
    {
      1_mps,
      1.5_mps
    };

    std::vector<frc::Pose2d>  sideBackupWaypointsRight
    {
      frc::Pose2d(1.45_m, 5.55_m, frc::Rotation2d(120_deg)),
      frc::Pose2d(3.5_m, 5.55_m, frc::Rotation2d(120_deg))
    };

    std::vector<units::meters_per_second_t> sideBackupPointSpeedRight
    {
      0_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> sideBackupCruiseSpeedRight
    {
      1_mps,
      1.5_mps
    };

    */


    /* BLUE - 3
    std::vector<frc::Pose2d>  B_3Waypoints
    {
      frc::Pose2d(1.45_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.90_m, 4.1_m, frc::Rotation2d(180_deg))
    };

    std::vector<units::meters_per_second_t> B_3PointSpeed
    {
      0_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_3CruiseSpeed
    {
      1_mps,
      1.5_mps
    };

    */

    /* BLUE - 1 2 3
    std::vector<frc::Pose2d>  B_1_2_3Waypoints
    {
      frc::Pose2d(1.45_m, 7.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 7.0_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(1.45_m, 5.55_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 5.55_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(1.45_m, 4.1_m, frc::Rotation2d(180_deg)),
      frc::Pose2d(2.9_m, 4.1_m, frc::Rotation2d(180_deg))
    };

    std::vector<units::meters_per_second_t> B_1_2_3PointSpeed
    {
      0_mps,
      0_mps,
      1_mps,
      0_mps,
      1_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_1_2_3CruiseSpeed
    {
      1_mps,
      2_mps,
      2_mps,
      2_mps,
      2_mps,
      2_mps
    };

    */

    /* BLUE - 3 6
    std::vector<frc::Pose2d> B_3_6Waypoints1
    {
      frc::Pose2d(0.7_m, 4.4_m, frc::Rotation2d(120_deg)), //if this works needs to be changed on all B_3_.... statements.
      frc::Pose2d(2.7_m, 4.1_m, frc::Rotation2d(-180_deg)),
      frc::Pose2d(2.45_m, 4.1_m, frc::Rotation2d(-180_deg))
    };

    std::vector<frc::Pose2d> B_3_6Waypoints2
    {
      frc::Pose2d(2.45_m, 3_m, frc::Rotation2d(-180_deg)),
      frc::Pose2d(4.6_m, 3.45_m, frc::Rotation2d(-180_deg)),
      frc::Pose2d(6.6_m, 4.1_m, frc::Rotation2d(-180_deg)),
      frc::Pose2d(8.15_m, 4.1_m, frc::Rotation2d(-180_deg))
    };

    std::vector<frc::Pose2d> B_3_6Waypoints3
    {
      frc::Pose2d(5.6_m, 4.1_m, frc::Rotation2d(-180_deg)),
      frc::Pose2d(4.6_m, 3.5_m, frc::Rotation2d(-180_deg)),
      frc::Pose2d(3_m, 3_m, frc::Rotation2d(-145_deg)),
      frc::Pose2d(2.5_m, 3.5_m, frc::Rotation2d(-145_deg)),
      frc::Pose2d(2.3_m, 3.5_m, frc::Rotation2d(-145_deg)) // auto rotate
    };

    //point speed is the speed you want to be going at the specific waypoint
    std::vector<units::meters_per_second_t> B_3_6PointSpeed1
    {
      0_mps,
      0_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> B_3_6PointSpeed2
    {
      2_mps,
      2_mps,
      1.75_mps,
      0.5_mps
    };

    std::vector<units::meters_per_second_t> B_3_6PointSpeed3
    {
      2_mps,
      2_mps,
      2_mps,
      1.5_mps,
      0_mps
    };

    //The cruise speed is the speed to get to that waypoint, so for the last item it would be the speed to get to that item.
    std::vector<units::meters_per_second_t> B_3_6CruiseSpeed1
    {
      1_mps,
      1_mps,
      1_mps
    };

    std::vector<units::meters_per_second_t> B_3_6CruiseSpeed2
    {
      2_mps,
      2_mps,
      3_mps,
      2_mps
    };

    std::vector<units::meters_per_second_t> B_3_6CruiseSpeed3
    {
      2_mps,
      2_mps,
      2_mps,
      2_mps,
      1.5_mps
    };

    */

   //THESE ARE THE TEST POINTS FOR JUSTSHOOTCENTER OUR TEST PATH
   std::vector<frc::Pose2d> close4Waypoint1JustShootCenter
    {
      frc::Pose2d(1.3_m, 5.55_m, 180_deg),
      frc::Pose2d(2.65_m, 5.55_m, 180_deg)
    };

    std::vector<units::meters_per_second_t> close4PointSpeed1JustShootCenter
    {
      0.5_mps,
      2_mps
    };

    std::vector<units::meters_per_second_t> close4CruiseSpeed1JustShootCenter
    {
      2.5_mps,
      1.5_mps
    };
    
    std::vector<frc::Pose2d> RedTest
    {
      frc::Pose2d(15.25_m, 5.55_m, 0_deg),
      frc::Pose2d(13.9_m, 5.55_m, 0_deg)
    };

    std::vector<units::meters_per_second_t> RedTestPointSpeed
    {
      0.5_mps,
      2_mps
    };

    std::vector<units::meters_per_second_t> RedTestCruiseSpeed
    {
      2.5_mps,
      1.5_mps
    };
    //END TEST
    

    // BLUE FRONT 4 NOTE
    std::vector<frc::Pose2d> Blue_close4Waypoint1
    {
      frc::Pose2d(1.3_m, 5.55_m, 180_deg),
      frc::Pose2d(2.65_m, 5.55_m, 180_deg)
    };

    std::vector<frc::Pose2d> Blue_close4Waypoint2
    {
      frc::Pose2d(2.65_m, 5.55_m, 180_deg),
      frc::Pose2d(2.7_m, 6.95_m, 230_deg)
    };

    std::vector<frc::Pose2d> Blue_close4Waypoint3
    {
      frc::Pose2d(2.7_m, 6.95_m, 230_deg),
      frc::Pose2d(2.2_m, 4.3_m, 180_deg),
      frc::Pose2d(2.6_m, 4.1_m, 180_deg),
      frc::Pose2d(2.35_m, 4.1_m, 130_deg)
    };

    std::vector<units::meters_per_second_t> Blue_close4PointSpeed1
    {
      0.5_mps,
      2_mps
    };

    std::vector<units::meters_per_second_t> Blue_close4PointSpeed2
    {
      1.5_mps,
      1.25_mps
    };

    std::vector<units::meters_per_second_t> Blue_close4PointSpeed3
    {
      1_mps,
      2_mps,
      1_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> Blue_close4CruiseSpeed1
    {
      2.5_mps,
      1.5_mps
    };

    std::vector<units::meters_per_second_t> Blue_close4CruiseSpeed2
    {
      1.5_mps,
      1.5_mps
    };

    std::vector<units::meters_per_second_t> Blue_close4CruiseSpeed3
    {
      1.5_mps,
      2_mps,
      1_mps,
      0.5_mps
    };





    // RED CLOSE 4 NOTE - TODO: RED SIDE NEEDS TO BE TESTED
    std::vector<frc::Pose2d> Red_Close4Waypoint1
    {
      frc::Pose2d(14.7_m, 5.55_m, 0_deg),
      frc::Pose2d(13.35_m, 5.55_m, 0_deg)
    };

    std::vector<frc::Pose2d> Red_Close4Waypoint2
    {
      frc::Pose2d(13.3_m, 7.05_m, -60_deg)
    };

    std::vector<frc::Pose2d> Red_Close4Waypoint3
    {
      frc::Pose2d(13.8_m, 4.3_m, 0_deg),
      frc::Pose2d(13.2_m, 4.1_m, 0_deg),
      frc::Pose2d(13.65_m, 4.1_m, 50_deg)
    };

    std::vector<units::meters_per_second_t> Red_Close4PointSpeed1
    {
      0.5_mps,
      2_mps
    };

    std::vector<units::meters_per_second_t> Red_Close4PointSpeed2
    {
      1.5_mps,
      1.5_mps,
      1.25_mps
    };

    std::vector<units::meters_per_second_t> Red_Close4PointSpeed3
    {
      1_mps,
      2_mps,
      1_mps,
      0_mps
    };

    std::vector<units::meters_per_second_t> Red_Close4CruiseSpeed1
    {
      2.5_mps,
      1.5_mps
    };

    std::vector<units::meters_per_second_t> Red_Close4CruiseSpeed2
    {
      1.5_mps,
      1.5_mps,
      1.5_mps
    };

    std::vector<units::meters_per_second_t> Red_Close4CruiseSpeed3
    {
      1.5_mps,
      3.5_mps,
      1_mps,
      0.5_mps
    };


    //BLUE SOURCE 3 NOTE
    //Start to first shooting spot
    std::vector<frc::Pose2d> Blue_FarSideMid1
    {
      frc::Pose2d(0.45_m, 2.5_m, 180_deg),
      frc::Pose2d(2.5_m, 2.5_m, 130_deg)
    };

    //Post first shoot to midline pickup
    std::vector<frc::Pose2d> Blue_FarSideMid2
    {
      frc::Pose2d(2.75_m, 2.5_m, 130_deg),
      frc::Pose2d(8.25_m, 0.85_m, 160_deg)
    };

    //Midline pickup to 2nd shooting spot
    std::vector<frc::Pose2d> Blue_FarSideMid3
    {
      frc::Pose2d(8.05_m, 0.85_m, 160_deg),
      frc::Pose2d(5.5_m, 1.8_m, 180_deg),
      frc::Pose2d(4.0_m, 3.0_m, 140_deg)
    };
    
    //Post 2nd shot around pillar to 3rd note pickup
    std::vector<frc::Pose2d> Blue_FarSideMid4
    {
      frc::Pose2d(3.9_m, 2.25_m, 130_deg),
      frc::Pose2d(5.5_m, 1.8_m, 180_deg),
      frc::Pose2d(8.25_m, 2.45_m, 200_deg)
    };

    //3rd note pickup to 3rd not shooting spot
    std::vector<frc::Pose2d> Blue_FarSideMid5
    {
      frc::Pose2d(8.05_m, 2.45_m, 200_deg),
      frc::Pose2d(6.0_m, 1.6_m, 180_deg),
      frc::Pose2d(4.0_m, 3.0_m, 130_deg)

      //Potential cut through middle stage for 3rd note.
      //frc::Pose2d(5.6_m, 4.0_m, 160_deg),
      //frc::Pose2d(4.0_m, 5.55_m, 200_deg)
    };

    std::vector<units::velocity::meters_per_second_t> Blue_FarSideMidPoint1
    {
      1_mps,
      0_mps
    };

    std::vector<units::velocity::meters_per_second_t> Blue_FarSideMidPoint2
    {
      1_mps,
      0_mps
    };

    
    std::vector<units::velocity::meters_per_second_t> Blue_FarSideMidPoint3
    {
      1_mps,
      2_mps,
      0_mps
    };

    
    std::vector<units::velocity::meters_per_second_t> Blue_FarSideMidPoint4
    {
      1_mps,
      2_mps,
      0_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Blue_FarSideMidPoint5
    {
      1_mps,
      3_mps,
      0_mps
    };


    std::vector<units::velocity::meters_per_second_t> Blue_FarSideMidCruise1
    {
      1_mps,
      3_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Blue_FarSideMidCruise2
    {
      1_mps,
      3_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Blue_FarSideMidCruise3
    {
      1_mps,
      3_mps,
      3_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Blue_FarSideMidCruise4
    {
      1_mps,
      3_mps,
      2_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Blue_FarSideMidCruise5
    {
      1_mps,
      3_mps,
      3_mps
    };

    std::vector<frc::Pose2d> Blue_AmpSide1
    {
      frc::Pose2d(0.45_m, 7_m, 180_deg),
      frc::Pose2d(1.5_m, 7_m, -160_deg)
    };

    std::vector<frc::Pose2d> Blue_AmpSide2
    {
      frc::Pose2d(1.5_m, 7_m, -160_deg),
      frc::Pose2d(2.9_m, 7_m, 180_deg)
    };

    std::vector<frc::Pose2d> Blue_AmpSide3
    {
      frc::Pose2d(2.9_m, 7_m, 180_deg),
      frc::Pose2d(7_m, 7_m, 180_deg),
      frc::Pose2d(8.25_m, 7.35_m, -150_deg)
    };

    std::vector<frc::Pose2d> Blue_AmpSide4
    {
      frc::Pose2d(8.15_m, 7.35_m, -150_deg),
      frc::Pose2d(5.3_m, 7_m, 180_deg)
    };

    //for a fourth note
    // std::vector<frc::Pose2d> Blue_AmpSide5
    // {
    //   frc::Pose2d(5.3_m, 7_m, 180_deg),
    //   frc::Pose2d(6.5_m, 6.6_m, 160_deg),
    //   frc::Pose2d(7.9_m, 5.95_m, 160_deg)
    // };

    // std::vector<frc::Pose2d> Blue_AmpSide6
    // {
    //   frc::Pose2d(7.9_m, 5.95_m, 160_deg),
    //   frc::Pose2d(6.5_m, 6.6_m, 180_deg),
    //   frc::Pose2d(5.3_m, 7_m, 180_deg)
    // };

    std::vector<units::velocity::meters_per_second_t> Blue_AmpSidePoint1
    {
      1_mps,
      0_mps
    };

    std::vector<units::velocity::meters_per_second_t> Blue_AmpSidePoint2
    {
      1_mps,
      2_mps
    };

    std::vector<units::velocity::meters_per_second_t> Blue_AmpSidePoint3
    {
      1_mps,
      2_mps,
      1_mps
    };

    std::vector<units::velocity::meters_per_second_t> Blue_AmpSidePoint4
    {
      1_mps,
      0_mps
    };

    std::vector<units::velocity::meters_per_second_t> Blue_AmpSidePoint5
    {
      1_mps,
      3_mps,
      1_mps
    };

    std::vector<units::velocity::meters_per_second_t> Blue_AmpSidePoint6
    {
      1_mps,
      3_mps,
      1_mps
    };

    std::vector<units::velocity::meters_per_second_t> Blue_AmpSideCruise1
    {
      1_mps,
      2_mps
    };

    std::vector<units::velocity::meters_per_second_t> Blue_AmpSideCruise2
    {
      1_mps,
      2_mps,
    };
    
    std::vector<units::velocity::meters_per_second_t> Blue_AmpSideCruise3
    {
      1_mps,
      3_mps,
      2_mps
    };

    std::vector<units::velocity::meters_per_second_t> Blue_AmpSideCruise4
    {
      1_mps,
      3_mps
    };

    std::vector<units::velocity::meters_per_second_t> Blue_AmpSideCruise5
    {
      1_mps,
      3_mps,
      2_mps
    };

    std::vector<units::velocity::meters_per_second_t> Blue_AmpSideCruise6
    {
      1_mps,
      3_mps,
      2_mps
    };

    /*********************************************************************************/
    
    //TODO: 
    //RED SOURCE 3 NOTE POSE VALUES HAVE NOT BEEN SET
    //IN FACT - THEY AREN'T EVEN CONSISTENT
    //DO NOT RUN THIS UNDER ANY CIRCUMSTANCE
    //WITHOUT FIRST INPUTTING THE CORRECT POSE WAYPOINT VALUES

    /*********************************************************************************/

    //RED SOURCE 3 NOTE
    //Start to first shooting spot
    std::vector<frc::Pose2d> Red_FarSideMid1
    {
      frc::Pose2d(16.1_m, 2.5_m, 0_deg),
      frc::Pose2d(14.05_m, 2.5_m, 50_deg)
    };

    //Post first shoot to midline pickup
    std::vector<frc::Pose2d> Red_FarSideMid2
    {
      frc::Pose2d(13.8_m, 2.5_m, 50_deg),
      frc::Pose2d(8.3_m, 1.05_m, 20_deg)
    };

    //Midline pickup to 2nd shooting spot
    std::vector<frc::Pose2d> Red_FarSideMid3
    {
      frc::Pose2d(8.5_m, 1.05_m, 20_deg),
      frc::Pose2d(11.05_m, 1.8_m, 0_deg),
      frc::Pose2d(12.55_m, 3.0_m, 50_deg)
    };
    
    //Post 2nd shot around pillar to 3rd note pickup
    std::vector<frc::Pose2d> Red_FarSideMid4
    {
      frc::Pose2d(12.65_m, 2.25_m, 50_deg),
      frc::Pose2d(11.05_m, 1.8_m, 0_deg),
      frc::Pose2d(8.3_m, 2.45_m, -20_deg)
    };

    //3rd note pickup to 3rd not shooting spot
    std::vector<frc::Pose2d> Red_FarSideMid5
    {
      frc::Pose2d(8.5_m, 2.45_m, -20_deg),
      frc::Pose2d(10.55_m, 1.6_m, 0_deg),
      frc::Pose2d(12.5_m, 3.0_m, 50_deg)

      //Potential cut through middle stage for 3rd note.
      //frc::Pose2d(5.6_m, 4.0_m, 160_deg),
      //frc::Pose2d(4.0_m, 5.55_m, 200_deg)
    };

    std::vector<units::velocity::meters_per_second_t> Red_FarSideMidPoint1
    {
      1_mps,
      0_mps
    };

    std::vector<units::velocity::meters_per_second_t> Red_FarSideMidPoint2
    {
      1_mps,
      0_mps
    };

    
    std::vector<units::velocity::meters_per_second_t> Red_FarSideMidPoint3
    {
      1_mps,
      2_mps,
      0_mps
    };

    
    std::vector<units::velocity::meters_per_second_t> Red_FarSideMidPoint4
    {
      1_mps,
      2_mps,
      0_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Red_FarSideMidPoint5
    {
      1_mps,
      3_mps,
      0_mps
    };


    std::vector<units::velocity::meters_per_second_t> Red_FarSideMidCruise1
    {
      1_mps,
      3_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Red_FarSideMidCruise2
    {
      1_mps,
      3_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Red_FarSideMidCruise3
    {
      1_mps,
      3_mps,
      3_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Red_FarSideMidCruise4
    {
      1_mps,
      3_mps,
      2_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Red_FarSideMidCruise5
    {
      1_mps,
      3_mps,
      3_mps
    };
};
