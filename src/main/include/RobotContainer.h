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
#include <frc2/command/ConditionalCommand.h>
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
#include "commands/AutoNotePickup.h"
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
#include "subsystems/LightSubsystem.h"
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
    void LightsOff();

    LightSubsystem m_lights;
    
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
    //BLUE - 4 NOTE
    //BLUE - SOURCE
    //BLUE - AMP
    //BLUE - MIDLINE 4


    //RED - 4 NOTE
    //RED - SOURCE
    //RED - AMP
    //RED - MIDLINE 4


    // BLUE FRONT 4 NOTE
    std::vector<frc::Pose2d> Blue_close4Waypoint1
    {
      frc::Pose2d(1.3_m, 5.55_m, 180_deg),
      frc::Pose2d(2.65_m, 5.55_m, 180_deg)
    };

    std::vector<frc::Pose2d> Blue_close4Waypoint2
    {
      frc::Pose2d(2.55_m, 6_m, -130_deg),
      frc::Pose2d(2.7_m, 7.05_m, -130_deg)
    };

    std::vector<frc::Pose2d> Blue_close4Waypoint3
    {
      frc::Pose2d(2.2_m, 4.85_m, 180_deg),
      frc::Pose2d(2.6_m, 4.15_m, 180_deg),
      frc::Pose2d(2.35_m, 4.15_m, 135_deg)
    };

    std::vector<units::meters_per_second_t> Blue_close4PointSpeed1
    {
      0.5_mps,
      2_mps
    };

    std::vector<units::meters_per_second_t> Blue_close4PointSpeed2
    {
      0_mps,
      1.25_mps,
      1.25_mps
    };

    std::vector<units::meters_per_second_t> Blue_close4PointSpeed3
    {
      0_mps,
      2.5_mps,
      1.5_mps,
      0.5_mps
    };

    std::vector<units::meters_per_second_t> Blue_close4CruiseSpeed1
    {
      2.5_mps,
      2.5_mps
    };

    std::vector<units::meters_per_second_t> Blue_close4CruiseSpeed2
    {
      1.5_mps,
      1.5_mps,
      1.5_mps
    };

    std::vector<units::meters_per_second_t> Blue_close4CruiseSpeed3
    {
      1.5_mps,
      3.25_mps,
      1_mps,
      0.5_mps
    };



    // RED FRONT 4 NOTE
    //the numbers are wrong according to our map but they work so dont touch, off by 0.55
    std::vector<frc::Pose2d> Red_Close4Waypoint1
    {
      frc::Pose2d(14.7_m, 5.55_m, 0_deg), //15.25
      frc::Pose2d(13.35_m, 5.55_m, 0_deg)
    };

    std::vector<frc::Pose2d> Red_Close4Waypoint2
    {
      frc::Pose2d(13.45_m, 6_m, -50_deg),
      frc::Pose2d(13.3_m, 7.05_m, -50_deg)
    };

    std::vector<frc::Pose2d> Red_Close4Waypoint3
    {
      frc::Pose2d(13.8_m, 4.85_m, 0_deg),
      frc::Pose2d(13.2_m, 4.1_m, 0_deg),
      frc::Pose2d(13.65_m, 4.1_m, 45_deg)
    };

    std::vector<units::meters_per_second_t> Red_Close4PointSpeed1
    {
      0.5_mps,
      2_mps
    };

    std::vector<units::meters_per_second_t> Red_Close4PointSpeed2
    {
      0_mps,
      1.25_mps,
      1.25_mps
    };

    std::vector<units::meters_per_second_t> Red_Close4PointSpeed3
    {
      0_mps,
      2.5_mps,
      1.5_mps,
      0.5_mps
    };

    std::vector<units::meters_per_second_t> Red_Close4CruiseSpeed1
    {
      2.5_mps,
      2.5_mps
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
      3.25_mps,
      1_mps,
      0.5_mps
    };


    //BLUE SOURCE 3 NOTE
    //Start to first shooting spot
    std::vector<frc::Pose2d> Blue_SourceSide1
    {
      frc::Pose2d(0.45_m, 2.5_m, 180_deg),
      frc::Pose2d(2.5_m, 2.5_m, 160_deg)
    };

    //Post first shoot to midline pickup
    std::vector<frc::Pose2d> Blue_SourceSide2
    {
      frc::Pose2d(5.5_m, 1.65_m, 160_deg)
    };

    //Midline pickup to 2nd shooting spot
    std::vector<frc::Pose2d> Blue_SourceSide3
    {
      //frc::Pose2d(8.05_m, 0.85_m, 160_deg),
      frc::Pose2d(5.5_m, 1.5_m, 150_deg),
      frc::Pose2d(4.0_m, 2.5_m, 150_deg)
    };
    
    //Post 2nd shot around pillar to 3rd note pickup
    std::vector<frc::Pose2d> Blue_SourceSide4
    {
      frc::Pose2d(5.5_m, 1.8_m, -160_deg),
      // frc::Pose2d(6.5_m, 2_m, -160_deg)
    };

    //3rd note pickup to 3rd not shooting spot
    std::vector<frc::Pose2d> Blue_SourceSide5
    {
      //frc::Pose2d(8.05_m, 2.45_m, 200_deg),
      frc::Pose2d(6.5_m, 1.5_m, 150_deg),
      frc::Pose2d(4.0_m, 2.5_m, 150_deg)

      //Potential cut through middle stage for 3rd note.
      //frc::Pose2d(5.6_m, 4.0_m, 160_deg),
      //frc::Pose2d(4.0_m, 5.55_m, 200_deg)
    };

    std::vector<units::velocity::meters_per_second_t> Blue_SourceSidePoint1
    {
      1_mps,
      0_mps
    };

    std::vector<units::velocity::meters_per_second_t> Blue_SourceSidePoint2
    {
      1_mps,
      3_mps
    };

    std::vector<units::velocity::meters_per_second_t> Blue_SourceSidePoint3
    {
      1_mps,
      3_mps,
      0_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Blue_SourceSidePoint4
    {
      1_mps,
      3_mps
      // 2_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Blue_SourceSidePoint5
    {
      1_mps,
      3_mps,
      0_mps
    };


    std::vector<units::velocity::meters_per_second_t> Blue_SourceSideCruise1
    {
      1_mps,
      3_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Blue_SourceSideCruise2
    {
      1_mps,
      3_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Blue_SourceSideCruise3
    {
      1_mps,
      4_mps,
      2_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Blue_SourceSideCruise4
    {
      1_mps,
      3_mps
      // 2_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Blue_SourceSideCruise5
    {
      1_mps,
      4_mps,
      2_mps
    };


    //Dead Reckoning 

        std::vector<frc::Pose2d> Blue_SourceSide1DR
    {
      frc::Pose2d(0.45_m, 2.5_m, 180_deg),
      frc::Pose2d(2.5_m, 2.5_m, 160_deg)
    };

    //Post first shoot to midline pickup
    std::vector<frc::Pose2d> Blue_SourceSide2DR
    {
      frc::Pose2d(8.05_m, 1.65_m, 160_deg)
    };

    //Midline pickup to 2nd shooting spot
    std::vector<frc::Pose2d> Blue_SourceSide3DR
    {
      //frc::Pose2d(8.05_m, 0.85_m, 160_deg),
      frc::Pose2d(5.5_m, 1.5_m, 150_deg),
      frc::Pose2d(4.0_m, 2.5_m, 150_deg)
    };
    
    //Post 2nd shot around pillar to 3rd note pickup
    std::vector<frc::Pose2d> Blue_SourceSide4DR
    {
      frc::Pose2d(7_m, 1.8_m, -160_deg),
      frc::Pose2d(8.05_m, 2.35_m, -160_deg),
      // frc::Pose2d(6.5_m, 2_m, -160_deg)
    };

    //3rd note pickup to 3rd not shooting spot
    std::vector<frc::Pose2d> Blue_SourceSide5DR
    {
      //frc::Pose2d(8.05_m, 2.45_m, 200_deg),
      frc::Pose2d(6.5_m, 1.5_m, 150_deg),
      frc::Pose2d(4.0_m, 2.5_m, 150_deg)

      //Potential cut through middle stage for 3rd note.
      //frc::Pose2d(5.6_m, 4.0_m, 160_deg),
      //frc::Pose2d(4.0_m, 5.55_m, 200_deg)
    };

    std::vector<units::velocity::meters_per_second_t> Blue_SourceSidePoint1DR
    {
      1_mps,
      0_mps
    };

    std::vector<units::velocity::meters_per_second_t> Blue_SourceSidePoint2DR
    {
      1_mps,
      0.5_mps
    };

    std::vector<units::velocity::meters_per_second_t> Blue_SourceSidePoint3DR
    {
      1_mps,
      3_mps,
      0_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Blue_SourceSidePoint4DR
    {
      1_mps,
      3_mps,
      0.5_mps
      // 2_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Blue_SourceSidePoint5DR
    {
      1_mps,
      3_mps,
      0_mps
    };


    std::vector<units::velocity::meters_per_second_t> Blue_SourceSideCruise1DR
    {
      1_mps,
      2.5_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Blue_SourceSideCruise2DR
    {
      1_mps,
      3_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Blue_SourceSideCruise3DR
    {
      1_mps,
      4_mps,
      2_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Blue_SourceSideCruise4DR
    {
      1_mps,
      3_mps,
      0.5_mps
      // 2_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Blue_SourceSideCruise5DR
    {
      1_mps,
      4_mps,
      2_mps
    };  

    //RED SOURCE 3 NOTE
    //Start to first shooting spot
    std::vector<frc::Pose2d> Red_SourceSide1
    {
      frc::Pose2d(16.1_m, 2.5_m, 0_deg),
      frc::Pose2d(14.05_m, 2.5_m, 20_deg)
    };

    //Post first shoot to midline pickup
    std::vector<frc::Pose2d> Red_SourceSide2
    {
      frc::Pose2d(11.05_m, 1.65_m, 20_deg),
    };

    //Midline pickup to 2nd shooting spot
    std::vector<frc::Pose2d> Red_SourceSide3
    {
      frc::Pose2d(11.05_m, 1.5_m, 30_deg),
      frc::Pose2d(12.55_m, 2.5_m, 30_deg)
    };
    
    //Post 2nd shot around pillar to 3rd note pickup
    std::vector<frc::Pose2d> Red_SourceSide4
    {
      frc::Pose2d(11.05_m, 1.8_m, -20_deg),
    };

    //3rd note pickup to 3rd not shooting spot
    std::vector<frc::Pose2d> Red_SourceSide5
    {
      frc::Pose2d(10.05_m, 1.5_m, 30_deg),
      frc::Pose2d(12.55_m, 2.5_m, 30_deg)
    };

    std::vector<units::velocity::meters_per_second_t> Red_SourceSidePoint1
    {
      1_mps,
      0_mps
    };

    std::vector<units::velocity::meters_per_second_t> Red_SourceSidePoint2
    {
      1_mps,
      3_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Red_SourceSidePoint3
    {
      1_mps,
      3_mps,
      0_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Red_SourceSidePoint4
    {
      1_mps,
      3_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Red_SourceSidePoint5
    {
      1_mps,
      3_mps,
      0_mps
    };


    std::vector<units::velocity::meters_per_second_t> Red_SourceSideCruise1
    {
      1_mps,
      3_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Red_SourceSideCruise2
    {
      1_mps,
      3_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Red_SourceSideCruise3
    {
      1_mps,
      4_mps,
      2_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Red_SourceSideCruise4
    {
      1_mps,
      3_mps
    };
    
    std::vector<units::velocity::meters_per_second_t> Red_SourceSideCruise5
    {
      1_mps,
      4_mps,
      2_mps
    };


    //BLUE AMP 3 NOTE
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
    //   frc::Pose2d(6.5_m, 6.6_m, 160_deg),
    //   frc::Pose2d(7.9_m, 5.95_m, 160_deg)
    // };

    // std::vector<frc::Pose2d> Blue_AmpSide6
    // {
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

    
    //RED AMP 3 NOTE

    std::vector<frc::Pose2d> Red_AmpSide1
    {
      frc::Pose2d(16.1_m, 7_m, 0_deg),
      frc::Pose2d(15.05_m, 7_m, 20_deg)
    };

    std::vector<frc::Pose2d> Red_AmpSide2
    {
      frc::Pose2d(13.65_m, 7_m, 0_deg)
    };

    std::vector<frc::Pose2d> Red_AmpSide3
    {
      frc::Pose2d(9.55_m, 7_m, 0_deg),
      frc::Pose2d(8.3_m, 7.35_m, 30_deg)
    };

    std::vector<frc::Pose2d> Red_AmpSide4
    {
      frc::Pose2d(11.25_m, 7_m, 0_deg)
    };

    //for a fourth note
    // std::vector<frc::Pose2d> Red_AmpSide5
    // {
    //   frc::Pose2d(10.05_m, 6.6_m, -20_deg),
    //   frc::Pose2d(8.65_m, 5.95_m, -20_deg)
    // };

    // std::vector<frc::Pose2d> Red_AmpSide6
    // {
    //   frc::Pose2d(10.05_m, 6.6_m, 0_deg),
    //   frc::Pose2d(11.25_m, 7_m, 0_deg)
    // };

    std::vector<units::velocity::meters_per_second_t> Red_AmpSidePoint1
    {
      1_mps,
      0_mps
    };

    std::vector<units::velocity::meters_per_second_t> Red_AmpSidePoint2
    {
      1_mps,
      2_mps
    };

    std::vector<units::velocity::meters_per_second_t> Red_AmpSidePoint3
    {
      1_mps,
      2_mps,
      1_mps
    };

    std::vector<units::velocity::meters_per_second_t> Red_AmpSidePoint4
    {
      1_mps,
      0_mps
    };

    std::vector<units::velocity::meters_per_second_t> Red_AmpSidePoint5
    {
      1_mps,
      3_mps,
      1_mps
    };

    std::vector<units::velocity::meters_per_second_t> Red_AmpSidePoint6
    {
      1_mps,
      3_mps,
      1_mps
    };


    std::vector<units::velocity::meters_per_second_t> Red_AmpSideCruise1
    {
      1_mps,
      2_mps
    };

    std::vector<units::velocity::meters_per_second_t> Red_AmpSideCruise2
    {
      1_mps,
      2_mps,
    };
    
    std::vector<units::velocity::meters_per_second_t> Red_AmpSideCruise3
    {
      1_mps,
      3_mps,
      2_mps
    };

    std::vector<units::velocity::meters_per_second_t> Red_AmpSideCruise4
    {
      1_mps,
      3_mps
    };

    std::vector<units::velocity::meters_per_second_t> Red_AmpSideCruise5
    {
      1_mps,
      3_mps,
      2_mps
    };

    std::vector<units::velocity::meters_per_second_t> Red_AmpSideCruise6
    {
      1_mps,
      3_mps,
      2_mps
    };


    //BLUE MIDLINE 4

    std::vector<frc::Pose2d> Blue_MidLine4Waypoint1
    {
      frc::Pose2d(1.3_m, 5.55_m, 180_deg),
      frc::Pose2d(2.65_m, 5.55_m, 180_deg)
    };

    std::vector<frc::Pose2d> Blue_MidLine4Waypoint2
    {
      frc::Pose2d(3.25_m, 5.55_m, 180_deg),
      frc::Pose2d(5.50_m, 4.05_m, 180_deg)
    };

    std::vector<frc::Pose2d> Blue_MidLine4Waypoint3
    {
      frc::Pose2d(5.00_m, 4.05_m, 180_deg),
      frc::Pose2d(4.50_m, 5.55_m, 180_deg),
      frc::Pose2d(2.25_m, 5.55_m, 180_deg)
    };

    std::vector<frc::Pose2d> Blue_MidLine4Waypoint4
    {
      frc::Pose2d(2.15_m, 5.55_m, 180_deg),
      frc::Pose2d(2.15_m, 4.25_m, 180_deg),
      frc::Pose2d(2.6_m, 4.25_m, 180_deg),
      frc::Pose2d(2.35_m, 4.25_m, 135_deg)
    };


    std::vector<units::meters_per_second_t> Blue_MidLine4PointSpeed1
    {
      0.5_mps,
      2_mps
    };

    std::vector<units::meters_per_second_t> Blue_MidLine4PointSpeed2
    {
      0_mps,
      1.25_mps,
      1.25_mps
    };

    std::vector<units::meters_per_second_t> Blue_MidLine4PointSpeed3
    {
      0_mps,
      2.5_mps,
      1.5_mps,
      0.5_mps
    };

    std::vector<units::meters_per_second_t> Blue_MidLine4PointSpeed4
    {
      0_mps,
      2.5_mps,
      1.5_mps,
      1.5_mps,
      0.5_mps
    };
    

    std::vector<units::meters_per_second_t> Blue_MidLine4CruiseSpeed1
    {
      2.5_mps,
      2.5_mps
    };

    std::vector<units::meters_per_second_t> Blue_MidLine4CruiseSpeed2
    {
      1.5_mps,
      1.5_mps,
      1.5_mps
    };

    std::vector<units::meters_per_second_t> Blue_MidLine4CruiseSpeed3
    {
      1.5_mps,
      3.0_mps,
      2.5_mps,
      0.5_mps
    };

    std::vector<units::meters_per_second_t> Blue_MidLine4CruiseSpeed4
    {
      1.5_mps,
      1.5_mps,
      1.5_mps,
      1.5_mps,
      0.5_mps
    };
};
