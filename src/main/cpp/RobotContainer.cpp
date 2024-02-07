#include "RobotContainer.h"

using namespace DriveConstants;

RobotContainer::RobotContainer()
{
  // Initialize all of your commands and subsystems here
  std::cout << "cout in robot container" << std::endl;
  // frc::SmartDashboard::PutData(&m_chooser);

  // Configure the button bindings
  ConfigureButtonBindings();
  m_drive.ZeroHeading(); //resets the heading on the gyro

m_drive.SetDefaultCommand(frc2::RunCommand(
    [this] {
      bool noJoystickInput = false; //checks if there is any joystick input (if true the wheels will go to the the 45 degree (X) position)
      double safeX = Deadzone(m_driverController.GetLeftX());
      double safeY =  Deadzone(m_driverController.GetLeftY());
      double safeRot = Deadzone(m_driverController.GetRightX());
      bool fieldOrientated;

      if (m_driverController.GetRawAxis(3)> 0.15){ //if the right trigger is pulled
        fieldOrientated = false; //robot orientated driving
      }

      if (m_driverController.GetRawAxis(3)< 0.15){ //if the right trigger is not pulled
        fieldOrientated = true; //field orientated driving
      }

      if ((safeX == 0) && (safeY == 0) && (safeRot == 0)) {
        noJoystickInput = true; //the wheels will move to the 45 degree (X) position
      }

      m_drive.Drive(
        units::meters_per_second_t(-safeY * AutoConstants::kMaxSpeed),
        units::meters_per_second_t(-safeX * AutoConstants::kMaxSpeed),
        units::radians_per_second_t(-safeRot * std::numbers::pi * 1.5),
        fieldOrientated,
        noJoystickInput
      );
    },
    {&m_drive}
  ));
}

void RobotContainer::ConfigureButtonBindings()
{
  //Resets the heading of the gyro. In other words, it resets which way the robot thinks is the front
  frc2::JoystickButton(&m_driverController, 5).OnTrue(m_drive.ZeroHeading());

  //Robot slides right (when front is away from the drivers)
  frc2::JoystickButton(&m_driverController, 1).WhileTrue(m_drive.Twitch(true));

  //Robot slides left (when front is away from the drivers)
  frc2::JoystickButton(&m_driverController, 2).WhileTrue(m_drive.Twitch(false));

  //Limelight Note Detection
  frc2::JoystickButton(&m_driverController, 3).WhileTrue(NoteFollower(m_limePose, m_drive, m_driverController).ToPtr());

  //Limelight April Tag Detection, y
  frc2::JoystickButton(&m_driverController, 4).WhileTrue(AprilTagFollower(m_limePose, m_drive, m_driverController).ToPtr());
}


float RobotContainer::Deadzone(float x)
{
  if ((x < 0.1) &&  (x > -0.1)){
    x=0;
  }
  else if (x >= 0.1){
    x = x - 0.1;
  }
  else if (x <= -0.1){
    x = x + 0.1;
  }
  return(x);
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  // m_drive.ZeroHeading();

  std::vector<frc::Pose2d> twoNotePoses{
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(0.75_m, 0.25_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(1.75_m, 0.25_m, frc::Rotation2d(235_deg))
  };
  
  std::vector<frc::Pose2d> thirdNotePoses{
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(0.75_m, 0.25_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(1.75_m, 0.25_m, frc::Rotation2d(235_deg)),
    frc::Pose2d(0.75_m, -0.42_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(0.75_m, -1.16_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(1.75_m, -1.16_m, frc::Rotation2d(235_deg))
  };

  std::vector<frc::Pose2d> forthNotePoses{
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(0.75_m, 0.25_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(1.75_m, 0.25_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(0.75_m, -0.42_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(0.75_m, -1.16_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(1.75_m, -1.16_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(0.75_m, -1.8_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(0.75_m, -2.66_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(1.75_m, -2.66_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(1.75_m, 0.25_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(7.1_m, 0.7_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(4.1_m, 0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(5.85_m, -0.95_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(7.1_m, -0.95_m, frc::Rotation2d(0_deg))
  };

  std::vector<frc::Pose2d> squareDance{
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(1.7_m, 0_m, frc::Rotation2d(90_deg)),
    frc::Pose2d(2_m, 0.3_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(2_m, 1.7_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(1.7_m, 2_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(0.3_m, 2_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(0_m, 1.7_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(0_m, 0.3_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg))
  };

  std::vector<units::meters_per_second_t> squareDanceSpeeds{
    0_mps,
    1_mps,
    1_mps,
    1_mps,
    1_mps,
    1_mps,
    1_mps,
    1_mps,
    0_mps
  };

  std::vector<units::meters_per_second_t> squareDanceCruiseSpeeds{
    0_mps,
    1_mps,
    1_mps,
    1_mps,
    1_mps,
    1_mps,
    1_mps,
    1_mps,
    1_mps
  };

  std::vector<frc::Pose2d> theTwist{
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(1_m, 0_m, frc::Rotation2d(90_deg)),
    frc::Pose2d(2_m, 0_m, frc::Rotation2d(180_deg))
  };

  std::vector<frc::Pose2d> walkTheLine{
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(1_m, 0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(2_m, 0_m, frc::Rotation2d(0_deg)),
    // frc::Pose2d(4_m, 0_m, frc::Rotation2d(0_deg)),
    // frc::Pose2d(5_m, 0_m, frc::Rotation2d(0_deg))
  };

  std::vector<units::meters_per_second_t> walkTheLineSpeeds{
    0_mps,
    1_mps,
    // 2_mps,
    // 1_mps,
    0_mps
  };

  std::vector<units::meters_per_second_t> walkTheLineCruiseSpeeds{
    1_mps,
    1_mps,
    // 2_mps,
    // 1_mps,
    1_mps
  };

  std::vector<frc::Pose2d>  BPose1Shoot2{
    frc::Pose2d(1.25_m, 7_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(2.90_m, 7_m, frc::Rotation2d(0_deg))
  };

    std::vector<frc::Pose2d>  BPose2Shoot2{
    frc::Pose2d(1.25_m, 5.55_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(2.90_m, 5.55_m, frc::Rotation2d(0_deg))
  };

    std::vector<frc::Pose2d>  BPose3Shoot2{
    frc::Pose2d(1.25_m, 4.1_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(2.90_m, 4.1_m, frc::Rotation2d(0_deg))
  };

    std::vector<frc::Pose2d>  RPose1Shoot2{
    frc::Pose2d(15.05_m, 7_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(13.65_m, 7_m, frc::Rotation2d(0_deg))
  };

    std::vector<frc::Pose2d>  RPose2Shoot2{
    frc::Pose2d(15.05_m, 5.55_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(13.65_m, 5.55_m, frc::Rotation2d(0_deg))
  };

    std::vector<frc::Pose2d>  RPose3Shoot2{
    frc::Pose2d(15.05_m, 4.1_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(13.65_m, 4.1_m, frc::Rotation2d(0_deg))
  };

  std::vector<frc::Pose2d>  BPose1Shoot3{
    frc::Pose2d(1.25_m, 7_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(8.3_m, 7.45_m, frc::Rotation2d(0_deg))
  };

  std::vector<frc::Pose2d>  RPose1Shoot3{
    frc::Pose2d(15.05_m, 7_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(8.3_m, 7.45_m, frc::Rotation2d(0_deg))
  };

  m_drive.ResetOdometry({0_m, 0_m, 0_deg});  //Counter clock wise is positive, Clockwise is positive.
  // m_drive.ResetOdometry(R_3_6Waypoints[0]);  

  return frc2::cmd::Sequence(
    // AutoAprilTag(m_limePose,m_drive).ToPtr(),
    frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
    FollowWaypoints(m_drive, m_limePose, squareDance, squareDanceSpeeds, squareDanceCruiseSpeeds, true).ToPtr()
  );  
}