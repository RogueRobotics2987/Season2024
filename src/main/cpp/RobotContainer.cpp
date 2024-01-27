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
  m_drive.ResetOdometry(m_drive.GetPose()); 

  std::vector<frc::Pose2d> twoNotePoses{
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(0.5_m, 0.9_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(1.61_m, 0.9_m, frc::Rotation2d(235_deg))
  };

    std::vector<frc::Pose2d> squareDance{
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(1_m, 0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(1_m, 1_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(0_m, 1_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg))
  };

  std::vector<frc::Pose2d> theTwist{
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(1_m, 0_m, frc::Rotation2d(90_deg)),
    frc::Pose2d(2_m, 0_m, frc::Rotation2d(180_deg)),
  };

  return frc2::cmd::Sequence(
    // AutoAprilTag(m_limePose,m_drive).ToPtr(),
    // std::move(GetPath(twoNotePoses))
    followWaypoints(m_drive, theTwist , 0.25_mps).ToPtr()
    // std::move(pathplanner::AutoBuilder::followPath(path))
    // frc2::InstantCommand(
    // [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false); }, {}).ToPtr(), //always drives in the X-axis no matter if we put input in the X or Y
    // frc2::WaitCommand(1.0_s).ToPtr(),
    // std::move(pathplanner::AutoBuilder::followPath(path2))
  );  
}
