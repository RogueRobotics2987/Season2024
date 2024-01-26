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

  //runs a basic autonomous
  frc2::JoystickButton(&m_driverController, 6).OnTrue(onFlyGeneration());

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
  m_drive.ZeroHeading();
  m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); 

  std::vector<frc::Pose2d> twoNotePoses{
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(0.5_m, 0.9_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(1.61_m, 0.9_m, frc::Rotation2d(235_deg))
  };

    std::vector<frc::Pose2d> snakePath{
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(1_m, 0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(2_m, 0_m, frc::Rotation2d(0_deg))
  };

  auto path = PathPlannerPath::fromPathFile("Rotate");

  return frc2::cmd::Sequence(
    // AutoAprilTag(m_limePose,m_drive).ToPtr(),
    // std::move(GetPath(twoNotePoses))
    followWaypoints(m_drive, snakePath, 0.25_mps).ToPtr()
    // std::move(pathplanner::AutoBuilder::followPath(path))
    // frc2::InstantCommand(
    // [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false); }, {}).ToPtr(), //always drives in the X-axis no matter if we put input in the X or Y
    // frc2::WaitCommand(1.0_s).ToPtr(),
    // std::move(pathplanner::AutoBuilder::followPath(path2))
  );  
}

frc2::CommandPtr RobotContainer::GetPath(std::vector<frc::Pose2d> waypoints)
{
  // m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, -180_deg}); 

  //   std::vector<frc::Pose2d> poses{
  //     frc::Pose2d(1.0_m, 0_m, frc::Rotation2d(0_deg)),
  //     frc::Pose2d(2.0_m, 1.0_m, frc::Rotation2d(0_deg)),
  //     frc::Pose2d(3.0_m, 2.0_m, frc::Rotation2d(0_deg)),
  //     frc::Pose2d(2.0_m, 1.0_m, frc::Rotation2d(0_deg)),
  //     frc::Pose2d(1.0_m, 2_m, frc::Rotation2d(0_deg)),
  //     frc::Pose2d(0.01_m, 0_m, frc::Rotation2d(0_deg))
  // };

  //   std::vector<frc::Pose2d> twoNotePoses{
  //     // frc::Pose2d(0.5_m, 0_m, frc::Rotation2d(-180_deg)),
  //     // frc::Pose2d(3_m, 0_m, frc::Rotation2d(-90_deg))
  //     frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
  //     frc::Pose2d(0.5_m, 0.9_m, frc::Rotation2d(0_deg)),
  //     frc::Pose2d(1.61_m, 0.9_m, frc::Rotation2d(45_deg))
  // };

  //   std::vector<frc::Pose2d> poses2{
  //     frc::Pose2d(1_m, 0_m, frc::Rotation2d(90_deg)),
  //     frc::Pose2d(0_m, 0_m, frc::Rotation2d(90_deg))
  // };

  std::vector<frc::Translation2d> bezierPoints = PathPlannerPath::bezierFromPoses(waypoints);

  // Create the path using the bezier points created above
  // We make a shared pointer here since the path following commands require a shared pointer
  auto path = std::make_shared<PathPlannerPath>(
    bezierPoints,
    PathConstraints(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration, AutoConstants::kMaxAngularSpeed, AutoConstants::kMaxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
    GoalEndState(0.0_mps, frc::Rotation2d(-235_deg), true) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
  );
  path->preventFlipping = true;

  // std::vector<frc::Translation2d> bezierPoints2 = PathPlannerPath::bezierFromPoses(poses2);

  // // Create the path using the bezier points created above
  // // We make a shared pointer here since the path following commands require a shared pointer
  // auto path2 = std::make_shared<PathPlannerPath>(
  //     bezierPoints2,
  //     PathConstraints(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration, AutoConstants::kMaxAngularSpeed, AutoConstants::kMaxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
  //     GoalEndState(0.0_mps, frc::Rotation2d(-90_deg), true) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
  // );
  // path2->preventFlipping = true;

  return m_drive.FollowPathCommand(path);
}

frc2::CommandPtr RobotContainer::onFlyGeneration(){
  return frc2::cmd::RunOnce([this]() {
  frc::Pose2d *currentPose = m_drive.GetDrivePosePtr();

  // The rotation component in these poses represents the direction of travel
  frc::Pose2d startPos = frc::Pose2d(currentPose->Translation(), frc::Rotation2d(currentPose->Rotation()));
  frc::Pose2d endPos = frc::Pose2d(currentPose->Translation() + frc::Translation2d(2.0_m, 0_m), frc::Rotation2d(currentPose->RotateBy(-180_deg).Rotation()));

  std::vector<frc::Translation2d> bezierPoints = PathPlannerPath::bezierFromPoses({startPos, endPos});
  // Paths must be used as shared pointers

  auto path = std::make_shared<PathPlannerPath>(
    bezierPoints, 
    PathConstraints(1.0_mps, 2.0_mps_sq, 180_deg_per_s, 180_deg_per_s_sq),
    GoalEndState(0_mps, frc::Rotation2d(currentPose->RotateBy(-180_deg).Rotation()))
  );

  // Prevent this path from being flipped on the red alliance, since the given positions are already correct
  path->preventFlipping = true;

  this->followOnTheFly = AutoBuilder::followPath(path).Unwrap();
  this->followOnTheFly->Schedule();
  },
  {&m_drive}
  );
}