#include "RobotContainer.h"


using namespace DriveConstants;


RobotContainer::RobotContainer() {
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


      m_drive.Drive(units::meters_per_second_t(
                    -safeY * AutoConstants::kMaxSpeed),
                    units::meters_per_second_t(
                    -safeX * AutoConstants::kMaxSpeed),
                    units::radians_per_second_t(
                    -safeRot * std::numbers::pi * 1.5),
                    fieldOrientated,
                    noJoystickInput);
      },{&m_drive}));

}



void RobotContainer::ConfigureButtonBindings() {
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


float RobotContainer::Deadzone(float x){
  if ((x < 0.1) &&  (x > -0.1)){
    x=0;
  } else if (x >= 0.1){
    x = x - 0.1;
  } else if (x <= -0.1){
    x = x + 0.1;
  }
  return(x);
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); 

  std::vector<frc::Pose2d> twoNotePoses{
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(0.75_m, 0.25_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(1.75_m, 0.25_m, frc::Rotation2d(235_deg))
  };
  std::vector<frc::Pose2d> thirdNotePoses{
    frc::Pose2d(0.75_m, -0.42_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(0.75_m, -1.16_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(1.75_m, -1.16_m, frc::Rotation2d(235_deg))
  };
  std::vector<frc::Pose2d> forthNotePoses{
    frc::Pose2d(0.75_m, -1.8_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(0.75_m, -2.66_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(1.75_m, -2.66_m, frc::Rotation2d(235_deg))
  };

  return frc2::cmd::Sequence(
      AutoAprilTag(m_limePose,m_drive).ToPtr(),
      std::move(GetPath(twoNotePoses)),
      AutoAprilTag(m_limePose,m_drive).ToPtr(),
      std::move(GetPath(thirdNotePoses)),
      AutoAprilTag(m_limePose,m_drive).ToPtr(),
      std::move(GetPath(forthNotePoses))

      // frc2::InstantCommand(
      // [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false); }, {}).ToPtr(), //always drives in the X-axis no matter if we put input in the X or Y
      // frc2::WaitCommand(1.0_s).ToPtr(),
      // std::move(pathplanner::AutoBuilder::followPath(path2))
    );  
}
frc2::CommandPtr RobotContainer::GetPath(std::vector<frc::Pose2d> waypoints) {
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
    GoalEndState(0.0_mps, frc::Rotation2d(-90_deg), true) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
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


  // return frc2::cmd::Sequence(
  //     std::move(m_drive.followPathCommand(path))
  //     // frc2::InstantCommand(
  //     // [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false); }, {}).ToPtr(), //always drives in the X-axis no matter if we put input in the X or Y
  //     // frc2::WaitCommand(1.0_s).ToPtr(),
  //     // std::move(pathplanner::AutoBuilder::followPath(path2))
  //   );  
}

// frc2::CommandPtr RobotContainer::GoToAbsolutePoint(frc::Pose2d waypoint, bool reversed){

//   // Set up config for trajectory
//   frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
//                                AutoConstants::kMaxAcceleration);
//   // Add kinematics to ensure max speed is actually obeyed
//   config.SetKinematics(m_drive.kDriveKinematics);
//   config.SetReversed(reversed);

//   // An example trajectory to follow.  All units in meters.

//   // auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
//   //     // Start at the origin facing the +X direction
//   //     {frc::Pose2d{m_drive.GetDrivePosePtr()->X(), 0_m, waypoint.Rotation()},
//   //     waypoint},
//   //     // Pass the config
//   //     config);

//   std::vector<frc::Pose2d> poses{
//     frc::Pose2d(1.0_m, 1.0_m, frc::Rotation2d(0_deg)),
//     frc::Pose2d(2.0_m, 1.0_m, frc::Rotation2d(0_deg)),
//     frc::Pose2d(3.0_m, 2.0_m, frc::Rotation2d(90_deg))
// };

//   // auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
//   //     // Start at the origin facing the +X direction
//   //     {poses},
//   //     // Pass the config
//   //     config);


// std::vector<frc::Translation2d> bezierPoints = PathPlannerPath::bezierFromPoses(poses);

// // Create the path using the bezier points created above
// // We make a shared pointer here since the path following commands require a shared pointer
// auto path = std::make_shared<PathPlannerPath>(
//     bezierPoints,
//     PathConstraints(3.0_mps, 3.0_mps_sq, 360_deg_per_s, 720_deg_per_s_sq), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
//     GoalEndState(0.0_mps, frc::Rotation2d(-90_deg)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
// );

// // Prevent the path from being flipped if the coordinates are already correct
// path->preventFlipping = true;


// // if(reversed == true){
// //       for(int i = 0; i<static_cast<int>(exampleTrajectory.States().size()); i++){
// //         std::cout << (double)exampleTrajectory.States()[i].pose.X() << std::endl;
// //       }
// // }
//   // auto FrontBack = frc::TrajectoryGenerator::GenerateTrajectory(
//   //   frc::Pose2d(0_m, 0_m, 0_deg),
//   //   {frc::Translation2d(2_m, 0_m), frc::Translation2d(1_m, 0.01_m), frc::Translation2d(0_m, 0_m), frc::Translation2d(-2_m, 0_m), frc::Translation2d(-1_m, 0.01_m)},
//   //   frc::Pose2d(0_m, 0_m, 0_deg), config
//   // );

//   // auto Forwards = frc::TrajectoryGenerator::GenerateTrajectory(
//   //   frc::Pose2d(0_m, 0_m, 0_deg),
//   //   {frc::Translation2d(-2_m, 0_m), frc::Translation2d(-1_m, 0.01_m)},
//   //   frc::Pose2d(0_m, 0_m, 0_deg), config
//   // );

//   // auto GoToPoint = frc::TrajectoryGenerator::GenerateTrajectory(
//   //     frc::Pose2d{m_drive.GetPose()},
//   //     {frc::Translation2d{2_m, 0.01_m}, frc::Translation2d{1_m, 0_m}},
//   //     frc::Pose2d{0_m, 0_m, 180_deg},
//   //     // Pass the config
//   //     config);
//   //     //180 is forward in degrees

//   // auto concatTraj = Backwards + Forwards;

//   //   frc::SmartDashboard::PutNumber("AutoMaxSpeed", (double)AutoConstants::kMaxSpeed);
//   //   frc::SmartDashboard::PutNumber("AutoMaxaAccel", (double)AutoConstants::kMaxAcceleration);


//   // // frc::SmartDashboard::PutNumber("WaypointX", (double)waypoint.X());
//   // // frc::SmartDashboard::PutNumber("WaypointY", (double)waypoint.Y());
//   // // frc::SmartDashboard::PutNumber("WaypointRot", (double)waypoint.Rotation().Degrees());
//   // // frc::SmartDashboard::PutNumber("RobotX", (double)m_drive.GetPose().X());
//   // // frc::SmartDashboard::PutNumber("RobotY", (double)m_drive.GetPose().Y());
//   // // frc::SmartDashboard::PutNumber("RobotRot", (double)m_drive.GetPose().Rotation().Degrees());

//   // frc::ProfiledPIDController<units::radians> thetaController {
//   //   AutoConstants::kPThetaController, 0, 0, AutoConstants::kThetaControllerConstraints
//   // };

//   // thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
//   //                                       units::radian_t{std::numbers::pi});

//   // frc::SmartDashboard::PutNumber("AutoX", (double)m_drive.GetPose().X());
//   // frc::SmartDashboard::PutNumber("AutoY", (double)m_drive.GetPose().Y());

//   // frc2::CommandPtr swerveControllerCommand =
//   // frc2::SwerveControllerCommand<4> (
//   //     exampleTrajectory, [this]() { return m_drive.GetPose(); },

//   //     m_drive.kDriveKinematics,

//   //     frc::PIDController{AutoConstants::kPXController, 0, 0},
//   //     frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

//   //     [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

//   //     {&m_drive})
//   //     .ToPtr();
      
//   //   return swerveControllerCommand;
// }


