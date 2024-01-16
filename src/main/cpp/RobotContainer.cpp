#include "RobotContainer.h"


using namespace DriveConstants;


RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
      std::cout << "cout in robot container" << std::endl;


  // Configure the button bindings
  ConfigureButtonBindings();
  m_drive.ZeroHeading(); //resets the heading on the gyro


  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
      bool noJoystickInput = false; //checks if there is any joystick input (if true the wheels will go to the the 45 degree (X) position)
      double safeX = DeadzoneCubed(m_driverController.GetLeftX());
      double safeY =  DeadzoneCubed(m_driverController.GetLeftY());
      double safeRot = DeadzoneCubed(m_driverController.GetRightX());


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

  m_limePose.SetDefaultCommand(LimeLightCmd(m_limePose));

}



void RobotContainer::ConfigureButtonBindings() {
  //Resets the heading of the gyro. In other words, it resets which way the robot thinks is the front
  frc2::JoystickButton(&m_driverController, 5).OnTrue(m_drive.ZeroHeading());


  //Robot slides right (when front is away from the drivers)
  frc2::JoystickButton(&m_driverController, 1).WhileTrue(m_drive.Twitch(true));


  //Robot slides left (when front is away from the drivers)
  frc2::JoystickButton(&m_driverController, 2).WhileTrue(m_drive.Twitch(false));
}


float RobotContainer::DeadzoneCubed(float x){
  x = x*x*x;

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
  // frc::Pose2d waypointB = m_drive.GetPose().TransformBy(frc::Transform2d{m_drive.GetPose(), {1_m, 0_m, 0_deg}});

  // frc::Pose2d waypointC = {-0.5_m, 0_m, 0_deg};

  return frc2::cmd::Sequence(
      std::move(GoToAbsolutePoint({2_m, 0_m, 0_deg})),
      // frc2::InstantCommand(
      //     [this]() { m_drive.Drive(0.5_mps, 0_mps, 0_rad_per_s, false, false); }, {}).ToPtr(),
      //     frc2::WaitCommand(2.0_s).ToPtr(),
      // std::move(GoToAbsolutePoint({1_m, 0_m, 0_deg})),
      frc2::InstantCommand(
          [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false); }, {}).ToPtr()
    );
}

frc2::CommandPtr RobotContainer::GoToAbsolutePoint(frc::Pose2d waypoint){

  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);
  config.SetReversed(true);

  // // An example trajectory to follow.  All units in meters.
  // auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  //     // Start at the origin facing the +X direction
  //     frc::Pose2d{m_drive.GetPose()},
  //     {frc::Translation2d{2_m, 0_m}, frc::Translation2d{1_m, 0_m}},
  //     waypoint,
  //     // Pass the config
  //     config);


  // auto trajectoryOne = frc::TrajectoryGenerator::GenerateTrajectory(
  //  frc::Pose2d(0_m, 0_m, 0_deg),
  //  {frc::Translation2d(0.5_m, 1_m), frc::Translation2d(-0.5_m, 2_m)},
  //  frc::Pose2d(0_m, 3_m, 0_deg), config);

  // auto trajectoryTwo = frc::TrajectoryGenerator::GenerateTrajectory(
  //  frc::Pose2d(0_m, 3_m, 0_deg),
  //  {frc::Translation2d(0.5_m, 4_m), frc::Translation2d(-0.5_m, 5_m)},
  //  frc::Pose2d(0_m, 6_m, 0_deg), config);

  // auto trajectoryOne = frc::TrajectoryGenerator::GenerateTrajectory(
  //  frc::Pose2d(0_m, 0_m, 0_deg),
  //  {frc::Translation2d(0.5_m, 1_m), frc::Translation2d(-0.5_m, 2_m)},
  //  frc::Pose2d(0_m, 3_m, 0_deg), config);

  // auto trajectoryTwo = frc::TrajectoryGenerator::GenerateTrajectory(
  //  frc::Pose2d(0_m, 3_m, 0_deg),
  //  {frc::Translation2d(0.5_m, 4_m), frc::Translation2d(-0.5_m, 5_m)},
  //  frc::Pose2d(0_m, 6_m, 0_deg), config);

  // concatTraj = trajectoryOne + trajectoryTwo;

  auto Backwards = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d(0_m, 0_m, 0_deg),
    {frc::Translation2d(2_m, 0_m), frc::Translation2d(1_m, 0.01_m)},
    frc::Pose2d(0_m, 0_m, 0_deg), config
  );

  auto Forwards = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d(0_m, 0_m, 0_deg),
    {frc::Translation2d(-2_m, 0_m), frc::Translation2d(-1_m, 0.01_m)},
    frc::Pose2d(0_m, 0_m, 0_deg), config
  );

  auto concatTraj = Backwards + Forwards;


  // frc::SmartDashboard::PutNumber("WaypointX", (double)waypoint.X());
  // frc::SmartDashboard::PutNumber("WaypointY", (double)waypoint.Y());
  // frc::SmartDashboard::PutNumber("WaypointRot", (double)waypoint.Rotation().Degrees());
  // frc::SmartDashboard::PutNumber("RobotX", (double)m_drive.GetPose().X());
  // frc::SmartDashboard::PutNumber("RobotY", (double)m_drive.GetPose().Y());
  // frc::SmartDashboard::PutNumber("RobotRot", (double)m_drive.GetPose().Rotation().Degrees());

  frc::ProfiledPIDController<units::radians> thetaController {
    AutoConstants::kPThetaController, 0, 0, AutoConstants::kThetaControllerConstraints
  };

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::CommandPtr swerveControllerCommand =
  frc2::SwerveControllerCommand<4> (
      concatTraj, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive})
      .ToPtr();
      
    return swerveControllerCommand;
}

//m_drive.GetPose().TransformBy(frc::Transform2d{m_drive.GetPose(), translation})

