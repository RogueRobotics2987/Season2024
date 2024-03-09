#include "RobotContainer.h"

using namespace DriveConstants;

RobotContainer::RobotContainer()
{
  // Initialize all of your commands and subsystems here
  std::cout << "cout in robot container" << std::endl;

  m_chooser.SetDefaultOption("B_2", "B_2");
  m_chooser.AddOption("JustShootLeft", "JustShootLeft");
  m_chooser.AddOption("JustShootRight", "JustShootRight");
  m_chooser.AddOption("JustShootCenter", "JustShootCenter");
  m_chooser.AddOption("ShootTobackupRight", "ShootTobackupRight");
  m_chooser.AddOption("ShootTobackupLeft", "ShootTobackupLeft");
  m_chooser.AddOption("Close4", "Close4");

  //paths commented out for now since they are unused
  
  m_chooser.AddOption("B_1", "B_1");
  // m_chooser.AddOption("B_2", "B_2");
  // m_chooser.AddOption("B_3", "B_3");
  // m_chooser.AddOption("B_1_2_3", "B_1_2_3");
  m_chooser.AddOption("B_3_6", "B_3_6");
  // m_chooser.AddOption("B_3_7", "B_3_7");
  // m_chooser.AddOption("B_2_6", "B_2_6");
  // m_chooser.AddOption("B_2_7", "B_2_7");
  // m_chooser.AddOption("B_1_4", "B_1_4");
  // m_chooser.AddOption("B_3_8", "B_3_8");
  // m_chooser.AddOption("B_2_5", "B_2_5");
  // m_chooser.AddOption("R_1", "R_1");
  // m_chooser.AddOption("R_2", "R_2");
  // m_chooser.AddOption("R_3", "R_3");
  // m_chooser.AddOption("R_3_6", "R_3_6");
  // m_chooser.AddOption("R_3_7", "R_3_7");
  // m_chooser.AddOption("B_3_2_1", "B_3_2_1");
  // m_chooser.AddOption("B_1_5", "B_1_5");
  // m_chooser.AddOption("B_3_7_8", "B_3_7_8");
  // m_chooser.AddOption("B_1_4_5", "B_1_4_5");
  // m_chooser.AddOption("B_2_6_7", "B_2_6_7");
  // m_chooser.AddOption("B_3_6_7", "B_3_6_7");
  // m_chooser.AddOption("B_2_6_5", "B_2_6_5");
  // m_chooser.AddOption("B_2_5_6", "B_2_5_6");
  // m_chooser.AddOption("B_3_2_1_4_5", "B_3_2_1_4_5");
  // m_chooser.AddOption("R_1_2_3", "R_1_2_3");
  // m_chooser.AddOption("R_2_6", "R_2_6");
  // m_chooser.AddOption("R_2_7", "R_2_7");
  // m_chooser.AddOption("R_1_4", "R_1_4");
  // m_chooser.AddOption("R_3_8", "R_3_8");
  // m_chooser.AddOption("R_2_5", "R_2_5");
  // m_chooser.AddOption("R_3_2_1", "R_3_2_1");
  // m_chooser.AddOption("R_1_5", "R_1_5");
  // m_chooser.AddOption("R_3_7_8", "R_3_7_8");
  // m_chooser.AddOption("R_1_4_5", "R_1_4_5");
  // m_chooser.AddOption("R_2_6_7", "R_2_6_7");
  // m_chooser.AddOption("R_3_6_7", "R_3_6_7");
  // m_chooser.AddOption("R_2_6_5", "R_2_6_5");
  // m_chooser.AddOption("R_2_5_6", "R_2_5_6");
  // m_chooser.AddOption("R_3_2_1_4_5", "R_3_2_1_4_5");
  frc::SmartDashboard::PutData(&m_chooser);

  // Configure the button bindings
  ConfigureButtonBindings();
  m_drive.ZeroHeading(0_rad); //resets the heading on the gyro

  //Idea for implementing drive into state machine is putting this function in the execute possibly?
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

  m_shooter.SetDefaultCommand(
  frc2::RunCommand(
    [this]
      {
        m_shooter.AngleTrimAdjust(m_auxController.GetRawButtonPressed(6), m_auxController.GetRawButtonPressed(5));
        m_shooter.setRestingActuatorPosition();
        m_shooter.StopShooter();
      },
      {&m_shooter}
  ));

  m_climb.SetDefaultCommand(
  frc2::RunCommand(
    [this]
      {
        m_climb.stopClimber();
      },
      {&m_climb}
  ));

  m_intake.SetDefaultCommand(
  frc2::RunCommand(
    [this]
      {
        m_intake.stopIntake();
        m_intake.holdMagazine(m_intake.GetCurrMagEncoderVal());
      },
      {&m_intake}
  ));

 m_arm.SetDefaultCommand(
  frc2::RunCommand(
    [this]
      {
        m_arm.setLowerArmAngle(30);
        m_arm.stopArmWheels();
      },
      {&m_arm}
  ));
   

}

void RobotContainer::ConfigureButtonBindings()
{
  //Resets the heading of the gyro. In other words, it resets which way the robot thinks is the front
  frc2::JoystickButton(&m_driverController, 7).OnTrue(m_drive.ZeroHeadingCmd());

  frc2::JoystickButton(&m_driverController, 5).ToggleOnTrue(IntakeCmd(m_intake).ToPtr());

  frc2::JoystickButton(&m_driverController, 6).ToggleOnTrue(ShootCommand(m_shooter, m_intake, m_driverController, m_auxController).ToPtr());

  frc2::JoystickButton(&m_driverController, 2).ToggleOnTrue(AprilTagAim(m_limelight, m_drive, m_driverController, m_shooter, m_auxController).ToPtr());

  frc2::JoystickButton(&m_driverController, 1).ToggleOnTrue(NoteFollower(m_limelight, m_drive, m_driverController).ToPtr());

  frc2::JoystickButton(&m_driverController, 3).ToggleOnTrue(AmpCommand(m_arm).ToPtr());

  frc2::JoystickButton(&m_driverController, 4).ToggleOnTrue(AmpShooter(m_shooter, m_intake, m_driverController, m_arm).ToPtr());

  frc2::JoystickButton(&m_driverController, 8).ToggleOnTrue(AmpLineup(m_drive, m_limelight).ToPtr());


  frc2::POVButton(&m_driverController, 90).WhileTrue(
    frc2::cmd::Run(
      [this]
        {
          m_climb.startClimber(); 
        },
      {&m_climb}
    )
  );

  frc2::POVButton(&m_auxController, 0).WhileTrue(SpitOutCmd(m_intake).ToPtr());

  frc2::POVButton(&m_auxController, 90).WhileTrue(
    frc2::cmd::Run(
      [this]
        {
          m_shooter.AngleTrimAdjust(m_auxController.GetRawButtonPressed(6), m_auxController.GetRawButtonPressed(5));
          m_shooter.SetActuator(ShooterConstants::StageAngle); 
        },
      {&m_shooter}
    )
  );

  frc2::POVButton(&m_auxController, 180).WhileTrue(
    frc2::cmd::Run(
      [this]
        {
          m_shooter.AngleTrimAdjust(m_auxController.GetRawButtonPressed(6), m_auxController.GetRawButtonPressed(5));
          m_shooter.SetActuator(ShooterConstants::SubwooferAngle); 
        },
      {&m_shooter}
    )
  );

  frc2::POVButton(&m_auxController, 270).WhileTrue(ManualAim(m_shooter, m_auxController).ToPtr()); //could change to a toggle depending on what drivers like

  // // Robot slides right (when front is away from the drivers)
  // frc2::JoystickButton(&m_driverController, 1).WhileTrue(m_drive.Twitch(true));
  // // Robot slides left (when front is away from the drivers)
  // frc2::JoystickButton(&m_driverController, 2).WhileTrue(m_drive.Twitch(false));

  // //Limelight Note Detection
  //frc2::JoystickButton(&m_driverController, 1).ToggleOnTrue(NoteFollower(m_limelight, m_drive, m_driverController, m_intake, m_shooter, m_arm).ToPtr());

  // //Limelight April Tag Detection, y
  // frc2::JoystickButton(&m_driverController, 4).ToggleOnTrue(AprilTagAim(m_limelight, m_drive, m_driverController, m_shooter).ToPtr());

  //TODO adjust deadzone so the robot will not be at an angle when aiming, also implement this fucntion into the state machine
  // frc2::JoystickButton(&m_driverController, 6).ToggleOnTrue(AutoAprilTag(m_limelight, m_drive).ToPtr());

  // //start PICKUP state
  // //frc2::JoystickButton(&m_driverController, 5).ToggleOnTrue(m_intakeShoot.Pickup());
  // //frc2::JoystickButton(&m_driverController, 5).ToggleOnFalse(m_intakeShoot.PickupStop());

}

float RobotContainer::DeadzoneCubed(float x){
  x = x * x * x;  // exponetial curve, slow acceleration at begining

  if ((x < 0.001) &&  (x > -0.001)){
    x=0;
  }
  else if (x >= 0.001){
    x = x ;//- 0.1;
  }
  else if (x <= -0.001){
    x = x ;//+ 0.1;
  }

  return(x);
 
}

float RobotContainer::Deadzone(float x){
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

// frc2::CommandPtr RobotContainer::GetAuxilaryStateMachine()
// {
//   return StateMachine(
//       m_drive,
//       m_limelight,
//       m_arm, 
//       m_climb,
//       m_color, //can remove?
//       m_intake,
//       m_shooter,
//       m_driverController,
//       m_auxController
//     ).ToPtr();
// }

/*
frc2::CommandPtr RobotContainer::GetDriveStateMachine(){
  return DriveStateMachine(
    m_drive,
    m_limelight, 
    m_driverController, 
    m_auxController, 
    driveShooterMessager
  ).ToPtr();
}
*/

// frc2::CommandPtr RobotContainer::GetAutoAuxilaryStateMachine(){
//   return AutoAuxilaryStateMachine(
//       m_arm, 
//       m_climb, //can remove?
//       m_color, //can remove.
//       m_intake,
//       m_shooter, 
//       m_driverController,
//       m_auxController,
//       driveShooterMessager
//     ).ToPtr();
// }

// frc2::CommandPtr RobotContainer::GetAutoDriveStateMachine(){

//   // m_drive.ResetOdometry(B_1Waypoints[0]);

//   return AutoDriveStateMachine(
//     m_drive,
//     m_limelight,
//     m_driverController,
//     m_auxController,
//     driveShooterMessager,
//     path
//   ).ToPtr();
// }

// void RobotContainer::SetRanAuto(bool ranAuto){
//   m_drive.SetRanAuto(ranAuto);
// }

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  m_shooter.zeroIntergralVal();

  chosenAuto = m_chooser.GetSelected();


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
    frc::Pose2d(2_m, 0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(4_m, 0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(5.5_m, 0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(7_m, 0_m, frc::Rotation2d(0_deg))
  };

  std::vector<units::meters_per_second_t> walkTheLineSpeeds{
    0_mps,
    1_mps,
    0_mps,
    1_mps,
    0_mps
  };

  std::vector<units::meters_per_second_t> walkTheLineCruiseSpeeds{
    1_mps,
    1_mps,
    1_mps,
    1_mps,
    1_mps
  };

  // std::vector<frc::Pose2d>  BPose1Shoot3{
  //   frc::Pose2d(1.25_m, 7_m, frc::Rotation2d(0_deg)),
  //   frc::Pose2d(8.3_m, 7.45_m, frc::Rotation2d(0_deg))
  // };

  // std::vector<frc::Pose2d>  RPose1Shoot3{
  //   frc::Pose2d(15.05_m, 7_m, frc::Rotation2d(0_deg)),
  //   frc::Pose2d(8.3_m, 7.45_m, frc::Rotation2d(0_deg))
  // };
  
  if(chosenAuto == "B_2")
  {
    m_drive.ResetOdometry(B_2Waypoints[0]);
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(1.5_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooter, m_intake).ToPtr(),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, B_2Waypoints, B_2PointSpeed, B_2CruiseSpeed, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      frc2::cmd::Race( //primes the shooter again as before
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(2_s).ToPtr()
      ),
      AutoShootCommand(m_shooter, m_intake).ToPtr(),
      frc2::cmd::RunOnce(
        [this]
        {
          m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
        },
        {&m_drive}
      )
    );
  }
  //JustShoot ------------------------------------------------------------------------------------------------------------------------------------------------
  else if(chosenAuto == "JustShootLeft")
  { 
    m_drive.ResetOdometry({0_m, 0_m, -120_deg});
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(1.5_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooter, m_intake).ToPtr(),
      frc2::cmd::RunOnce(
        [this]
        {
          m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
        },
        {&m_drive}
      )
    );
  }
  else if(chosenAuto == "JustShootCenter")
  { 
    m_drive.ResetOdometry({0_m, 0_m, 180_deg});
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(1.5_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooter, m_intake).ToPtr(),
      frc2::cmd::RunOnce(
        [this]
        {
          m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
        },
        {&m_drive}
      )
    );
  }
  else if(chosenAuto == "JustShootRight")
  { 
    m_drive.ResetOdometry({0_m, 0_m, 120_deg});
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(1.5_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooter, m_intake).ToPtr(),
      frc2::cmd::RunOnce(
        [this]
        {
          m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
        },
        {&m_drive}
      )
    );
  }
  //Shoot then backup -------------------------------------------------------------------------------------------------------------------------------
  else if(chosenAuto == "ShootTobackupLeft")
  {
    m_drive.ResetOdometry(sideBackupWaypointsLeft[0]);
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      frc2::cmd::RunOnce(
        [this]
          {
            m_shooter.zeroIntergralVal();
          }
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(1.5_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooter, m_intake).ToPtr(),
      FollowWaypoints(m_drive, m_limelight, sideBackupWaypointsLeft, sideBackupPointSpeedLeft, sideBackupCruiseSpeedLeft, false).ToPtr(),
      frc2::cmd::RunOnce(
        [this]
        {
          m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
        },
        {&m_drive}
      )
    );
  }
  else if(chosenAuto == "ShootTobackupRight")
  {
    // m_drive.ZeroHeading(B_1Waypoints[0].Rotation());
    m_drive.ResetOdometry(sideBackupWaypointsRight[0]);
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      frc2::cmd::RunOnce(
        [this]
          {
            m_shooter.zeroIntergralVal();
          }
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(1.5_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooter, m_intake).ToPtr(),
      FollowWaypoints(m_drive, m_limelight, sideBackupWaypointsRight, sideBackupPointSpeedRight, sideBackupCruiseSpeedRight, false).ToPtr(),
      frc2::cmd::RunOnce(
        [this]
        {
          m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
        },
        {&m_drive}
      )
    );
  }
  else if(chosenAuto == "B_1")
  {
    m_drive.ResetOdometry(B_1Waypoints[0]);
      return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time.
      frc2::cmd::Parallel( 
        frc2::cmd::Race( //aim shooter for 0.75s
          AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
          frc2::WaitCommand(1.5_s).ToPtr() //can change if need
        )
      ),
      AutoShootCommand(m_shooter, m_intake).ToPtr(),
      frc2::cmd::Parallel( // follow path and turn on intake until a note has been seen by the mag
        FollowWaypoints(m_drive, m_limelight, B_1Waypoints, B_1PointSpeed, B_1CruiseSpeed, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      frc2::cmd::Parallel( 
        frc2::cmd::Race( //aim shooter for 0.75s
          AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
          frc2::WaitCommand(1.5_s).ToPtr() //can change if need
        )
      ),
      AutoShootCommand(m_shooter, m_intake).ToPtr(),
      frc2::cmd::RunOnce(
        [this]
        {
          m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
        },
        {&m_drive}
      )
    );
  }
  else if(chosenAuto == "B_3_6")
  { 
    m_drive.ResetOdometry(B_3_6Waypoints1[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(0.5_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooter, m_intake).ToPtr(),
      frc2::cmd::Parallel( // follow path and turn on intake until a note has been seen by the mag
        FollowWaypoints(m_drive, m_limelight, B_3_6Waypoints1, B_3_6PointSpeed1, B_3_6CruiseSpeed1, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      frc2::cmd::Parallel( 
        frc2::cmd::Race( //aim shooter for 0.75s
          AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
          frc2::WaitCommand(1.5_s).ToPtr() //can change if need
        )
      ),
      AutoShootCommand(m_shooter, m_intake).ToPtr(),
      frc2::cmd::Parallel( // follow path and turn on intake until a note has been seen by the mag
        FollowWaypoints(m_drive, m_limelight, B_3_6Waypoints2, B_3_6PointSpeed2, B_3_6CruiseSpeed2, false).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(1_s).ToPtr(), //possibly remove the sequential statement with a wait in it to delay when the intake turns on 
          IntakeCmd(m_intake).ToPtr() 
        )
      ),
      FollowWaypoints(m_drive, m_limelight, B_3_6Waypoints3, B_3_6PointSpeed3, B_3_6CruiseSpeed3, false).ToPtr(),
      frc2::cmd::Parallel( 
        frc2::cmd::Race( //aim shooter for 0.75s
          AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
          frc2::WaitCommand(0.5_s).ToPtr() //can change if need
        )
      ),
      AutoShootCommand(m_shooter, m_intake).ToPtr(),
      frc2::cmd::RunOnce(
        [this]
        {
          m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
        },
        {&m_drive}
      )
    );  
  }
  else if(chosenAuto == "Close4")
  {
    m_drive.ResetOdometry(close4Waypoint1[0]);
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      frc2::cmd::RunOnce(
        [this]
          {
            m_shooter.zeroIntergralVal();
          }
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(0.5_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooter, m_intake).ToPtr(),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, close4Waypoint1, close4PointSpeed1, close4CruiseSpeed1, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(1.25_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooter, m_intake).ToPtr(),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, close4Waypoint2, close4PointSpeed2, close4CruiseSpeed2, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(1.25_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooter, m_intake).ToPtr(),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, close4Waypoint3, close4PointSpeed3, close4CruiseSpeed3, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(1.25_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooter, m_intake).ToPtr(),
      frc2::cmd::RunOnce(
        [this]
        {
          m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
        },
        {&m_drive}
      )
    );
  }
  else
  {
    //Should never get to this case
    std::cout << "You should never see this error.... bottom of robot container" << std::endl;

    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr()
    );
  }
}