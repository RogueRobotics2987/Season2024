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
  m_chooser.AddOption("RedClose4", "RedClose4");
  m_chooser.AddOption("FarSideMid", "FarSideMid");

  //paths commented out for now since they are unused
  
  m_chooser.AddOption("B_1", "B_1");
  m_chooser.AddOption("B_3_6", "B_3_6");

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

      if (m_driverController.GetRawAxis(3)> 0.15) //if the right trigger is pulled
      {
        fieldOrientated = false; //robot orientated driving
      }

      if (m_driverController.GetRawAxis(3)< 0.15) //if the right trigger is not pulled
      {
        fieldOrientated = true; //field orientated driving
      }

      if ((safeX == 0) && (safeY == 0) && (safeRot == 0))
      {
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
      },
      {&m_shooter}
  ));

  m_climb.SetDefaultCommand(
  frc2::RunCommand(
    [this]
      {
        m_climb.StopClimber();
      },
      {&m_climb}
  ));

  m_intake.SetDefaultCommand(
  frc2::RunCommand(
    [this]
      {
        m_intake.StopIntake();
        m_intake.HoldMagazine(m_intake.GetCurrMagEncoderVal());
      },
      {&m_intake}
  ));

 m_arm.SetDefaultCommand(
  frc2::RunCommand(
    [this]
      {
        m_arm.SetLowerArmAngle(30);
        m_arm.StopArmWheels();
      },
      {&m_arm}
  ));
}

void RobotContainer::ConfigureButtonBindings()
{
  //Resets the heading of the gyro. In other words, it resets which way the robot thinks is the front
  frc2::JoystickButton(&m_driverController, 7).OnTrue(m_drive.ZeroHeadingCmd());

  frc2::JoystickButton(&m_driverController, 5).ToggleOnTrue(IntakeCmd(m_intake).ToPtr());

  frc2::JoystickButton(&m_driverController, 6).ToggleOnTrue(ShootCommand(m_shooterWheels, m_intake, m_driverController, m_auxController).ToPtr());

  frc2::JoystickButton(&m_driverController, 2).ToggleOnTrue(AprilTagAim(m_limelight, m_drive, m_driverController, m_shooter, m_auxController).ToPtr());

  frc2::JoystickButton(&m_driverController, 1).ToggleOnTrue(NoteFollower(m_limelight, m_drive, m_driverController, m_intake).ToPtr());

  frc2::JoystickButton(&m_driverController, 3).ToggleOnTrue(AmpCommand(m_arm).ToPtr());

  frc2::JoystickButton(&m_driverController, 4).ToggleOnTrue(AmpShooter(m_shooter, m_intake, m_driverController, m_arm, m_shooterWheels).ToPtr());

  frc2::JoystickButton(&m_driverController, 8).ToggleOnTrue(AmpLineup(m_drive, m_limelight).ToPtr());

  frc2::POVButton(&m_driverController, 90).WhileTrue(
    frc2::cmd::Run(
      [this]
        {
          m_climb.StartClimber(); 
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
}

float RobotContainer::DeadzoneCubed(float x)
{
  x = x * x * x;  // exponetial curve, slow acceleration at begining

  if ((x < 0.001) &&  (x > -0.001))
  {
    x = 0;
  }
  else if (x >= 0.001)
  {
    x = x ;
  }
  else if (x <= -0.001)
  {
    x = x ;
  }

  return(x);
}

float RobotContainer::Deadzone(float x)
{
  if ((x < 0.1) && (x > -0.1))
  {
    x = 0;
  }
  else if (x >= 0.1)
  {
    x = x - 0.1;
  }
  else if (x <= -0.1)
  {
    x = x + 0.1;
  }

  return(x);
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  m_shooter.zeroIntergralVal();

  chosenAuto = m_chooser.GetSelected();

  std::vector<frc::Pose2d> twoNotePoses
  {
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(0.75_m, 0.25_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(1.75_m, 0.25_m, frc::Rotation2d(235_deg))
  };
  
  std::vector<frc::Pose2d> thirdNotePoses
  {
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(0.75_m, 0.25_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(1.75_m, 0.25_m, frc::Rotation2d(235_deg)),
    frc::Pose2d(0.75_m, -0.42_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(0.75_m, -1.16_m, frc::Rotation2d(180_deg)),
    frc::Pose2d(1.75_m, -1.16_m, frc::Rotation2d(235_deg))
  };

  std::vector<frc::Pose2d> forthNotePoses
  {
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

  std::vector<frc::Pose2d> squareDance
  {
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

  std::vector<units::meters_per_second_t> squareDanceSpeeds
  {
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

  std::vector<units::meters_per_second_t> squareDanceCruiseSpeeds
  {
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

  std::vector<frc::Pose2d> theTwist
  {
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(1_m, 0_m, frc::Rotation2d(90_deg)),
    frc::Pose2d(2_m, 0_m, frc::Rotation2d(180_deg))
  };

  std::vector<frc::Pose2d> walkTheLine
  {
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(2_m, 0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(4_m, 0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(5.5_m, 0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(7_m, 0_m, frc::Rotation2d(0_deg))
  };

  std::vector<units::meters_per_second_t> walkTheLineSpeeds
  {
    0_mps,
    1_mps,
    0_mps,
    1_mps,
    0_mps
  };

  std::vector<units::meters_per_second_t> walkTheLineCruiseSpeeds
  {
    1_mps,
    1_mps,
    1_mps,
    1_mps,
    1_mps
  };
  
  if(chosenAuto == "B_2")
  {
    m_drive.ResetOdometry(B_2Waypoints[0]);
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      AutoShooterWarmupCmd(m_shooterWheels).ToPtr(),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(1.5_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooterWheels, m_intake).ToPtr(),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, B_2Waypoints, B_2PointSpeed, B_2CruiseSpeed, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      frc2::cmd::Race( //primes the shooter again as before
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(2_s).ToPtr()
      ),
      AutoShootCommand(m_shooterWheels, m_intake).ToPtr()
      // frc2::cmd::RunOnce(
      //   [this]
      //   {
      //     m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
      //   },
      //   {&m_drive}
      // )
    );
  }
  //JustShoot ------------------------------------------------------------------------------------------------------------------------------------------------
  else if(chosenAuto == "JustShootLeft")
  { 
    m_drive.ResetOdometry({0_m, 0_m, -120_deg});
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      AutoShooterWarmupCmd(m_shooterWheels).ToPtr(),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(1.5_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooterWheels, m_intake).ToPtr()
      // frc2::cmd::RunOnce(
      //   [this]
      //   {
      //     m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
      //   },
      //   {&m_drive}
      // )
    );
  }
  else if(chosenAuto == "JustShootCenter")
  { 
    m_drive.ResetOdometry({0_m, 0_m, 180_deg});
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      AutoShooterWarmupCmd(m_shooterWheels).ToPtr(),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(1.5_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooterWheels, m_intake).ToPtr()
      // frc2::cmd::RunOnce(
      //   [this]
      //   {
      //     m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
      //   },
      //   {&m_drive}
      // )
    );
  }
  else if(chosenAuto == "JustShootRight")
  { 
    m_drive.ResetOdometry({0_m, 0_m, 120_deg});
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      AutoShooterWarmupCmd(m_shooterWheels).ToPtr(),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(1.5_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooterWheels, m_intake).ToPtr()
      // frc2::cmd::RunOnce(
      //   [this]
      //   {
      //     m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
      //   },
      //   {&m_drive}
      // )
    );
  }
  //Shoot then backup -------------------------------------------------------------------------------------------------------------------------------
  else if(chosenAuto == "ShootTobackupLeft")
  {
    m_drive.ResetOdometry(sideBackupWaypointsLeft[0]);
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      AutoShooterWarmupCmd(m_shooterWheels).ToPtr(),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(1.5_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooterWheels, m_intake).ToPtr(),
      FollowWaypoints(m_drive, m_limelight, sideBackupWaypointsLeft, sideBackupPointSpeedLeft, sideBackupCruiseSpeedLeft, false).ToPtr()
      // frc2::cmd::RunOnce(
      //   [this]
      //   {
      //     m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
      //   },
      //   {&m_drive}
      // )
    );
  }
  else if(chosenAuto == "ShootTobackupRight")
  {
    // m_drive.ZeroHeading(B_1Waypoints[0].Rotation());
    m_drive.ResetOdometry(sideBackupWaypointsRight[0]);
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      AutoShooterWarmupCmd(m_shooterWheels).ToPtr(),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(1.5_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooterWheels, m_intake).ToPtr(),
      FollowWaypoints(m_drive, m_limelight, sideBackupWaypointsRight, sideBackupPointSpeedRight, sideBackupCruiseSpeedRight, false).ToPtr()
      // frc2::cmd::RunOnce(
      //   [this]
      //   {
      //     m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
      //   },
      //   {&m_drive}
      // )
    );
  }
  else if(chosenAuto == "B_1")
  {
    m_drive.ResetOdometry(B_1Waypoints[0]);
      return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time.
      AutoShooterWarmupCmd(m_shooterWheels).ToPtr(),
      frc2::cmd::Parallel( 
        frc2::cmd::Race( //aim shooter for 0.75s
          AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
          frc2::WaitCommand(1.5_s).ToPtr() //can change if need
        )
      ),
      AutoShootCommand(m_shooterWheels, m_intake).ToPtr(),
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
      AutoShootCommand(m_shooterWheels, m_intake).ToPtr()
      // frc2::cmd::RunOnce(
      //   [this]
      //   {
      //     m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
      //   },
      //   {&m_drive}
      // )
    );
  }
  else if(chosenAuto == "B_3_6")
  { 
    m_drive.ResetOdometry(B_3_6Waypoints1[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      AutoShooterWarmupCmd(m_shooterWheels).ToPtr(),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(0.5_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooterWheels, m_intake).ToPtr(),
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
      AutoShootCommand(m_shooterWheels, m_intake).ToPtr(),
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
      AutoShootCommand(m_shooterWheels, m_intake).ToPtr()
      // frc2::cmd::RunOnce(
      //   [this]
      //   {
      //     m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
      //   },
      //   {&m_drive}
      // )
    );  
  }
  else if(chosenAuto == "Close4")
  {
    m_drive.ResetOdometry(close4Waypoint1[0]);
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      AutoShooterWarmupCmd(m_shooterWheels).ToPtr(),
      frc2::cmd::Race( //aim shooter for 0.75s
        //AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        AutoSubAim(m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(0.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr()
        )
      ),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, close4Waypoint1, close4PointSpeed1, close4CruiseSpeed1, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(0.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr()
        )
      ),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, close4Waypoint2, close4PointSpeed2, close4CruiseSpeed2, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(0.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr()
        )
      ),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, close4Waypoint3, close4PointSpeed3, close4CruiseSpeed3, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(0.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr()
        )
      ),
      frc2::cmd::RunOnce(
        [this]
        {
          m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
        },
        {&m_drive}
      )
    );
  }
  else if(chosenAuto == "RedClose4")
  {
    m_drive.ResetOdometry(redClose4Waypoint1[0]);
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      AutoShooterWarmupCmd(m_shooterWheels).ToPtr(),
      frc2::cmd::Race( //aim shooter for 0.75s
        //AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        AutoSubAim(m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(0.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr()
        )
      ),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, redClose4Waypoint1, redClose4PointSpeed1, redClose4CruiseSpeed1, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(0.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr()
        )
      ),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, redClose4Waypoint2, redClose4PointSpeed2, redClose4CruiseSpeed2, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(0.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr()
        )
      ),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, redClose4Waypoint3, redClose4PointSpeed3, redClose4CruiseSpeed3, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(0.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr()
        )
      ),
      frc2::cmd::RunOnce(
        [this]
        {
          m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
        },
        {&m_drive}
      )
    );
  }
  else if(chosenAuto == "FarSideMid")
  {
    m_drive.ResetOdometry(FarSideMid1[0]);
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      AutoShooterWarmupCmd(m_shooterWheels).ToPtr(),
      FollowWaypoints(m_drive, m_limelight, FarSideMid1, FarSideMidPoint1, FarSideMidCruise1, false).ToPtr(),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(0.5_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooterWheels, m_intake).ToPtr(),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, FarSideMid2, FarSideMidPoint2, FarSideMidCruise2, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      FollowWaypoints(m_drive, m_limelight, FarSideMid3, FarSideMidPoint3, FarSideMidCruise3, false).ToPtr(),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(1.25_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooterWheels, m_intake).ToPtr(),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, FarSideMid4, FarSideMidPoint4, FarSideMidCruise4, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      FollowWaypoints(m_drive, m_limelight, FarSideMid5, FarSideMidPoint5, FarSideMidCruise5, false).ToPtr(),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::WaitCommand(1.25_s).ToPtr() //can change if need
      ),
      AutoShootCommand(m_shooterWheels, m_intake).ToPtr(),
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

void RobotContainer::ZeroHeading()
{
  m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
}

void RobotContainer::ShooterOff(){
  m_shooterWheels.StopShooter();
}