#include "RobotContainer.h"

using namespace DriveConstants;

RobotContainer::RobotContainer()
{
  // Initialize all of your commands and subsystems here
  std::cout << "cout in robot container" << std::endl;

  m_chooser.SetDefaultOption("Blue_Close4", "Blue_Close4");
  m_chooser.AddOption("Red_Close4", "Red_Close4");
  m_chooser.AddOption("Blue_FarSideMid", "Blue_FarSideMid");
  m_chooser.AddOption("Red_FarSideMid", "Red_FarSideMid");
  //m_chooser.AddOption("Blue_AmpSide", "Blue_AmpSide");

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

      if (m_driverController.GetRawAxis(3) > 0.15) //if the right trigger is pulled
      {
        fieldOrientated = false; //robot orientated driving
      }

      if (m_driverController.GetRawAxis(3) < 0.15) //if the right trigger is not pulled
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

  frc2::JoystickButton(&m_auxController, 1).ToggleOnTrue(AmpCommand(m_arm).ToPtr());

  frc2::JoystickButton(&m_auxController, 2).ToggleOnTrue(AmpShooter(m_shooter, m_intake, m_driverController, m_arm, m_shooterWheels).ToPtr()); 

  // frc2::JoystickButton(&m_driverController, 8).ToggleOnTrue(AmpLineup(m_drive, m_limelight).ToPtr());

  frc2::JoystickButton(&m_auxController, 4).WhileTrue(
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

  if(chosenAuto == "Blue_Close4")
  {
    m_drive.ResetOdometry(Blue_close4Waypoint1[0]);
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      frc2::cmd::RunOnce(
        [this]
        {
          m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
        },
        {&m_drive}),
      AutoShooterWarmupCmd(m_shooterWheels).ToPtr(),
      frc2::cmd::Race(
        AutoSubAim(m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(0.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr())
      ),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, Blue_close4Waypoint1, Blue_close4PointSpeed1, Blue_close4CruiseSpeed1, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(0.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr())
      ),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, Blue_close4Waypoint2, Blue_close4PointSpeed2, Blue_close4CruiseSpeed2, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(0.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr())
      ),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, Blue_close4Waypoint3, Blue_close4PointSpeed3, Blue_close4CruiseSpeed3, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(0.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr())
      )
    );
  }
  else if(chosenAuto == "Red_Close4")
  {
    m_drive.ResetOdometry(Red_Close4Waypoint1[0]);
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      frc2::cmd::RunOnce(
        [this]
        {
          m_drive.ZeroHeading(180_deg);
        },
      {&m_drive}),
      AutoShooterWarmupCmd(m_shooterWheels).ToPtr(),
      frc2::cmd::Race(
        AutoSubAim(m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(0.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr())
      ),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, Red_Close4Waypoint1, Red_Close4PointSpeed1, Red_Close4CruiseSpeed1, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(0.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr())
      ),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, Red_Close4Waypoint2, Red_Close4PointSpeed2, Red_Close4CruiseSpeed2, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(0.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr())
      ),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, Red_Close4Waypoint3, Red_Close4PointSpeed3, Red_Close4CruiseSpeed3, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(0.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr())
      )
    );
  }
  else if(chosenAuto == "Blue_FarSideMid")
  {
    m_drive.ResetOdometry(Blue_FarSideMid1[0]);
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      frc2::cmd::RunOnce(
        [this]
        {
          m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
        },
        {&m_drive}),
      AutoShooterWarmupCmd(m_shooterWheels).ToPtr(),
      FollowWaypoints(m_drive, m_limelight, Blue_FarSideMid1, Blue_FarSideMidPoint1, Blue_FarSideMidCruise1, false).ToPtr(),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(1.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr())
      ),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, Blue_FarSideMid2, Blue_FarSideMidPoint2, Blue_FarSideMidCruise2, false).ToPtr(),
        frc2::cmd::Race(
          IntakeCmd(m_intake).ToPtr(),
          frc2::WaitCommand(5.5_s).ToPtr()
        )
      ),
      FollowWaypoints(m_drive, m_limelight, Blue_FarSideMid3, Blue_FarSideMidPoint3, Blue_FarSideMidCruise3, false).ToPtr(),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(1.5_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr()
        )
      ),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, Blue_FarSideMid4, Blue_FarSideMidPoint4, Blue_FarSideMidCruise4, false).ToPtr(),
        IntakeCmd(m_intake).ToPtr()
      ),
      FollowWaypoints(m_drive, m_limelight, Blue_FarSideMid5, Blue_FarSideMidPoint5, Blue_FarSideMidCruise5, false).ToPtr(),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(0.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr() 
        )
      )
    );
  }
  else if(chosenAuto == "Red_FarSideMid")
  {
    m_drive.ResetOdometry(Red_FarSideMid1[0]);
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      frc2::cmd::RunOnce(
        [this]
        {
          m_drive.ZeroHeading(180_deg);
        },
        {&m_drive}
      ),
      AutoShooterWarmupCmd(m_shooterWheels).ToPtr(),
      FollowWaypoints(m_drive, m_limelight, Red_FarSideMid1, Red_FarSideMidPoint1, Red_FarSideMidCruise1, false).ToPtr(),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(1.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr()
        )
      ),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, Red_FarSideMid2, Red_FarSideMidPoint2, Red_FarSideMidCruise2, false).ToPtr(),
        frc2::cmd::Race(
          IntakeCmd(m_intake).ToPtr(),
          frc2::WaitCommand(5.5_s).ToPtr()
        )
      ),
      FollowWaypoints(m_drive, m_limelight, Red_FarSideMid3, Red_FarSideMidPoint3, Red_FarSideMidCruise3, false).ToPtr(),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(0.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr()
        )
      ),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, Red_FarSideMid4, Red_FarSideMidPoint4, Red_FarSideMidCruise4, false).ToPtr(),
        frc2::cmd::Race(
          IntakeCmd(m_intake).ToPtr(),
          frc2::WaitCommand(5.5_s).ToPtr()
        )
      ),
      FollowWaypoints(m_drive, m_limelight, Red_FarSideMid5, Red_FarSideMidPoint5, Red_FarSideMidCruise5, false).ToPtr(),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(0.75_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr() 
        )
      )
    );
  }
  /*
  else if(chosenAuto == "Blue_AmpSide")
  {
    m_drive.ResetOdometry(Blue_AmpSide1[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      frc2::cmd::RunOnce(
        [this]
        {
          m_drive.ZeroHeading(m_drive.GetPose().Rotation().Degrees());
        },
        {&m_drive}),
      AutoShooterWarmupCmd(m_shooterWheels).ToPtr(),
      FollowWaypoints(m_drive, m_limelight, Blue_AmpSide1, Blue_AmpSidePoint1, Blue_AmpSideCruise1, false).ToPtr(),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(1.25_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr()
        )
      ),
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, Blue_AmpSide2, Blue_AmpSidePoint2, Blue_AmpSideCruise2, false).ToPtr(),
        frc2::cmd::Race(
          IntakeCmd(m_intake).ToPtr(),
          frc2::WaitCommand(3_s).ToPtr()
        )
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(1.25_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr()
        )
      ), 
      frc2::cmd::Parallel(
        FollowWaypoints(m_drive, m_limelight, Blue_AmpSide3, Blue_AmpSidePoint3, Blue_AmpSideCruise3, false).ToPtr(),
        frc2::cmd::Race(
          IntakeCmd(m_intake).ToPtr(),
          frc2::WaitCommand(5_s).ToPtr()
        )
      ),
      FollowWaypoints(m_drive, m_limelight, Blue_AmpSide4, Blue_AmpSidePoint4, Blue_AmpSideCruise4, false).ToPtr(),
      frc2::cmd::Race( //aim shooter for 0.75s
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr(),
        frc2::cmd::Sequence(
          frc2::WaitCommand(1.25_s).ToPtr(), //can change if need
          AutoShootCommand(m_shooterWheels, m_intake).ToPtr()
        )
      )
    );
  }
  */
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