#include "RobotContainer.h"

using namespace DriveConstants;

RobotContainer::RobotContainer()
{
  // Initialize all of your commands and subsystems here
  std::cout << "cout in robot container" << std::endl;

  m_chooser.SetDefaultOption("basic_auto", "basic_auto");
  m_chooser.AddOption("JustShoot", "JustShoot");
  m_chooser.AddOption("sideShootToback", "sideShootToback");

  //paths commented out for now since they are unused
  
  m_chooser.AddOption("B_1", "B_1");
  // m_chooser.AddOption("B_2", "B_2");
  // m_chooser.AddOption("B_3", "B_3");
  // m_chooser.AddOption("B_1_2_3", "B_1_2_3");
  // m_chooser.AddOption("B_3_6", "B_3_6");
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
  m_drive.ZeroHeading(); //resets the heading on the gyro

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
}

void RobotContainer::ConfigureButtonBindings()
{
  //Resets the heading of the gyro. In other words, it resets which way the robot thinks is the front
  // frc2::JoystickButton(&m_driverController, 7).OnTrue(m_drive.ZeroHeading());

  // // Robot slides right (when front is away from the drivers)
  // frc2::JoystickButton(&m_driverController, 1).WhileTrue(m_drive.Twitch(true));
  // // Robot slides left (when front is away from the drivers)
  // frc2::JoystickButton(&m_driverController, 2).WhileTrue(m_drive.Twitch(false));

  // //Limelight Note Detection
  //frc2::JoystickButton(&m_driverController, 1).ToggleOnTrue(NoteFollower(m_limelight, m_drive, m_driverController, m_intake, m_shooter, m_arm).ToPtr());

  // //Limelight April Tag Detection, y
  // frc2::JoystickButton(&m_driverController, 4).ToggleOnTrue(AprilTagFollower(m_limelight, m_drive, m_driverController, m_shooter).ToPtr());

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

frc2::CommandPtr RobotContainer::GetAuxilaryStateMachine()
{
  return StateMachine(
      m_drive,
      m_limelight,
      m_arm, 
      m_climb,
      m_color, //can remove?
      m_intake,
      m_shooter,
      m_driverController,
      m_auxController
    ).ToPtr();
}

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

frc2::CommandPtr RobotContainer::GetAutoAuxilaryStateMachine(){
  return AutoAuxilaryStateMachine(
      m_arm, 
      m_climb, //can remove?
      m_color, //can remove.
      m_intake,
      m_shooter, 
      m_driverController,
      m_auxController,
      driveShooterMessager
    ).ToPtr();
}

frc2::CommandPtr RobotContainer::GetAutoDriveStateMachine(){

  // m_drive.ResetOdometry(B_1Waypoints[0]);

  return AutoDriveStateMachine(
    m_drive,
    m_limelight,
    m_driverController,
    m_auxController,
    driveShooterMessager,
    path
  ).ToPtr();
}

// void RobotContainer::SetRanAuto(bool ranAuto){
//   m_drive.SetRanAuto(ranAuto);
// }

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{

  m_drive.ZeroHeading();

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
  
  if(chosenAuto == "basic_auto")
  {
    m_drive.ResetOdometry(B_1Waypoints[0]);
    // m_drive.SetAngleAdjustment(180);
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      frc2::cmd::RunOnce(
        [this]
          {
            m_shooter.zeroIntergralVal();
          }
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        frc2::cmd::Run(
          [this]
            {
              double filteredTargetID = m_limelight.GetFilteredTarget().GetFiducialId();
              double magEncoderPos = m_shooter.GetCurrMagEncoderVal();
              m_shooter.accumulateError();
              if(filteredTargetID == 7 || filteredTargetID == 4)
              {
                m_shooter.ApriltagShooterTheta(m_limelight.FilteredDistance(), magEncoderPos);
              }
            }
        ),
        frc2::WaitCommand(1.5_s).ToPtr() //can change if need
      ),
      frc2::cmd::Sequence( //shoot a note and turn back off mechanisms
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.SetShooter(0.75, 0.75);
            }
        ),
        frc2::WaitCommand(0.75_s).ToPtr(),
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.runMagazine(1);
              m_arm.runArmWheels(1);
              m_intake.runIntake(1);
              m_intake.Direction(1);
            }
        ),
        frc2::WaitCommand(1.5_s).ToPtr(),
        frc2::cmd::RunOnce(
          [this]  
            {
              m_shooter.SetShooter(0, 0);
              m_shooter.runMagazine(0);
              m_arm.runArmWheels(0);
              m_intake.runIntake(0);
              m_intake.Direction(0); 
              m_shooter.SetIntakePose();
            }
        )
      ),
      frc2::cmd::Parallel( // follow path and turn on intake until a note has been seen by the mag
        FollowWaypoints(m_drive, m_limelight, B_1Waypoints, B_1PointSpeed, B_1CruiseSpeed, false).ToPtr(),
        frc2::FunctionalCommand(
          [this] 
            { //init
              m_intake.runIntake(0.25);
              m_intake.DirectionNote(0.25); //will always run direction backwards unless note detected in front, useful here as pickup will only be from back atm
              m_arm.runArmWheels(0.25);
              m_shooter.runMagazine(0.25);
            },
          [this] 
            { //execute
              //do nothin.
            },
          [this] (bool interrupted)
            { //on end
              m_intake.runIntake(0);
              m_intake.DirectionNote(0);
              m_arm.runArmWheels(0);
              m_shooter.runMagazine(0);
            },
          [this]
            { //isfinished
              return m_shooter.GetMagazineSensor();
            },
          {&m_intake, &m_shooter, &m_arm}
        ).ToPtr()
      ),
      frc2::cmd::Sequence( //does a backup with the mag to prime the notes position
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.runMagazine(-0.2);
            }
        ),
        frc2::WaitCommand(0.11_s).ToPtr(),
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.runMagazine(0);
            }
        )
      ),
      frc2::cmd::Race( //primes the shooter again as before
        frc2::cmd::Run(
          [this]
            {
              double filteredTargetID = m_limelight.GetFilteredTarget().GetFiducialId();
              double magEncoderPos = m_shooter.GetCurrMagEncoderVal();
              m_shooter.accumulateError();
              if(filteredTargetID == 7 || filteredTargetID == 4)
              {
                m_shooter.ApriltagShooterTheta(m_limelight.FilteredDistance(), magEncoderPos);
              }
            }
        ),
        frc2::WaitCommand(2_s).ToPtr()
      ),
      frc2::cmd::Sequence( //shoots note
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.SetShooter(0.75, 0.75);
            }
        ),
        frc2::WaitCommand(0.75_s).ToPtr(),
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.runMagazine(1);
              m_arm.runArmWheels(1);
              m_intake.runIntake(1);
              m_intake.Direction(1);
            }
        ),
        frc2::WaitCommand(1.5_s).ToPtr(),
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.SetShooter(0, 0);
              m_shooter.runMagazine(0);
              m_arm.runArmWheels(0);
              m_intake.runIntake(0);
              m_intake.Direction(0);  
            }
        )
      )
    );
    
    // to deploy to run no auto. --------------------------------------------------------------

    // return frc2::cmd::Sequence(
    //   frc2::WaitCommand(0.1_s).ToPtr()  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
    //   // FollowWaypoints(m_drive, m_limelight, B_1Waypoints, B_1PointSpeed, B_1CruiseSpeed, false).ToPtr()
    // );  

    //-----------------------------------------------------------------
  }
  else if(chosenAuto == "JustShoot")
  { 
    // m_drive.SetAngleAdjustment(180);
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      frc2::cmd::RunOnce(
        [this]
          {
            m_shooter.zeroIntergralVal();
          }
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        frc2::cmd::Run(
          [this]
            {
              double filteredTargetID = m_limelight.GetFilteredTarget().GetFiducialId();
              double magEncoderPos = m_shooter.GetCurrMagEncoderVal();
              m_shooter.accumulateError();
              if(filteredTargetID == 7 || filteredTargetID == 4)
              {
                m_shooter.ApriltagShooterTheta(m_limelight.FilteredDistance(), magEncoderPos);
              }
            }
        ),
        frc2::WaitCommand(1.5_s).ToPtr() //can change if need
      ),
      frc2::cmd::Sequence( //shoot a note and turn back off mechanisms
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.SetShooter(0.75, 0.75);
            }
        ),
        frc2::WaitCommand(0.75_s).ToPtr(),
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.runMagazine(1);
              m_arm.runArmWheels(1);
              m_intake.runIntake(1);
              m_intake.Direction(1);
            }
        ),
        frc2::WaitCommand(1.5_s).ToPtr(),
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.SetShooter(0, 0);
              m_shooter.runMagazine(0);
              m_arm.runArmWheels(0);
              m_intake.runIntake(0);
              m_intake.Direction(0);  
            }
        )
      )
    );
  }
  else if(chosenAuto == "sideShootToback")
  {
    return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      frc2::cmd::RunOnce(
        [this]
          {
            m_shooter.zeroIntergralVal();
          }
      ),
      frc2::cmd::Race( //aim shooter for 0.75s
        frc2::cmd::Run(
          [this]
            {
              double filteredTargetID = m_limelight.GetFilteredTarget().GetFiducialId();
              double magEncoderPos = m_shooter.GetCurrMagEncoderVal();
              m_shooter.accumulateError();
              if(filteredTargetID == 7 || filteredTargetID == 4)
              {
                m_shooter.ApriltagShooterTheta(m_limelight.FilteredDistance(), magEncoderPos);
              }
            }
        ),
        frc2::WaitCommand(1.5_s).ToPtr() //can change if need
      ),
      frc2::cmd::Sequence( //shoot a note and turn back off mechanisms
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.SetShooter(0.75, 0.75);
            }
        ),
        frc2::WaitCommand(0.75_s).ToPtr(),
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.runMagazine(1);
              m_arm.runArmWheels(1);
              m_intake.runIntake(1);
              m_intake.Direction(1);
            }
        ),
        frc2::WaitCommand(1.5_s).ToPtr(),
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.SetShooter(0, 0);
              m_shooter.runMagazine(0);
              m_arm.runArmWheels(0);
              m_intake.runIntake(0);
              m_intake.Direction(0);  
            }
        )
      ),
      FollowWaypoints(m_drive, m_limelight, B_2Waypoints, B_2PointSpeed, B_2CruiseSpeed, false).ToPtr()
    );
  }
  else if(chosenAuto == "B_1")
  {
    m_drive.ResetOdometry(B_1Waypoints[0]);
      return frc2::cmd::Sequence( //the whole auto path!
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time.
      frc2::cmd::Parallel( 
        frc2::cmd::Race( //aim shooter for 0.75s
          frc2::cmd::Run(
            [this]
              {
                double magEncoderPos = m_shooter.GetCurrMagEncoderVal();
                m_shooter.ApriltagShooterTheta(m_limelight.FilteredDistance(), magEncoderPos);
              }
          ),
          frc2::WaitCommand(1.5_s).ToPtr() //can change if need
        ),
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr()
      ),
      frc2::cmd::Sequence( //shoot a note and turn back off mechanisms
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.SetShooter(0.75, 0.75);
            }
        ),
        frc2::WaitCommand(0.75_s).ToPtr(),
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.runMagazine(1);
              m_arm.runArmWheels(1);
              m_intake.runIntake(1);
              m_intake.Direction(1);
            }
        ),
        frc2::WaitCommand(1.5_s).ToPtr(),
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.SetShooter(0, 0);
              m_shooter.runMagazine(0);
              m_arm.runArmWheels(0);
              m_intake.runIntake(0);
              m_intake.Direction(0);  
            }
        )
      ),
      frc2::cmd::Parallel( // follow path and turn on intake until a note has been seen by the mag
        FollowWaypoints(m_drive, m_limelight, B_1Waypoints, B_1PointSpeed, B_1CruiseSpeed, false).ToPtr(),
        frc2::FunctionalCommand(
          [this] 
            { //init
              m_intake.runIntake(0.25);
              m_intake.DirectionNote(0.25); //will always run direction backwards unless note detected in front, useful here as pickup will only be from back atm
              m_arm.runArmWheels(0.25);
              m_shooter.runMagazine(0.25);
            },
          [this] 
            { //execute
              //do nothin.
            },
          [this] (bool interrupted) //This is neccesary or it breaks
            { //on end
              m_intake.runIntake(0);
              m_intake.DirectionNote(0);
              m_arm.runArmWheels(0);
              m_shooter.runMagazine(0);
            },
          [this]
            { //isfinished
              return m_shooter.GetMagazineSensor();
            },
          {&m_intake, &m_shooter, &m_arm}
        ).ToPtr()
      ),
      frc2::cmd::Sequence( //does a backup with the mag to prime the notes position
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.runMagazine(-0.2);
            }
        ),
        frc2::WaitCommand(0.11_s).ToPtr(),
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.runMagazine(0);
            }
        )
      ),
      frc2::cmd::Parallel( 
        frc2::cmd::Race( //aim shooter for 0.75s
          frc2::cmd::Run(
            [this]
              {
                double magEncoderPos = m_shooter.GetCurrMagEncoderVal();
                m_shooter.ApriltagShooterTheta(m_limelight.FilteredDistance(), magEncoderPos);
              }
          ),
          frc2::WaitCommand(1.5_s).ToPtr() //can change if need
        ),
        AutoAprilTag(m_limelight, m_drive, m_shooter).ToPtr()
      ),
      frc2::cmd::Sequence( //shoots note
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.SetShooter(0.75, 0.75);
            }
        ),
        frc2::WaitCommand(0.75_s).ToPtr(),
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.runMagazine(1);
              m_arm.runArmWheels(1);
              m_intake.runIntake(1);
              m_intake.Direction(1);
            }
        ),
        frc2::WaitCommand(1.5_s).ToPtr(),
        frc2::cmd::RunOnce(
          [this]
            {
              m_shooter.SetShooter(0, 0);
              m_shooter.runMagazine(0);
              m_arm.runArmWheels(0);
              m_intake.runIntake(0);
              m_intake.Direction(0);  
            }
        )
      )
    );
  }
  else if(chosenAuto == "B_2")
  { 
    m_drive.ResetOdometry(B_2Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, B_2Waypoints, B_2PointSpeed, B_2CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "B_3")
  { 
    m_drive.ResetOdometry(B_3Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, B_3Waypoints, B_3PointSpeed, B_3CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "B_1_2_3")
  { 
    m_drive.ResetOdometry(B_1_2_3Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, B_1_2_3Waypoints, B_1_2_3PointSpeed, B_1_2_3CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "B_3_6")
  { 
    m_drive.ResetOdometry(B_3_6Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, B_3_6Waypoints, B_3_6PointSpeed, B_3_6CruiseSpeed, true).ToPtr()
    );  
  }
  else if(chosenAuto == "B_3_7")
  { 
    m_drive.ResetOdometry(B_3_7Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, B_3_7Waypoints, B_3_7PointSpeed, B_3_7CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "B_2_6")
  { 
    m_drive.ResetOdometry(B_2_6Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, B_2_6Waypoints, B_2_6PointSpeed, B_2_6CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "B_2_7")
  { 
    m_drive.ResetOdometry(B_2_7Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, B_2_7Waypoints, B_2_7PointSpeed, B_2_7CruiseSpeed, false).ToPtr()
    );  
  }
    else if(chosenAuto == "B_1_4")
  { 
    m_drive.ResetOdometry(B_1_4Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, B_1_4Waypoints, B_1_4PointSpeed, B_1_4CruiseSpeed, false).ToPtr()
    );  
  }
    else if(chosenAuto == "B_3_8")
  { 
    m_drive.ResetOdometry(B_3_8Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, B_3_8Waypoints, B_3_8PointSpeed, B_3_8CruiseSpeed, false).ToPtr()
    );  
  }
    else if(chosenAuto == "B_2_5")
  { 
    m_drive.ResetOdometry(B_2_5Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, B_2_5Waypoints, B_2_5PointSpeed, B_2_5CruiseSpeed, false).ToPtr()
    );  
  }
    else if(chosenAuto == "R_1")
  { 
    m_drive.ResetOdometry(R_1Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, R_1Waypoints, R_1PointSpeed, R_1CruiseSpeed, false).ToPtr()
    );  
  }
    else if(chosenAuto == "R_2")
  { 
    m_drive.ResetOdometry(R_2Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, R_2Waypoints, R_2PointSpeed, R_2CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "R_3")
  { 
    m_drive.ResetOdometry(R_3Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, R_3Waypoints, R_3PointSpeed, R_3CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "R_3_6")
  { 
    m_drive.ResetOdometry(R_3_6Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, R_3_6Waypoints, R_3_6PointSpeed, R_3_6CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "R_3_7")
  { 
    m_drive.ResetOdometry(R_3_7Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, R_3_7Waypoints, R_3_7PointSpeed, R_3_7CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "B_3_2_1")
  { 
    m_drive.ResetOdometry(B_3_2_1Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, B_3_2_1Waypoints, B_3_2_1PointSpeed, B_3_2_1CruiseSpeed, false).ToPtr()
    );  
  }
   else if(chosenAuto == "B_1_5")
  { 
    m_drive.ResetOdometry(B_1_5Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, B_1_5Waypoints, B_1_5PointSpeed, B_1_5CruiseSpeed, false).ToPtr()
    );  
  }
   else if(chosenAuto == "B_3_7_8")
  { 
    m_drive.ResetOdometry(B_3_7_8Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, B_3_7_8Waypoints, B_3_7_8PointSpeed, B_3_7_8CruiseSpeed, false).ToPtr()
    );  
  }
   else if(chosenAuto == "B_1_4_5")
  { 
    m_drive.ResetOdometry(B_1_4_5Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, B_1_4_5Waypoints, B_1_4_5PointSpeed, B_1_4_5CruiseSpeed, false).ToPtr()
    );  
  }
 else if(chosenAuto == "B_2_6_7")
  { 
    m_drive.ResetOdometry(B_2_6_7Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, B_2_6_7Waypoints, B_2_6_7PointSpeed, B_2_6_7CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "B_3_6_7")
  { 
    m_drive.ResetOdometry(B_3_6_7Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, B_3_6_7Waypoints, B_3_6_7PointSpeed, B_3_6_7CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "B_2_6_5")
  { 
    m_drive.ResetOdometry(B_2_6_5Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, B_2_6_5Waypoints, B_2_6_5PointSpeed, B_2_6_5CruiseSpeed, false).ToPtr()
    );  
  }
   else if(chosenAuto == "B_3_2_1_4_5")
  { 
    m_drive.ResetOdometry(B_3_2_1_4_5Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, B_3_2_1_4_5Waypoints, B_3_2_1_4_5PointSpeed, B_3_2_1_4_5CruiseSpeed, false).ToPtr()
    );  
  }
   else if(chosenAuto == "R_1_2_3")
  { 
    m_drive.ResetOdometry(R_1_2_3Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, R_1_2_3Waypoints, R_1_2_3PointSpeed, R_1_2_3CruiseSpeed, false).ToPtr()
    );  
  }
   else if(chosenAuto == "R_2_6")
  { 
    m_drive.ResetOdometry(R_2_6Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, R_2_6Waypoints, R_2_6PointSpeed, R_2_6CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "R_2_7")
  { 
    m_drive.ResetOdometry(R_2_7Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, R_2_7Waypoints, R_2_7PointSpeed, R_2_7CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "R_3_8")
  { 
    m_drive.ResetOdometry(R_3_8Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, R_3_8Waypoints, R_3_8PointSpeed, R_3_8CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "R_1_4")
  { 
    m_drive.ResetOdometry(R_1_4Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, R_1_4Waypoints, R_1_4PointSpeed, R_1_4CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "R_2_5")
  { 
    m_drive.ResetOdometry(R_2_5Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, R_2_5Waypoints, R_2_5PointSpeed, R_2_5CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "R_3_2_1")
  { 
    m_drive.ResetOdometry(R_3_2_1Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, R_3_2_1Waypoints, R_3_2_1PointSpeed, R_3_2_1CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "R_1_5")
  { 
    m_drive.ResetOdometry(R_1_5Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, R_1_5Waypoints, R_1_5PointSpeed, R_1_5CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "R_3_7_8")
  { 
    m_drive.ResetOdometry(R_3_7_8Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, R_3_7_8Waypoints, R_3_7_8PointSpeed, R_3_7_8CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "R_1_4_5")
  { 
    m_drive.ResetOdometry(R_1_4_5Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, R_1_4_5Waypoints, R_1_4_5PointSpeed, R_1_4_5CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "R_2_6_7")
  { 
    m_drive.ResetOdometry(R_2_6_7Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, R_2_6_7Waypoints, R_2_6_7PointSpeed, R_2_6_7CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "R_3_6_7")
  { 
    m_drive.ResetOdometry(R_3_6_7Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, R_3_6_7Waypoints, R_3_6_7PointSpeed, R_3_6_7CruiseSpeed, false).ToPtr()
    );  
  }
  else if(chosenAuto == "R_2_6_5")
  { 
    m_drive.ResetOdometry(R_2_6_5Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, R_2_6_5Waypoints, R_2_6_5PointSpeed, R_2_6_5CruiseSpeed, false).ToPtr()
    );  
  }
   else if(chosenAuto == "R_2_5_6")
  { 
    m_drive.ResetOdometry(R_2_5_6Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, R_2_5_6Waypoints, R_2_5_6PointSpeed, R_2_5_6CruiseSpeed, false).ToPtr()
    );  
  }
   else if(chosenAuto == "R_3_2_1_4_5")
  { 
    m_drive.ResetOdometry(R_3_2_1_4_5Waypoints[0]);
    return frc2::cmd::Sequence(
      frc2::WaitCommand(0.1_s).ToPtr(),  //This is neccesary because the reset odometry will not actually reset until after a very small amount of time. 
      FollowWaypoints(m_drive, m_limelight, R_3_2_1_4_5Waypoints, R_3_2_1_4_5PointSpeed, R_3_2_1_4_5CruiseSpeed, false).ToPtr()
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