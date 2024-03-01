// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/StateMachine.h"

StateMachine::StateMachine() {}

StateMachine::StateMachine(
  DriveSubsystem &drive,
  LimelightSubsystem &limelight,
  ArmSubsystem &arm,
  ClimberSubsystem &climb,
  ColorSensorSubsystem &color,
  IntakeSubsystem &intake,
  ShooterSubsystem &shooter, 
  frc::XboxController &driverController,
  frc::XboxController &auxController)
{
  m_arm = &arm;
  AddRequirements({m_arm});
  m_climb = &climb;
  AddRequirements({m_climb});
  m_colorSensor = &color;
  AddRequirements({m_colorSensor});
  m_intake = &intake;
  AddRequirements({m_intake});
  m_shooter = &shooter;
  AddRequirements({m_shooter});
  m_limelight = &limelight;
  AddRequirements({m_limelight});
  m_drive = &drive;
  AddRequirements({m_drive});

  m_driverController = &driverController;
  m_auxController = &auxController;
}

// Called when the command is initially scheduled.
void StateMachine::Initialize()
{
  //TODO: TEST!!!!!
  SetBoolsFalse();

  m_shooter->zeroIntergralVal();
  m_shooter->setRestingActuatorPosition();

  m_arm->ZeroIntergral();

  m_arm->setLowerArmAngle(1);

  nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("pipeline", 0);
  frc::SmartDashboard::PutNumber("Driver Angle", 20);

  state = EMPTY;

  if(m_shooter->GetMagazineSensor())
  {
    state = BACKUP;
    magEncoderPos = m_shooter->GetCurrMagEncoderVal();
  }
}

// Called repeatedly when this Command is scheduled to run
void StateMachine::Execute()
{
  frc::SmartDashboard::PutBoolean("Note Follow", noteFollowState);
  frc::SmartDashboard::PutBoolean("April Follow", aprilFollowState);
  frc::SmartDashboard::PutBoolean("Pick up note?: ", pickupNote);

  filteredTargetID = m_limelight->GetFilteredTarget().GetFiducialId();
 
  m_shooter->accumulateError();
  m_shooter->SetShooterAngle(); //TODO this should probably get removed...

  m_arm->accumulateErrorLower();
  m_arm->accumulateErrorUpper();
  


  // BUTTONS!!!
  driveButtonA = m_driverController->GetRawButtonPressed(1);
  driveButtonB = m_driverController->GetRawButtonPressed(2);
  driveLeftBumperPushed = m_driverController->GetRawButtonPressed(5);

  // std::cout << "Debug:" << state << std::endl;
  // std::cout << "Debug:" << noteFollowState << std::endl;
  // std::cout << "Debug: button A " << buttonA << std::endl;

  frc::SmartDashboard::PutBoolean("ButtonA", driveButtonA);
  frc::SmartDashboard::PutBoolean("ButtonB", driveButtonB);

  if(driveButtonA && state == EMPTY)
  {
    // std::cout << "debug" << 93 << std::endl;
    noteFollowState = true;
    pickupNote = true;
  }
  else if(driveButtonA && state == PICKUP && noteFollowState == true)
  {
  // std::cout << "debug" << 99 << std::endl;
    noteFollowState = false;
    pickupNote = false;
  }
  else if(state == LOADED)
  {
    // std::cout << "debug" << 105 << std::endl;
    noteFollowState = false;
    pickupNote = false;
  }

  if(driveButtonB && state == SHOOTER_WARMUP)
  {
    aprilFollowState = !aprilFollowState;
  }
  else if(state != SHOOTER_WARMUP)
  {
    aprilFollowState = false;
  }

  /*

  if(m_auxController->GetLeftY())
  {
    lowerArmTrap = true;
  }

  if(m_auxController->GetRightY())
  {
    upperArmTrap = true;
  }
  */

  if(m_driverController->GetRawButtonPressed(3))
  {
    chainClimb = !chainClimb;
  }

  if(driveLeftBumperPushed && state == EMPTY)
  { 
    pickupNote = true;
  }
  
  else if(driveLeftBumperPushed && state == PICKUP && noteFollowState == false)
  {
    pickupNote = false;
  }

  if(m_driverController->GetRawButtonPressed(7))
  {
    m_drive->ZeroHeading();
  }

  //this will get lost sometimes where you cannot turn it off while april tag tracker is being used
  if(m_driverController->GetRawButtonPressed(6))
  { 
    warmUpShooter = !warmUpShooter;
  }

  if(m_driverController->GetRawAxis(3) > 0.05)
  {
    moveNote2Shoot = true;

  } else{
    moveNote2Shoot = false;
  }

  if(m_auxController->GetPOV() != -1 && m_auxController->GetPOV() == 0)
  {
    emptyIntake = true;
  }  
  else
  {
    emptyIntake = false;
  }

  if(m_driverController->GetPOV() != -1 && m_driverController->GetPOV() == 90)
  {
    // raiseClimber = true;
    m_climb->startClimber();
  }
  else 
  {
    // raiseClimber = false;
    m_climb->stopClimber();
  }

/*
  if(m_auxController->GetRawButtonPressed(2)){
    if(placeInForwardAmp == false)
    {
      placeInForwardAmp = true;
      placeInTrap = false;
    }
    else
    {
      placeInForwardAmp = false;
    }
  } 
  if(m_auxController->GetRawButtonPressed(4))
  { 
    if(placeInTrap == false)
    {
      placeInTrap = true;
      placeInForwardAmp = false;
    } 
    else
    {
      placeInTrap = false;
    }
  }
  */

  if(m_driverController->GetRawButtonPressed(8))
  {
    resetLoaded = true;
    magEncoderPos = m_shooter->GetCurrMagEncoderVal();
  }

  if(fabs(m_auxController->GetRightY()) > 0.15)
  {
    m_shooter->JoystickActuator(m_auxController->GetRightY());
  }

  m_shooter->AngleTrimAdjust(m_auxController->GetRawButtonPressed(6), m_auxController->GetRawButtonPressed(5));

  frc::SmartDashboard::PutBoolean("noteFollowState", noteFollowState);
  frc::SmartDashboard::PutBoolean("aprilFollowState", aprilFollowState); //remove(?)

  if(noteFollowState != true && aprilFollowState != true)
  {
    //regular drive
    frc::SmartDashboard::PutString("DriveConfiguration ", "DefaultDrive");

    speedY = Deadzone(m_driverController->GetLeftY());
    speedX = Deadzone(m_driverController->GetLeftX());
    rot = Deadzone(m_driverController->GetRightX());

    if((fabs(speedY) + fabs(speedX) + fabs(rot)) < .05)
    {
      NoJoystickInput = true;
    }
    else
    {
      NoJoystickInput = false;
    }

    m_drive->Drive(units::velocity::meters_per_second_t(-speedY * AutoConstants::kMaxSpeed), units::velocity::meters_per_second_t(-speedX * AutoConstants::kMaxSpeed), units::radians_per_second_t(-rot * AutoConstants::kMaxAngularSpeed), true, NoJoystickInput);
  }
  else if(noteFollowState == true)
  {
    frc::SmartDashboard::PutString("DriveConfiguration ", "NoteFollow");

    //limited drive, else regular
    if(nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("tv", 0) == 1)
    {
      txNote = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("tx", 0.0);

      rotNote = units::angular_velocity::radians_per_second_t((0 - txNote) * kpNote);

      speedY = Deadzone(m_driverController->GetLeftY());

      if((fabs(speedY) + fabs(rotNote.value())) < .05)
      {
        NoJoystickInput = true;
      }
      else
      {
        NoJoystickInput = false;
      }
      m_drive->Drive(units::velocity::meters_per_second_t(speedY * AutoConstants::kMaxSpeed), units::velocity::meters_per_second_t(0), rotNote, false, NoJoystickInput);
    } 
    else 
    {
      speedY = Deadzone(m_driverController->GetLeftY());
      speedX = Deadzone(m_driverController->GetLeftX());
      rot = Deadzone(m_driverController->GetRightX());

      if((fabs(speedY) + fabs(speedX) + fabs(rot)) < .05)
      {
        NoJoystickInput = true;
      }
      else
      {
        NoJoystickInput = false;
      }

      m_drive->Drive(units::velocity::meters_per_second_t(-speedY * AutoConstants::kMaxSpeed), units::velocity::meters_per_second_t(-speedX * AutoConstants::kMaxSpeed), units::radians_per_second_t(-rot * AutoConstants::kMaxAngularSpeed), true, NoJoystickInput);
    }
  }
  else if(aprilFollowState == true)
  {
    frc::SmartDashboard::PutString("DriveConfiguration ", "AprilFollow");


    if(m_limelight->PhotonHasTarget() == true)  //limited drive, else regular
    {
      currentHeading = m_drive->GetPose().Rotation().Degrees().value();

      if (filteredTargetID == 4 || filteredTargetID == 7)
      {
        txApril = m_limelight->FilteredPhotonYaw(); //m_limelight->GetAprilTagtx() - 5; // TODO: check
        desiredHeading = currentHeading + txApril;
      }

      frc::SmartDashboard::PutNumber("filtered yaw val", txApril);

      double error = DistanceBetweenAngles(desiredHeading, currentHeading);

      rotApril = units::angular_velocity::radians_per_second_t(error * kpApril);

 
      //rotApril = units::angular_velocity::radians_per_second_t(0);
      //if(txApril > 7 || txApril < -7){
      // rotApril = units::angular_velocity::radians_per_second_t((0 - txApril) * kpApril);
      //}

      speedY = Deadzone(m_driverController->GetLeftY());
      speedX = Deadzone(m_driverController->GetLeftX());

      if((fabs(speedY) + fabs(speedX) + fabs(rotApril.value())) < .05)
      {
        NoJoystickInput = true;
      }
      else
      {
        NoJoystickInput = false;
      }
        
      m_drive->Drive(units::velocity::meters_per_second_t(-speedY  * AutoConstants::kMaxSpeed), units::velocity::meters_per_second_t(-speedX * AutoConstants::kMaxSpeed), -rotApril, false, NoJoystickInput);

    } 
    else 
    {
      speedY = Deadzone(m_driverController->GetLeftY());
      speedX = Deadzone(m_driverController->GetLeftX());
      rot = Deadzone(m_driverController->GetRightX());

      if((fabs(speedY) + fabs(speedX) + fabs(rot)) < .05)
      {
        NoJoystickInput = true;
      }
      else
      {
        NoJoystickInput = false;
      }

      m_drive->Drive(units::velocity::meters_per_second_t(-speedY * AutoConstants::kMaxSpeed), units::velocity::meters_per_second_t(-speedX * AutoConstants::kMaxSpeed), units::radians_per_second_t(-rot * AutoConstants::kMaxAngularSpeed), true, NoJoystickInput);
    }
  }

  // state machine
  switch (state)
  {
    case EMPTY:
    {     // turn everything off
      frc::SmartDashboard::PutString("state: ", "EMPTY");
      if(resetLoaded)
      {
        state = LOADED;
        SetBoolsFalse();
        break;
      }

      // stop all motors
      m_arm->setLowerArmAngle(0.5);
      m_arm->setUpperArmAngle(0.5);
      m_arm->stopDrop();
      m_intake->stopIntake();
      m_shooter->stopMagazine();
      m_shooter->StopShooter();
      m_arm->stopArmWheels();
      m_shooter->SetIntakePose(); 

      if(pickupNote == true)
      {
        state = PICKUP;   
        frc::SmartDashboard::PutString("state: ", "changing to PICKUP");
        m_intake->DirectionNote(0.25);
      }

      if(chainClimb == true)
      {
        state = CHAIN_CLIMB;
        frc::SmartDashboard::PutString("state:", "changing to CHAIN_CLIMB");
      }
    
      if(emptyIntake == true)
      {
        state = SPIT_OUT;
        frc::SmartDashboard::PutString("state: ", "changing to SPIT_OUT");
      }

      break;
    }

    case SPIT_OUT:
    {
      frc::SmartDashboard::PutString("state: ", "SPIT_OUT");
      if(resetLoaded)
      {
        state = LOADED;
        SetBoolsFalse();
        break;
      }

      m_shooter->runMagazine(-0.2);
      m_arm->runArmWheels(-0.2);
      m_intake->spitOutIntake();
      
      if(emptyIntake == false)
      {
        state = EMPTY;
        frc::SmartDashboard::PutString("state: ", "changing to EMPTY");
      }

      break;
    }

    case PICKUP:    // start intake and magazine
    {
      frc::SmartDashboard::PutString("state: ", "PICKUP");

      if(resetLoaded)
      {
        state = LOADED;
        SetBoolsFalse();
        break;
      }

      m_shooter->SetIntakePose();
      
      // start intake motors, REMEMBER: middle motor changes direction
      m_intake->runIntake(0.25);
      m_intake->Direction(0.25);

      if(m_intake->GetIntakeFront() || m_intake->GetIntakeRear())
      {
        m_arm->runArmWheels(0.25);
        m_shooter->runMagazine(0.25);
      }

      if(pickupNote == false)
      {
        state = EMPTY;
        frc::SmartDashboard::PutString("state: ", "changing to EMPTY");
      }

      if(m_shooter->GetMagazineSensor())
      {
        state = BACKUP;
        pickupNote = false;

        magEncoderPos = m_shooter->GetCurrMagEncoderVal();

        m_intake->stopIntake();
        m_arm->stopArmWheels();
        m_shooter->stopMagazine();
        m_shooter->StopShooter();

        frc::SmartDashboard::PutString("state: ", "changing to LOADED");
      }

      if(emptyIntake == true)
      {
        state = SPIT_OUT;
        frc::SmartDashboard::PutString("state: ", "changing to SPIT_OUT");
      }
      
      break;
    }

    case BACKUP:
    {
      frc::SmartDashboard::PutString("state: ", "BACKUP");

      if(time < 7)
      {
        m_shooter->runMagazine(-0.2);
        m_arm->runArmWheels(-0.2);
        m_intake->spitOutIntake();
      }
      else
      {
        m_shooter->stopMagazine();
        magEncoderPos = m_shooter->GetCurrMagEncoderVal();
        m_arm->stopArmWheels();
        m_intake->stopIntake();
        time = 0;
        state = LOADED;
        noteFollowState = false;
      }

      time++;

      break;
    }
      
    case LOADED:    // self explanitory
    {
      frc::SmartDashboard::PutString("state: ", "LOADED");
      resetLoaded = false;
      pickupNote = false;

      // turn running motors off
      m_intake->stopIntake();
      m_arm->stopArmWheels();
      m_shooter->stopMagazine();
      m_shooter->StopShooter();

      //m_shooter->holdMagazine(magEncoderPos);

      if(filteredTargetID == 7 || filteredTargetID == 4)
      {
        //frc::SmartDashboard::PutNumber("Shooter Angle Theta",filteredRange.value());
        m_shooter->ApriltagShooterTheta(m_limelight->FilteredDistance(), magEncoderPos);
      }

      if(warmUpShooter == true)
      {
        state = SHOOTER_WARMUP;
        frc::SmartDashboard::PutString("state: ", "changing to SHOOTER_WARMUP");
      } 

      if(chainClimb == true)
      {
        state = CHAIN_CLIMB;
        frc::SmartDashboard::PutString("state:", "changing to CHAIN_CLIMB");
      }

      if(emptyIntake == true)
      {
        state = SPIT_OUT;
        frc::SmartDashboard::PutString("state: ", "changing to SPIT_OUT");
      }

      break;
    }

    case SHOOTER_WARMUP:
    {
      frc::SmartDashboard::PutString("state: ", "SHOOTER_WARMUP");
      if(resetLoaded)
      {
        state = LOADED;
        SetBoolsFalse();
        break;
      }

      //m_shooter->holdMagazine(magEncoderPos);

      if(filteredTargetID == 7 || filteredTargetID == 4)
      {
        //frc::SmartDashboard::PutNumber("Shooter Angle Theta",filteredRange.value());
        m_shooter->ApriltagShooterTheta(m_limelight->FilteredDistance(), magEncoderPos);
      }

      //start shooter motors
      m_shooter->SetShooter(0.75, 0.75);

      if(warmUpShooter == false)
      {
        state = LOADED;
        frc::SmartDashboard::PutString("state: ", "changing to LOADED");
      } 
      
      if(moveNote2Shoot == true)
      {
        state = SHOOT;
        frc::SmartDashboard::PutString("state: ", "changing to SHOOT");
      }

      break;
    }

    case SHOOT:
    {
      frc::SmartDashboard::PutString("state: ", "SHOOT");

      warmUpShooter = false;

      //turn on mag motors
      m_shooter->runMagazine(1);
      m_arm->runArmWheels(1);
      m_intake->runIntake(1);
      m_intake->Direction(1);

      //switch states when timer has exceded 1.5 seconds
      //run 60 times a second
      time++;

      if(time >= 60)
      {
        state = EMPTY;
        frc::SmartDashboard::PutString("state: ", "changing to EMPTY");

        time = 0;
        moveNote2Shoot = false;
        aprilFollowState = false;
      }

      break;
    }

    //TODO THIS CODE BELOW HAS NOT BEEN TESTED, PLEASE TEST BEFORE CONTINUING

    case FORWARD_ARM_AMP:
    {
      frc::SmartDashboard::PutString("state: ", "ARM_AMP");

      m_arm->setLowerArmAngle(ArmConstants::LowerForwardAmpExtentionAngle);

      time++;

      if(time >= 300)
      {
        //state = DROP;
        frc::SmartDashboard::PutString("state: ", "changing to DROP");
        time = 0;
      }

      break;
    }

    case BACKWARD_ARM_AMP:
    {
      frc::SmartDashboard::PutString("state: ", "ARM_AMP");

      m_arm->setLowerArmAngle(ArmConstants::LowerBackwardAmpExtentionAngle);

      //switch states when timer has exceded 1.0 seconds
      //run 60 times a second
      time++;

      if(time >= 300)
      {
        //state = DROP;
        frc::SmartDashboard::PutString("state: ", "changing to DROP");
        time = 0;
      }

      break;
    }

    case ARM_TRAP:
    {
      frc::SmartDashboard::PutString("state: ", "ARM_TRAP");

      m_arm->setLowerArmAngle(ArmConstants::LowerTrapExtentionAngle);
      // m_arm->setUpperArmAngle(ArmConstants::UpperTrapExtentionAngle);

      //switch states when timer has exceded 5 .0 seconds
      //run 60 times a second
      time++;

      if(time >= 300)
      {
        //state = DROP;
        frc::SmartDashboard::PutString("state: ", "changing to DROP");
        time = 0;
      }

      break;
    }

    case CHAIN_CLIMB:
    {
      frc::SmartDashboard::PutString("state: ", "CHAIN_CLIMB");

      //move shooter out of way
      // m_shooter->SetActuator(ShooterConstants::ShooterMaxSoftLimit);

      // if(time >= 65 && time < 140)
      // {
      //   //move arms out of way 
      //   m_arm->setLowerArmAngle(80);
      // }

      // if(time >= 140 && time < 200)
      // {
      //   //move arms out of way 
      //   m_arm->setUpperArmAngle(0);
      //   time = 360;
      // }
      
      // time++;
      
      //button 1 things
      // if(raiseClimber == true)
      // {
      //   m_climb->startClimber();
      // }
      // else
      // {
      //   m_climb->stopClimber();
      // }

      // //Driver
      // if(m_auxController->GetRawButtonPressed(1))
      // {
      //   m_arm->setLowerArmAngle(50);
      // }

      // if(m_auxController->GetRawButtonPressed(2))
      // {
      //   m_arm->setUpperArmAngle(160);
      // }

      // if(m_auxController->GetRawButtonPressed(3))
      // {
      //   m_arm->setLowerArmAngle(85);
      // }
    
      //COMMENTED OUT BECAUSE IS UNSTABLE RIGHT NOW DO NOT GO INTO YOUR CLIMBER BECAUSE YOU CANT GET OUT
      // if(chainClimb == false)
      // {
      //   state = ARMS_RETRACT;
      //   time = 0;
      //   m_arm->setLowerArmAngle(35);
      //   m_arm->setUpperArmAngle(0.5);
      // }

      if(chainClimb == false)
      {
        state = LOADED;
      }

      break;
    }
    case ARMS_RETRACT:
    {
      if(fabs(35 - m_arm->GetOffSetEncoderValueLower()) < 1)
      {
        m_shooter->SetActuator(40);
        m_arm->setLowerArmAngle(1);
      
        if(fabs(m_shooter->GetDesired() - m_shooter->GetOffSetEncoderValue()) < 2)
        {
          state = EMPTY;
        }
      }
      break;
    }

    default:
    {
      state = EMPTY;
      break;
    }
  }
}

// Called once the command ends or is interrupted.
void StateMachine::End(bool interrupted) {}

// Returns true when the command should end.
bool StateMachine::IsFinished() {
  return false;
}

float StateMachine::Deadzone(float x)
{
  if ((x < 0.1) &&  (x > -0.1))
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

double StateMachine::DistanceBetweenAngles(double targetAngle, double sourceAngle)
{
  double a = targetAngle - sourceAngle;
  if(a > 180)
  {
    a = a + -360;
  }
  else if(a < -180)
  {
    a = a + 360;
  }
  else
  {
    a = a;
  }

  return a;
}

void StateMachine::SetBoolsFalse()
{
  pickupNote = false;
  emptyIntake = false;       
  warmUpShooter = false;     
  moveNote2Shoot = false;

  placeInForwardAmp = false;
  placeInBackwardsAmp = false;
  placeInTrap = false;
  chainClimb = false;

  noteFollowState = false;
  aprilFollowState = false;
  driveButtonA = false;
  driveButtonB = false;
  driveLeftBumperPushed = false;

  resetLoaded = false;
}