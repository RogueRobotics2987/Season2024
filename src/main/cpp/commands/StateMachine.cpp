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
  m_shooter->zeroIntergralVal();
  m_shooter->setRestingActuatorPosition();

  nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("pipeline", 0);
  frc::SmartDashboard::PutNumber("Driver Angle", 20);

  if(m_shooter->GetMagazineSensor())
  {
    state = LOADED;
    magEncoderPos = m_shooter->GetCurrMagEncoderVal();
  }
}

// Called repeatedly when this Command is scheduled to run
void StateMachine::Execute()
{
  frc::SmartDashboard::PutBoolean("Note Follow", noteFollowState);
  frc::SmartDashboard::PutBoolean("April Follow", aprilFollowState);

  //double Driver_Angle = frc::SmartDashboard::GetNumber("Driver Angle", 30 );
  // if (frc::SmartDashboard::GetNumber("Driver Angle", 20 )< 20){

  // }else if (frc::SmartDashboard::GetNumber("Driver Angle", 20 )>21 && frc::SmartDashboard::GetNumber("Driver Angle", 20 )<90){
  //    m_shooter->SetActuator(frc::SmartDashboard::GetNumber("Driver Angle", 20 ));
  // }
  frc::SmartDashboard::PutBoolean("Pick up note?: ", pickupNote);
  targetIDs.clear();

  //TODO: is this going to interfear with the limelight subsystem?
// Define Camera
// Find out what we need to do to get strgest tag id
// calculate camra tangent get camera dx and dy and get tan of that
  photon::PhotonCamera camera = photon::PhotonCamera("FrontCamera");
  photon::PhotonPipelineResult result = camera.GetLatestResult();
  bool hasTarget = result.HasTargets();

  frc::SmartDashboard::PutBoolean("hasTarget", hasTarget);

  if(hasTarget)
  {
    //TODO clean up variable names to make them more understandable in this if statement
    std::span<const photon::PhotonTrackedTarget> tempTargets = result.GetTargets();
    myTargets.assign(tempTargets.begin(), tempTargets.end());

    for(unsigned int i = 0; i < myTargets.size(); i++)
    {
      targetIDs.emplace_back(myTargets.at(i).GetFiducialId());

      if(myTargets.at(i).GetFiducialId() == 4)
      {
        filteredTarget = myTargets.at(i);
        filteredTargetID = filteredTarget.GetFiducialId();
        //TODO have the yaw on our robot search for 0. include our specific id into the calc dist to target
        frc::SmartDashboard::PutNumber("FilteredYaw", filteredTarget.GetYaw());
        frc::SmartDashboard::PutNumber("FilteredPitch", filteredTarget.GetPitch());

        filteredRange = photon::PhotonUtils::CalculateDistanceToTarget(
          CAMERA_HEIGHT, TAREGT_HEIGHT, CAMERA_PITCH,
        units::degree_t{filteredTarget.GetPitch()});
        frc::SmartDashboard::PutNumber("FilteredRange", filteredRange.value());
      }
    }

    photon::PhotonTrackedTarget target = result.GetBestTarget();
    
    units::meter_t range = photon::PhotonUtils::CalculateDistanceToTarget(
      CAMERA_HEIGHT, TAREGT_HEIGHT, CAMERA_PITCH,
    units::degree_t{target.GetPitch()});
  
    frc::SmartDashboard::PutNumber("DistanceToTarget", range.value());

    double yaw = target.GetYaw();
    int targetID = target.GetFiducialId();

    frc::SmartDashboard::PutNumberArray("Targets", targetIDs);

    frc::SmartDashboard::PutNumber("TargetID", targetID);
    frc::SmartDashboard::PutNumber("TargetYaw", yaw);

  }

  if (result.MultiTagResult().result.isPresent) {
    wpi::SmallVector<int16_t, 32U> fieldToCamera = result.MultiTagResult().fiducialIdsUsed;
    std::vector<double> temp;
    temp.assign(fieldToCamera.begin(), fieldToCamera.end());
    frc::SmartDashboard::PutNumberArray("MegaPoseID", temp);

    frc::Transform3d multi = result.MultiTagResult().result.best;

    frc::SmartDashboard::PutNumber("MegaPoseRoll", (multi.Rotation().X().value() * (180/3.14159)));
    frc::SmartDashboard::PutNumber("MegaPosePitch", (multi.Rotation().Y().value() * (180/3.14159)));
    frc::SmartDashboard::PutNumber("MegaPoseYaw", (multi.Rotation().Z().value()  * (180/3.14159)));
    frc::SmartDashboard::PutNumber("MegaPoseX", multi.X().value());
    frc::SmartDashboard::PutNumber("MegaPoseY", multi.Y().value());
    frc::SmartDashboard::PutNumber("MegaPoseZ", multi.Z().value());
    // frc::SmartDashboard::PutNumber("MegaPoseTranslation", multi.Translation());
  }
  
  

  // RedDistVector = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumberArray("botpose_wpired", std::span<const double>({0, 0, 0, 0, 0, 0}));
  // BlueDistVector = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumberArray("botpose_wpiblue", std::span<const double>({0, 0, 0, 0, 0, 0}));

  // blueDist = BlueDistVector[0];
  // redDist = RedDistVector[0];

  // apriltagID = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("tid", 0);

  m_shooter->accumulateError();
  m_shooter->SetShooterAngle();

  // frc::SmartDashboard::PutNumber("angle test value", angleTest);

  // BUTTONS!!!
  // if(m_driverController->GetRawButton(1)){
  //   m_arm->setVoltage(0.1);
  // }
  // else{
  //   m_arm->setVoltage(0);
  // }

  if(m_driverController->GetRawButtonPressed(1))
  {
    noteFollowState = !noteFollowState;
  }

  if(m_driverController->GetRawButtonPressed(2))
  {
    aprilFollowState = !aprilFollowState;
  }

  if(m_driverController->GetRawButtonPressed(5))
  { 
    //TODO: trace code
    pickupNote = !pickupNote;
  }

  if(m_driverController->GetRawButtonPressed(6))
  { 
    warmUpShooter = !warmUpShooter;
  }

  if(m_driverController->GetRawAxis(3) > 0.05/*|| m_auxController->GetRawButtonPressed(8)*/)
  {
    moveNote2Shoot = true;

  } else{
    moveNote2Shoot = false;
  }

  
  // if(m_driverController->GetRawButtonPressed(4)){
  //   huntingNote = !huntingNote;
  // }

  if(m_auxController->GetPOV(0) == 0)
  {
    pov0 = !pov0;
  }
  

//  if(m_auxController->GetRawButtonPressed(2))
//   {
//     pov0 = !pov0;
//   }

  if(m_auxController->GetRawButtonPressed(2)){
    if(placeInForwardAmp == false){
      placeInForwardAmp = true;
      placeInTrap = false;
    } else {
      placeInForwardAmp = false;
    }
  } 
  if(m_auxController->GetRawButtonPressed(4))
  { 
    if(placeInTrap == false){
      placeInTrap = true;
      placeInForwardAmp = false;
    } else {
      placeInTrap = false;
    }
  }
  // if(m_auxController->GetRawButton(4)){
  //   m_arm->MoveLowerArm();
  // }

  if(m_auxController->GetRawButtonPressed(8))
  {

  }

  if(fabs(m_auxController->GetRightY()) > 0.15)
  {
    m_shooter->JoystickActuator(m_auxController->GetRightY());
  }

  m_shooter->AngleTrimAdjust(m_auxController->GetRawButtonPressed(6), m_auxController->GetRawButtonPressed(5));



  // state machine
  switch (state)
  {
  case EMPTY:     // turn everything off
    frc::SmartDashboard::PutString("state: ", "EMPTY");
    // stop all motors
    m_arm->stopDrop();
    //m_arm->setLowerArmAngle(ArmConstants::LowerFullRetractedAngle);
    //m_arm->setUpperArmAngle(ArmConstants::UpperFullRetractedAngle);
    m_intake->stopIntake();
    m_shooter->stopMagazine();
    m_shooter->StopShooter();
    m_arm->stopArmWheels();
    

    m_shooter->SetIntakePose(); 

    if(pickupNote == true)
    {
      state = PICKUP;   
      frc::SmartDashboard::PutString("state: ", "changing to PICKUP");
    }
    

    //TODO: THERE IS NO MESSENGER
    /*
    if(m_messager->GetDriveMessage().compare("runIntake") == 0)
    {
      state = PICKUP;
      pickupNote = true;
    }
    */


    // if(pov0 == true)
    // {
    //   state = SPIT_OUT;
    //   frc::SmartDashboard::PutString("state: ", "changing to SPIT_OUT");
    // }

    // if(huntingNote == true){
    //   state = NOTE_HUNTING;
    //   frc::SmartDashboard::PutString("state: ", "changing to Hunting Note");

    // }

    if(placeInTrap || placeInForwardAmp){
      state = RAISE_SHOOTER;
    }


    break;

  case SPIT_OUT:
    frc::SmartDashboard::PutString("state: ", "SPIT_OUT");
    m_shooter->runMagazine(-0.2);
    m_arm->runArmWheels(-0.2);
    m_intake->spitOutIntake();
    
    if(pov0 == false)
    {
      state = EMPTY;
      frc::SmartDashboard::PutString("state: ", "changing to EMPTY");
    }

    break;

  case PICKUP:    // start intake and magazine
    // m_shooter->driveActuator(m_auxController->GetRightY());
    frc::SmartDashboard::PutString("state: ", "PICKUP");

    m_shooter->SetIntakePose();
    
    // start intake motors, REMEMBER: middle motor changes direction
    m_intake->runIntake(0.25);
    m_intake->Direction(0.25);

    if(m_intake->GetIntakeFront() || m_intake->GetIntakeRear())
    {
      m_arm->runArmWheels(0.25);
      m_shooter->runMagazine(0.25);  //TODO test this function, might not have behaved correctly first test
    }

    if(pickupNote == false)
    {
      state = EMPTY;
      frc::SmartDashboard::PutString("state: ", "changing to EMPTY");
    }
    // else if(emptyIntake == true){
    //   state = SPIT_OUT;
    //   frc::SmartDashboard::PutString("state: ", "changing to SPIT_OUT");
    // }
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

    //TODO: THERE IS NO MESSENGER
    /*
    if(m_messager->GetDriveMessage().compare("Empty") == 0)
    {
      state = EMPTY;
      pickupNote = false;
    }
    */

    // if(pov0 == true)
    // {
    //   state = SPIT_OUT;
    //   frc::SmartDashboard::PutString("state: ", "changing to SPIT_OUT");
    // }
    
    break;

  case BACKUP:
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
      m_arm->stopArmWheels();
      m_intake->stopIntake();
      time = 0;
      state = LOADED;
    }

    time++;

    break;
    
  case LOADED:    // self explanitory
    frc::SmartDashboard::PutString("state: ", "LOADED");
    //pickupNote = false;

    // turn running motors off
    m_intake->stopIntake();
    //m_shooter->holdMagazine(magEncoderPos);
    m_arm->stopArmWheels();
    m_shooter->stopMagazine();
    m_shooter->StopShooter();

    if(filteredTargetID == 7 || filteredTargetID == 4)
    {
      //frc::SmartDashboard::PutNumber("Shooter Angle Theta",filteredRange.value());
      m_shooter->ApriltagShooterTheta(filteredRange.value());
    }

    if(warmUpShooter == true)
    {
      state = SHOOTER_WARMUP;
      frc::SmartDashboard::PutString("state: ", "changing to SHOOTER_WARMUP");

    } 

    // if(pov0 == true)
    // {
    //   state = SPIT_OUT;
    //   frc::SmartDashboard::PutString("state: ", "changing to SPIT_OUT");
    // }

    // if(placeInTrap || placeInAmp){
    //   state = RAISE_SHOOTER;
    //   frc::SmartDashboard::PutString("state: ", "changing to RAISE_SHOOTER");
    // }
    /*else if(moveArm2Drop == true){
      state = DROP_WARMUP;
      frc::SmartDashboard::PutString("state: ", "changing to DROP_WARMUP");
    }*/

    break;

  case SHOOTER_WARMUP:
    frc::SmartDashboard::PutString("state: ", "SHOOTER_WARMUP");

    if(filteredTargetID == 7 || filteredTargetID == 4)
    {
      //frc::SmartDashboard::PutNumber("Shooter Angle Theta",filteredRange.value());
      m_shooter->ApriltagShooterTheta(filteredRange.value());
    }

    //start shooter motors
    m_shooter->SetShooter(0.75, 0.75);

    if(warmUpShooter == false)
    {
      state = LOADED;
      magEncoderPos = m_shooter->GetCurrMagEncoderVal();
      frc::SmartDashboard::PutString("state: ", "changing to LOADED");
    } 
    
    if(moveNote2Shoot == true)
    {
      state = SHOOT;
      frc::SmartDashboard::PutString("state: ", "changing to SHOOT");
    }

    break;

  case SHOOT:
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
    }

    break;

  //TODO THIS CODE BELOW HAS NOT BEEN TESTED, PLEASE TEST BEFORE CONTINUING

  case RAISE_SHOOTER:
    m_shooter->SetActuator(ShooterConstants::RaisedShooterAngle);

    //switch states when timer has exceded 1.5 seconds
    //run 60 times a second
    time++;

    if(time >= 90){
      state = ARMS_EXTEND_INITIAL;
      frc::SmartDashboard::PutString("state: ", "changing to LOWER_ARM_EXTEND_INITIAL");
      time = 0;
    }

    break;

  case ARMS_EXTEND_INITIAL:
    frc::SmartDashboard::PutString("state: ", "LOWER_ARM_EXTEND_INITAL");

    m_arm->setLowerArmAngle(ArmConstants::LowerFirstExtentionAngle);
    // m_arm->setUpperArmAngle(ArmConstants::UpperFirstExtentionAngle);
    //switch states when timer has exceded 5.0 seconds
    //run 60 times a second
    time++;

    if(time >= 300)
    {
      if(placeInTrap)
      {
        state = ARM_TRAP;
        frc::SmartDashboard::PutString("state: ", "changing to ARM_TRAP");
      }
      else if (placeInForwardAmp)
      {
        state = FORWARD_ARM_AMP;
        frc::SmartDashboard::PutString("state: ", "changing to ARM_AMP");
      }
      else if (placeInBackwardsAmp)
      {
        state = BACKWARD_ARM_AMP;
        frc::SmartDashboard::PutString("state: ", "changing to ARM_AMP");
      }
      else
      {
        state = ARM_RETRACT_INITIAL;
        frc::SmartDashboard::PutString("state: ", "changing to ARM_RETRACT_INITIAL");
      }
      time = 0;
    }

    break;

      // case UPPER_ARM_EXTEND_INITIAL:
    //   frc::SmartDashboard::PutString("state: ", "UPPER_ARM_EXTEND_INITAL");

    //   m_arm->setLowerArmAngle(ArmConstants::LowerExtentionAngle);
    //   m_arm->setUpperArmAngle(ArmConstants::UpperExtentionAngle);
    //   //switch states when timer has exceded 1.0 seconds
    //   //run 60 times a second
    //   time++;

    //   if(time >= 60){
    //     if(placeInTrap){
    //       state = ARM_TRAP;
    //       frc::SmartDashboard::PutString("state: ", "changing to ARM_TRAP");
    //     }
    //     else if (placeInAmp){
    //       state = ARM_AMP;
    //       frc::SmartDashboard::PutString("state: ", "changing to ARM_AMP");
    //     }
    //     else{
    //       state = ARM_RETRACT_INITIAL;
    //       frc::SmartDashboard::PutString("state: ", "changing to ARM_RETRACT_INITIAL");
    //     }
    //     time = 0;
    //   }


  //   break;

  case FORWARD_ARM_AMP:
    frc::SmartDashboard::PutString("state: ", "ARM_AMP");

    m_arm->setLowerArmAngle(ArmConstants::LowerForwardAmpExtentionAngle);
    // m_arm->setUpperArmAngle(ArmConstants::UpperForwardAmpExtentionAngle);

    //switch states when timer has exceded 1.0 seconds
    //run 60 times a second
    time++;

    if(time >= 300)
    {
      state = DROP;
      frc::SmartDashboard::PutString("state: ", "changing to DROP");
      time = 0;
    }

    break;

    case BACKWARD_ARM_AMP:
      frc::SmartDashboard::PutString("state: ", "ARM_AMP");

      m_arm->setLowerArmAngle(ArmConstants::LowerBackwardAmpExtentionAngle);
      // m_arm->setUpperArmAngle(ArmConstants::UpperBackwardAmpExtentionAngle);

      //switch states when timer has exceded 1.0 seconds
      //run 60 times a second
      time++;

      if(time >= 300)
      {
        state = DROP;
        frc::SmartDashboard::PutString("state: ", "changing to DROP");
        time = 0;
      }

    break;

    case ARM_TRAP:
      frc::SmartDashboard::PutString("state: ", "ARM_TRAP");

      m_arm->setLowerArmAngle(ArmConstants::LowerTrapExtentionAngle);
      // m_arm->setUpperArmAngle(ArmConstants::UpperTrapExtentionAngle);

      //switch states when timer has exceded 5 .0 seconds
      //run 60 times a second
      time++;

      if(time >= 300)
      {
        state = DROP;
        frc::SmartDashboard::PutString("state: ", "changing to DROP");
        time = 0;
      }

    break;

    case DROP:
      frc::SmartDashboard::PutString("state: ", "DROP");
      //m_arm->dropNote();
      // m_arm->runArmWheels(0.5); //temp speed value
      //switch states when timer has exceded 1.0 seconds
      //run 60 times a second
      time++;

      if(time >= 0)
      {
        state = ARM_RETRACT_INITIAL;
        frc::SmartDashboard::PutString("state: ", "changing to ARM_RETRACT_INITIAL");
        time = 0;
      }

    break;

    case ARM_RETRACT_INITIAL:
      frc::SmartDashboard::PutString("state: ", "ARM_RETRACT_INITAL");

      // m_arm->setLowerArmAngle(ArmConstants::LowerFirstRetractionAngle);
      // m_arm->setUpperArmAngle(ArmConstants::UpperFirstRetractionAngle);
      //switch states when timer has exceded 1.0 seconds
      //run 60 times a second
      time++;

      if(time >= 300)
      {
        state = ARM_RETRACT_FINAL;
        frc::SmartDashboard::PutString("state: ", "changing to ARM_RETRACT_FINAL");
        time = 0;
      }

    break;

    case ARM_RETRACT_FINAL:
      frc::SmartDashboard::PutString("state: ", "ARM_RETRACT_FINAL");

      m_arm->setLowerArmAngle(ArmConstants::LowerInitialAngle);
      // m_arm->setUpperArmAngle(ArmConstants::UpperInitialAngle); 
      //switch states when timer has exceded 1.0 seconds
      //run 60 times a second
      time++;

      if(time >= 300)
      {
        state = DROP_SHOOTER;
        frc::SmartDashboard::PutString("state: ", "changing to DROP_SHOOTER");
        time = 0;
      }

    break;
  
    case DROP_SHOOTER:
      frc::SmartDashboard::PutString("state: ", "DROP_SHOOTER");

      m_shooter->SetActuator(ShooterConstants::RestingAngle);
      // if(m_Xbox->getr)
      //switch states when timer has exceded 1.5 seconds
      //run 60 times a second
      time++;

      if(time >= 90)
      {
        placeInForwardAmp = false;
        placeInTrap = false;
        placeInBackwardsAmp = false; 
        state = EMPTY;
        frc::SmartDashboard::PutString("state: ", "changing to EMPTY");
        time = 0;
      }

    break;

    //  case NOTE_HUNTING:

    //   m_intake->runIntake();
    //   m_intake->Direction();
    //   m_arm->runArmWheels(0.4);
    //     m_shooter->runMagazine(0.4); 

    //  tx = m_limelight->GetNotetx();
    // if(tx > 7 || tx < -7){
    //   rot = units::angular_velocity::radians_per_second_t((0 - tx) * kp);
    // }
    // else
    // {
    //   rot = units::angular_velocity::radians_per_second_t(0);
    // }
    // speedY = Deadzone(m_driverController->GetLeftY());

    // if((fabs(speedY) + fabs(rot.value())) < .05) 
    // {
    //   NoJoystickInput = true;
    // }
    // else
    // {
    //   NoJoystickInput = false;
    // }

    // m_drivetrain->Drive(units::velocity::meters_per_second_t(speedY) * 6.7, units::velocity::meters_per_second_t(0), rot, false, NoJoystickInput);

    // if(m_shooter->GetMagazineSensor() == true){
    //     state = LOADED;
    //     frc::SmartDashboard::PutString("state: ", "changing to LOADED");
    //   }

    // break;


    // case DROP_WARMUP:
    //   frc::SmartDashboard::PutString("state: ", "DROP_WARMUP");

    //   //lift shooter and extend arm
    //   if(m_arm->getLowerEncoderPos() < 25 /*&&/|| shooter is at okay angle*/){   //double check algorithm
    //     //lift shooter

    //     m_arm->setLowerArmAngle(90);

    //     m_arm->setUpperArmAngle(45);
    //   }
    //   /*turn on lower motor first
    //     turn on upper after it reaches safe point (90 degrees)
    //     17 = bottom; 18 = joint; 19 = drop???
    //   */

    //   if(moveArm2Drop == false){
    //     state = LOADED;
    //     frc::SmartDashboard::PutString("state: ", "changing to LOADED");

    //   } else if(dropNote == true){
    //     state = DROP;
    //     frc::SmartDashboard::PutString("state: ", "changing to DROP");
    //   }

    //   break;
    
    // case DROP:
    //   frc::SmartDashboard::PutString("state: ", "DROP");
    //   moveArm2Drop = false;

    //   //drop note
    //   m_arm->dropNote();

    //   timeDrop++;

    //   if(timeDrop >= 90){
    //     state = EMPTY;
    //     frc::SmartDashboard::PutString("state: ", "changing to EMPTY");

    //     timeDrop = 0;
    //     dropNote = false;
    //   }

    //   break;

    default:
      state = EMPTY;
      break;
  }
}

// Called once the command ends or is interrupted.
void StateMachine::End(bool interrupted) {}

// Returns true when the command should end.
bool StateMachine::IsFinished() {
  return false;
}

//TODO: THIS WAS NOT COMMENTED OUT IN DRIVE STATE MACHINE
/*
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
*/