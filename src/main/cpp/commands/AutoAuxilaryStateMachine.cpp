// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoAuxilaryStateMachine.h"

AutoAuxilaryStateMachine::AutoAuxilaryStateMachine() {}

AutoAuxilaryStateMachine::AutoAuxilaryStateMachine(
  ArmSubsystem &arm,
  ClimberSubsystem &climb,
  ColorSensorSubsystem &color, 
  IntakeSubsystem &intake,
  ShooterSubsystem &shooter, 
  frc::XboxController &driveXbox,
  frc::XboxController &auxXbox,
  CommandMessenger &message)
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
  m_messager = &message;

  m_driverController = &driveXbox;
  m_auxController = &auxXbox;
}

// Called when the command is initially scheduled.
void AutoAuxilaryStateMachine::Initialize()
{
  m_shooter->zeroIntergralVal();
  m_shooter->setRestingActuatorPosition();

  state = BACKUP;

  // if(m_shooter->GetMagazineSensor())
  // {
  //   state = BACKUP;
  //   magEncoderPos = m_shooter->GetCurrMagEncoderVal();
  // }

}


// Called repeatedly when this Command is scheduled to run
void AutoAuxilaryStateMachine::Execute()
{  
  frc::SmartDashboard::PutBoolean("Pick up note?: ", pickupNote);

  //For the Shooter Apriltag test
  targetIDs.clear();

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

      if(myTargets.at(i).GetFiducialId() == 4 || myTargets.at(i).GetFiducialId() == 7)
      {
        filteredTarget = myTargets.at(i);
        filteredTargetID = filteredTarget.GetFiducialId();
        //TODO have the yaw on our robot search for 0. include our specific id into the calc dist to target

        filteredRange = photon::PhotonUtils::CalculateDistanceToTarget(
          CAMERA_HEIGHT, TAREGT_HEIGHT, CAMERA_PITCH,
        units::degree_t{filteredTarget.GetPitch()});

        if(DebugConstants::debugLimelight == true){
            frc::SmartDashboard::PutNumber("FilteredRange", filteredRange.value());
            frc::SmartDashboard::PutNumber("FilteredYaw", filteredTarget.GetYaw());
            frc::SmartDashboard::PutNumber("FilteredPitch", filteredTarget.GetPitch());
        }
      }
    }

    photon::PhotonTrackedTarget target = result.GetBestTarget();
    
    units::meter_t range = photon::PhotonUtils::CalculateDistanceToTarget(
      CAMERA_HEIGHT, TAREGT_HEIGHT, CAMERA_PITCH,
    units::degree_t{target.GetPitch()});
  
    double yaw = target.GetYaw();
    int targetID = target.GetFiducialId();

    if(DebugConstants::debugLimelight == true)
    {
        frc::SmartDashboard::PutNumber("DistanceToTarget", range.value());
        frc::SmartDashboard::PutNumberArray("Targets", targetIDs);
        frc::SmartDashboard::PutNumber("TargetID", targetID);
        frc::SmartDashboard::PutNumber("TargetYaw", yaw);
    }
  }

  if (result.MultiTagResult().result.isPresent)
  {
    wpi::SmallVector<int16_t, 32U> fieldToCamera = result.MultiTagResult().fiducialIdsUsed;
    std::vector<double> temp;
    temp.assign(fieldToCamera.begin(), fieldToCamera.end());

    frc::Transform3d multi = result.MultiTagResult().result.best;

    if(DebugConstants::debugLimelight == true)
    {
        frc::SmartDashboard::PutNumberArray("MegaPoseID", temp);
        frc::SmartDashboard::PutNumber("MegaPoseRoll", (multi.Rotation().X().value() * (180/3.14159)));
        frc::SmartDashboard::PutNumber("MegaPosePitch", (multi.Rotation().Y().value() * (180/3.14159)));
        frc::SmartDashboard::PutNumber("MegaPoseYaw", (multi.Rotation().Z().value()  * (180/3.14159)));
        frc::SmartDashboard::PutNumber("MegaPoseX", multi.X().value());
        frc::SmartDashboard::PutNumber("MegaPoseY", multi.Y().value());
        frc::SmartDashboard::PutNumber("MegaPoseZ", multi.Z().value());
        // frc::SmartDashboard::PutNumber("MegaPoseTranslation", multi.Translation());
    }
  }
  
  

  // RedDistVector = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumberArray("botpose_wpired", std::span<const double>({0, 0, 0, 0, 0, 0}));
  // BlueDistVector = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumberArray("botpose_wpiblue", std::span<const double>({0, 0, 0, 0, 0, 0}));

  // blueDist = BlueDistVector[0];
  // redDist = RedDistVector[0];

  // apriltagID = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("tid", 0);

//   m_shooter->accumulateError();
//   m_shooter->SetShooterAngle();


  // BUTTONS!!!
//   if(m_driverController->GetRawButtonPressed(5))
//   { 
//     //TODO: trace code
//     pickupNote = !pickupNote;
//   }

  if(lastMessage.compare(m_messager->GetDriveMessage()) != 0)
  {
    std::cout << m_messager->GetDriveMessage() << std::endl;
    lastMessage = m_messager->GetDriveMessage();
  }
 

  if(m_messager->GetDriveMessage().compare("TurnOnIntake") == 0)
  {
    pickupNote = true;
  }

// std::cout << "Drive Message " << m_messager->GetDriveMessage() << std::endl;

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
    m_arm->StopWheels();

    m_shooter->SetIntakePose();

    if(pickupNote == true)
    {
      state = PICKUP;   
      frc::SmartDashboard::PutString("state: ", "changing to PICKUP");
    }
    
    // if(pov0 == true)
    // {
    //   state = SPIT_OUT;
    //   frc::SmartDashboard::PutString("state: ", "changing to SPIT_OUT");
    // }

    // if(huntingNote == true){
    //   state = NOTE_HUNTING;
    //   frc::SmartDashboard::PutString("state: ", "changing to Hunting Note");

    // }

    /*if(placeInTrap || placeInAmp){
      state = RAISE_SHOOTER;
    }*/

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
    else if(m_shooter->GetMagazineSensor())
    {
      state = BACKUP;
      pickupNote = false;

      magEncoderPos = m_shooter->GetCurrMagEncoderVal();

      m_intake->stopIntake();
      m_arm->StopWheels();
      m_shooter->stopMagazine();
      m_shooter->StopShooter();

      frc::SmartDashboard::PutString("state: ", "changing to LOADED");
    }

    // if(pov0 == true)
    // {
    //   state = SPIT_OUT;
    //   frc::SmartDashboard::PutString("state: ", "changing to SPIT_OUT");
    // }
    
    break;

  case BACKUP:
    frc::SmartDashboard::PutString("state: ", "BACKUP");

    // if(m_messager->GetDriveMessage().compare("InitShoot") == 0){
    //   m_messager->SetDriveMessage("Shoot");
    // }

    if(time<7)
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
      m_messager->SetAuxMessage("HasNote"); //tells the driveState machine when the robot has a note
    }

    time++;

    break;
    
  case LOADED:    // self explanitory
    frc::SmartDashboard::PutString("state: ", "LOADED");

    pickupNote = false;

    // if(m_messager->GetDriveMessage().compare("InitShoot") == 0){
    //   state = BACKUP;
    // }

    // turn running motors off
    m_intake->stopIntake();
    m_arm->StopWheels();
    m_shooter->holdMagazine(magEncoderPos);
    m_shooter->StopShooter();

    frc::SmartDashboard::PutNumber("filteredRange2", filteredRange.value());

    if(filteredTargetID == 7 || filteredTargetID == 4)
    {
      m_shooter->ApriltagShooterTheta(filteredRange.value());
    }

    if((fabs(m_shooter->GetOffSetEncoderValue() - m_shooter->GetDesired()) < 5 && m_messager->GetDriveMessage().compare("Shoot") == 0 ) && fabs(filteredTarget.GetYaw()) <= 5)
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

    if(time > 100)
    {
      state = SHOOT;
      frc::SmartDashboard::PutString("state: ", "changing to SHOOT");
      time = 0;
    }
    
    time++;

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
      state = NEXT_PATH;
      frc::SmartDashboard::PutString("state: ", "changing to EMPTY");

      time = 0;
      moveNote2Shoot = false;
    }

    break;

  //TODO THIS CODE BELOW HAS NOT BEEN TESTED, PLEASE TEST BEFORE CONTINUING

  case NEXT_PATH:
    frc::SmartDashboard::PutString("state: ", "NEXT PATH");
    std::cout << "NextPath: AutoAux452" << std::endl;

    m_messager->SetAuxMessage("NextPath");

    pickupNote = false;
    state = EMPTY;

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
void AutoAuxilaryStateMachine::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoAuxilaryStateMachine::IsFinished()
{
  return false;
}

// float StateMachine::Deadzone(float x)
// {
//   if ((x < 0.1) &&  (x > -0.1)){
//     x=0;
//   }
//   else if (x >= 0.1){
//     x = x - 0.1;
//   }
//   else if (x <= -0.1){
//     x = x + 0.1;
//   }
//   return(x);
// }