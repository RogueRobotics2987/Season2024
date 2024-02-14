// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/StateMachine.h"

StateMachine::StateMachine() {}

StateMachine::StateMachine(ArmSubsystem &arm, ClimberSubsystem &climb, ColorSensorSubsystem &color, 
                           IntakeSubsystem &intake, ShooterSubsystem &shooter, 
                           frc::XboxController &driveXbox, frc::XboxController &auxXbox)
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

  m_driverController = &driveXbox;
  m_auxController = &auxXbox;
}

// Called when the command is initially scheduled.
void StateMachine::Initialize() {}


// Called repeatedly when this Command is scheduled to run
void StateMachine::Execute() {  
  frc::SmartDashboard::PutBoolean("Pick up note?: ", pickupNote);

  // BUTTONS!!!
  if(m_driverController->GetRawButtonPressed(5)){     //TODO: trace code
    if(pickupNote == false){
      pickupNote = true;

    } else {
      pickupNote = false;
    }
  }
  if(m_driverController->GetRawButtonPressed(6) || m_auxController->GetRawButtonPressed(8)){
    if(moveNote2Shoot == false){
      moveNote2Shoot = true;

    } else {
      moveNote2Shoot = false;
    }

  }
  if(m_driverController->GetRawAxis(3)){  // TODO: find alt; cannot toggle axis
    if(warmUpShooter == false){
      warmUpShooter = true;

    } else {
      warmUpShooter = false;
    }

  }
  if(m_auxController->GetPOV(0)){
    emptyIntake = true;

  }
  if(m_auxController->GetRawButtonPressed(2)){
    if(placeInAmp == false){
      placeInAmp = true;
      placeInTrap = false;

    } else {
      placeInAmp = false;
    }

  }
  if(m_auxController->GetRawButtonPressed(4)){
    if(placeInTrap == false){
      placeInTrap = true;
      placeInTrap = false;

    } else {
      placeInTrap = false;
    }

  }
        if(m_auxController->GetRawButtonPressed(8)){

        }



  // state machine
  switch (state) {
  case EMPTY:     // turn everything off
    frc::SmartDashboard::PutString("state: ", "EMPTY");
    // m_shooter->driveActuator(m_auxController->GetRightY());
    m_shooter->setRestingActuatorPosition();

    // stop all motors
    m_arm->stopDrop();
    m_arm->setLowerArmAngle(ArmConstants::LowerFullRetractedAngle);
    m_arm->setUpperArmAngle(ArmConstants::UpperFullRetractedAngle);
    m_intake->stopIntake();
    m_shooter->stopMagazine();
    m_shooter->StopShooter();


    if(pickupNote == true){
      
      state = PICKUP;   
      frc::SmartDashboard::PutString("state: ", "changing to PICKUP");
    } 

    /*if(placeInTrap || placeInAmp){
      state = RAISE_SHOOTER;
    }*/


    break;

  case SPIT_OUT:
    frc::SmartDashboard::PutString("state: ", "SPIT_OUT");

    //reverse intake

    if(m_colorSensor->detectNoteIntake1 == true){
      state = EMPTY;

      emptyIntake = false;
    }


    break;

  case PICKUP:    // start intake and magazine
     m_shooter->driveActuator(m_auxController->GetRightY());
    frc::SmartDashboard::PutString("state: ", "PICKUP");
    
    // start intake motors, REMEMBER: middle motor changes direction
    m_intake->runIntake();
    //m_arm-> //DC
    
    frc::SmartDashboard::PutBoolean("detect note?: ", m_colorSensor->detectNoteIntake1);


    if(pickupNote == false){
      state = EMPTY;
      frc::SmartDashboard::PutString("state: ", "changing to EMPTY");
    }
    // else if(emptyIntake == true){
    //   state = SPIT_OUT;
    //   frc::SmartDashboard::PutString("state: ", "changing to SPIT_OUT");

    // } else if(m_shooter->GetMagazineSensor() == true){
    //   state = LOADED;
    //   frc::SmartDashboard::PutString("state: ", "changing to LOADED");
    // }
    
    break;

  case LOADED:    // self explanitory
    frc::SmartDashboard::PutString("state: ", "LOADED");
    //pickupNote = false;

    // turn running motors off
    m_intake->stopIntake();
    //m_intake->stopMagazine(); // double check


    if(warmUpShooter == true){
      state = SHOOTER_WARMUP;
      frc::SmartDashboard::PutString("state: ", "changing to SHOOTER_WARMUP");

    } 
    if(placeInTrap || placeInAmp){
      state = RAISE_SHOOTER;
      frc::SmartDashboard::PutString("state: ", "changing to RAISE_SHOOTER");
    }
    /*else if(moveArm2Drop == true){
      state = DROP_WARMUP;
      frc::SmartDashboard::PutString("state: ", "changing to DROP_WARMUP");
    }*/


    break;

  case SHOOTER_WARMUP:
    frc::SmartDashboard::PutString("state: ", "SHOOTER_WARMUP");

    //start shooter motors
    m_shooter->SetShooter(0.5);


    if(warmUpShooter == false){
      state = LOADED;
      frc::SmartDashboard::PutString("state: ", "changing to LOADED");

    } else if(moveNote2Shoot == true){
      state = SHOOT;
      frc::SmartDashboard::PutString("state: ", "changing to SHOOT");
    }


    break;

  case SHOOT:
    frc::SmartDashboard::PutString("state: ", "SHOOT");
    warmUpShooter = false;

    //turn on mag motors
    m_shooter->runMagazine();


    //switch states when timer has exceded 1.5 seconds
    //run 60 times a second
    time++;

    if(time >= 90){
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
      state = LOWER_ARM_EXTEND_INITIAL;
      frc::SmartDashboard::PutString("state: ", "changing to LOWER_ARM_EXTEND_INITIAL");
      time = 0;
    }


    break;

  case LOWER_ARM_EXTEND_INITIAL:
    frc::SmartDashboard::PutString("state: ", "LOWER_ARM_EXTEND_INITAL");

    m_arm->setLowerArmAngle(ArmConstants::LowerFirstExtentionAngle);
    m_arm->setUpperArmAngle(ArmConstants::UpperFirstExtentionAngle);
    //switch states when timer has exceded 1.5 seconds
    //run 60 times a second
    time++;

    if(time >= 60){
      state = UPPER_ARM_EXTEND_INITIAL;
      frc::SmartDashboard::PutString("state: ", "changing to UPPER_ARM_EXTEND_INITIAL");
      time = 0;
    }


    break;

  case UPPER_ARM_EXTEND_INITIAL:
    frc::SmartDashboard::PutString("state: ", "UPPER_ARM_EXTEND_INITAL");

    m_arm->setLowerArmAngle(ArmConstants::LowerExtentionAngle);
    m_arm->setUpperArmAngle(ArmConstants::UpperExtentionAngle);
    //switch states when timer has exceded 1.5 seconds
    //run 60 times a second
    time++;

    if(time >= 60){
      if(placeInTrap){
        state = ARM_TRAP;
        frc::SmartDashboard::PutString("state: ", "changing to ARM_TRAP");
      }
      else if (placeInAmp){
        state = ARM_AMP;
        frc::SmartDashboard::PutString("state: ", "changing to ARM_AMP");
      }
      else{
        state = ARM_RETRACT_INITIAL;
        frc::SmartDashboard::PutString("state: ", "changing to ARM_RETRACT_INITIAL");
      }
      time = 0;
    }


    break;

  case ARM_TRAP:
    frc::SmartDashboard::PutString("state: ", "ARM_TRAP");

    m_arm->setLowerArmAngle(ArmConstants::LowerTrapExtentionAngle);
    m_arm->setUpperArmAngle(ArmConstants::UpperTrapExtentionAngle);

    //switch states when timer has exceded 1.5 seconds
    //run 60 times a second
    time++;

    if(time >= 60){
      state = DROP;
      frc::SmartDashboard::PutString("state: ", "changing to DROP");
      time = 0;
    }


    break;

  case ARM_AMP:
    frc::SmartDashboard::PutString("state: ", "ARM_AMP");

    m_arm->setLowerArmAngle(ArmConstants::LowerAmpExtentionAngle);
    m_arm->setUpperArmAngle(ArmConstants::UpperAmpExtentionAngle);

    //switch states when timer has exceded 1.5 seconds
    //run 60 times a second
    time++;

    if(time >= 60){
      state = DROP;
      frc::SmartDashboard::PutString("state: ", "changing to DROP");
      time = 0;
    }


    break;

  case DROP:
    frc::SmartDashboard::PutString("state: ", "DROP");

    m_arm->dropNote();
    //switch states when timer has exceded 1.5 seconds
    //run 60 times a second
    time++;

    if(time >= 120){
      state = ARM_RETRACT_INITIAL;
      frc::SmartDashboard::PutString("state: ", "changing to ARM_RETRACT_INITIAL");
      time = 0;
    }


    break;

  case ARM_RETRACT_INITIAL:
    frc::SmartDashboard::PutString("state: ", "ARM_RETRACT_INITAL");

    m_arm->setLowerArmAngle(ArmConstants::LowerFirstRetractionAngle);
    m_arm->setUpperArmAngle(ArmConstants::UpperFirstRetractionAngle);
    //switch states when timer has exceded 1.5 seconds
    //run 60 times a second
    time++;

    if(time >= 60){
      state = ARM_RETRACT_FINAL;
      frc::SmartDashboard::PutString("state: ", "changing to ARM_RETRACT_FINAL");
      time = 0;
    }


    break;

  case ARM_RETRACT_FINAL:
    frc::SmartDashboard::PutString("state: ", "ARM_RETRACT_FINAL");

    m_arm->setLowerArmAngle(ArmConstants::LowerFullRetractedAngle);
    m_arm->setUpperArmAngle(ArmConstants::UpperFullRetractedAngle);
    //switch states when timer has exceded 1.5 seconds
    //run 60 times a second
    time++;

    if(time >= 60){
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

    if(time >= 90){
      state = EMPTY;
      frc::SmartDashboard::PutString("state: ", "changing to EMPTY");
      time = 0;
    }

    break;

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
