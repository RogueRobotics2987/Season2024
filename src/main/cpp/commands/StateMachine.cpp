// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/StateMachine.h"

StateMachine::StateMachine() {
  // Use addRequirements() here to declare subsystem dependencies.
}
StateMachine::StateMachine(ArmSubsystem &arm, ClimberSubsystem &climb, ColorSensorSubsystem &color, IntakeSubsystem &intake, ShooterSubsystem &shooter){
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
}

// Called when the command is initially scheduled.
void StateMachine::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void StateMachine::Execute() {  
  frc::SmartDashboard::PutBoolean("Pick up note?: ", pickupNote);

  switch (state) {
  case EMPTY:     // turn everything off
    frc::SmartDashboard::PutString("state: ", "EMPTY");

    // stop all motors

    if(pickupNote == true){
      state = PICKUP;   
      frc::SmartDashboard::PutString("state: ", "changing to PICKUP");
    } 

    break;

  case PICKUP:    // start intake and magazine
    frc::SmartDashboard::PutString("state: ", "PICKUP");
    
    // start intake motors, REMEMBER: middle motor changes direction

    if(m_colorSensor->detectNoteIntake1 == true /*|| detectNoteIntake2 == true*/){
      // start magazine motor

      frc::SmartDashboard::PutBoolean("detect note?: ", m_colorSensor->detectNoteIntake1);

    }

    if(pickupNote == false){
      state = EMPTY;

    } else if(m_colorSensor->eatenNote == true){
      state = LOADED;
    }

    break;

  case LOADED:    // self explanitory
    frc::SmartDashboard::PutString("state: ", "LOADED");
    //pickupNote = false;

    // turn running motors off

    if(warmUpShooter == true){
      state = SHOOTER_WARMUP;

    } else if(moveArm2Drop == true){
      state = DROP_WARMUP;
        
    }

    break;

  case SHOOTER_WARMUP:
    frc::SmartDashboard::PutString("state: ", "SHOOTER_WARMUP");

    //start shooter motors

    if(warmUpShooter == false){
      state = LOADED;

    } else if(moveNote2Shoot == true){
      state = SHOOT;
    }

    break;

  case SHOOT:
    frc::SmartDashboard::PutString("state: ", "SHOOT");
    warmUpShooter = false;

    //turn on mag motors

    //switch states when timer has exceded 1.5 seconds
    //run 60 times a second
    time++;

    if(time >= 90){
      state = EMPTY;

      time = 0;
      moveNote2Shoot = false;
    }

    break;
  
  case DROP_WARMUP:
    frc::SmartDashboard::PutString("state: ", "DROP_WARMUP");

    //lift shooter and extend arm
    if(m_arm->getLowerEncoderPos() < 25 /*&&/|| shooter is at okay angle*/){   //double check algorithm
      //lift shooter

      m_arm->setLowerArmAngle(90);

      m_arm->setUpperArmAngle(45);
    }
    /*turn on lower motor first
      turn on upper after it reaches safe point (90 degrees)
      17 = bottom; 18 = joint; 19 = drop???
    */

    if(moveArm2Drop == false){
      state = LOADED;

    } else if(dropNote == true){
      state = DROP;
    }

    break;
  
  case DROP:
    frc::SmartDashboard::PutString("state: ", "DROP");
    moveArm2Drop = false;

    //drop note
    m_arm->dropNote();

    timeDrop++;

    if(timeDrop >= 90){
      state = EMPTY;

      timeDrop = 0;
      dropNote = false;
    }

    break;
  
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
