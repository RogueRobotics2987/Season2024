// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/StateMachine.h"

StateMachine::StateMachine() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void StateMachine::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void StateMachine::Execute() {  
  frc::SmartDashboard::PutBoolean("orangeCheerio: ", orangeCheerio);

  switch (state) {
  case EMPTY:     // turn everything off
    frc::SmartDashboard::PutString("state: ", "EMPTY");

    // stop all motors

    if(orangeCheerio == true){
        state = PICKUP;   
        frc::SmartDashboard::PutString("state: ", "changing to PICKUP");
    } 

    break;

  case PICKUP:    // start intake and magazine
    frc::SmartDashboard::PutString("state: ", "PICKUP");
    
    // start intake motors, REMEMBER: middle motor changes direction

    if(m_colorSensor.detectiveOrange1 == true /*|| detectiveOrange2 == true*/){
      // start magazine motor

      frc::SmartDashboard::PutBoolean("detect cheerio?: ", m_colorSensor.detectiveOrange1);

    }

    /*if(orangeCheerio == false){
        state = EMPTY;

    } else if(eatenCheerio == true){
        state = LOADED;
    }*/

    break;

  case LOADED:    // self explanitory
    frc::SmartDashboard::PutString("state: ", "LOADED");

    // turn running motors off


    orangeCheerio = false;


    if(warmMilk == true){
        state = SHOOTER_WARMUP;

    } else if(spillMilk == true){
        state = DROP_WARMUP;
        
    }

    break;

  case SHOOTER_WARMUP:
    frc::SmartDashboard::PutString("state: ", "SHOOTER_WARMUP");

    break;

  case SHOOT:
    frc::SmartDashboard::PutString("state: ", "SHOOT");

    break;
  
  case DROP_WARMUP:
    frc::SmartDashboard::PutString("state: ", "DROP_WARMUP");

    break;
  
  case DROP:
    frc::SmartDashboard::PutString("state: ", "DROP");

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
