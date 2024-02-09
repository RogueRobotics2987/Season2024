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

    if(orangeCheerio == false){
        state = EMPTY;

    } else if(m_colorSensor.eatenCheerio == true){
        state = LOADED;
    }

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

    //start shooter motors

    if(warmMilk == false ){
      state = LOADED;
      
    }else if(spoon == true){
      state = SHOOT;
    }

    break;

  case SHOOT:
    frc::SmartDashboard::PutString("state: ", "SHOOT");

    //turn on the mag motors 
    //keep shooter motors turned on
    //swich states when timer have exceded 1.5 seconds
    // runs 60 times a second  so we want 90 times

    time++;

    if(time>90){
      state = EMPTY;
      time = 0;
    }

    break;
  
  case DROP_WARMUP:
    frc::SmartDashboard::PutString("state: ", "DROP_WARMUP");

    //lift shooter
    /* turn on the bottom mottor first
       turn on upper one after it reaches safe point
       17 is bottom motor 
       18 is joint motor 
       19 is drop motor
    */

    //extend the arm

    if(micdrop == true){
      state = DROP;
    }


    break;
  
  case DROP:
    frc::SmartDashboard::PutString("state: ", "DROP");

    //drop note, start upper arm motors

    //timer, then switch state to EMPTY

    timeDrop++;

    if(timeDrop>90){
      state = EMPTY;
      timeDrop = 0;
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
