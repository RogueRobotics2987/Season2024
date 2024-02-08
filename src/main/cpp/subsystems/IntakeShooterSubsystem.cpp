// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeShooterSubsystem.h"

IntakeShooterSubsystem::IntakeShooterSubsystem() = default;

// This method will be called once per scheduler run
void IntakeShooterSubsystem::Periodic() {
    detectiveOrange1 = colorInput.Get();

    frc::SmartDashboard::PutBoolean("color sensor val: ", detectiveOrange1);
    
    frc::SmartDashboard::PutBoolean("orangeCheerio: ", orangeCheerio);

    switch (state) {
    case EMPTY:     // turn everything off
        frc::SmartDashboard::PutString("state: ", "EMPTY");


        if(orangeCheerio == true){
            state = PICKUP;   
            frc::SmartDashboard::PutString("state: ", "changing to PICKUP");
        } 

        break;
    
    case PICKUP:    // start intake and magazine
        frc::SmartDashboard::PutString("state: ", "PICKUP");
        

        if(detectiveOrange1 == true){

            frc::SmartDashboard::PutBoolean("detect cheerio?: ", detectiveOrange1);
        }

        /*if(orangeCheerio == false){
            state = EMPTY;

        } else if(eatenCheerio == true){
            state = LOADED;
        }*/

        break;

    case LOADED:    // self explanitory
        frc::SmartDashboard::PutString("state: ", "LOADED");

        orangeCheerio = false;


        if(warmMilk == true){
            state = SHOOTER_WARMUP;

        } else if(spillMilk == true){
            state = DROP_WARMUP;
            
        }

        break;

    case SHOOTER_WARMUP:

        break;

    case SHOOT:

        break;
    
    case DROP_WARMUP:

        break;
    
    case DROP:

        break;
    
    default:
        state = EMPTY;
        break;
    }

}

frc2::CommandPtr IntakeShooterSubsystem::Pickup(){
    return this->RunOnce( [this] {
      orangeCheerio = true;

    });
}
frc2::CommandPtr IntakeShooterSubsystem::PickupStop(){
    return this->RunOnce( [this] {
      orangeCheerio = false;
    });
}
