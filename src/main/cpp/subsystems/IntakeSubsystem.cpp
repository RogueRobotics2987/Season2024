// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem(){

}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic(){

}


void IntakeSubsystem::Direction(){  // get current val to tell which direction motors moving in
    frontVal= FrontIntake.GetOutputCurrent();
    backVal = BackIntake.GetOutputCurrent();

    if (frontVal > 2.5){ // need to find actual value and put in constants 
        CenterIntake. Set(0.5); // need to find actual speed
        
    }else if (backVal > 2.5){
        CenterIntake.Set(-0.5);
    }
}

void IntakeSubsystem::runIntake(){
    // need to make it so that these turn on with a button
    FrontIntake.Set(0.5);
    BackIntake.Set(-0.5);

}

void IntakeSubsystem::stopIntake(){
    FrontIntake.Set(0.0);
    BackIntake.Set(0.0);
    CenterIntake.Set(0.0);
}

void IntakeSubsystem::runMagazine(){

}

void IntakeSubsystem::stopMagazine(){

}