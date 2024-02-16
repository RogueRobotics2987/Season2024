// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem(){}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic()
{
    if(DebugConstants::debug == true){
        frc::SmartDashboard::PutBoolean("ColorFront", intakeColorSensorFront.Get());
        frc::SmartDashboard::PutBoolean("ColorBack", intakeColorSensorRear.Get());
    }
}


void IntakeSubsystem::Direction(){  // get current val to tell which direction motors moving in
    // frontVal= FrontIntake.GetOutputCurrent();
    // backVal = BackIntake.GetOutputCurrent();

    if (intakeColorSensorFront.Get()){ // need to find actual value and put in constants 
        CenterIntake.Set(-0.25); // need to find actual speed

    }else if (intakeColorSensorRear.Get()){
        CenterIntake.Set(0.25);
    }
}

void IntakeSubsystem::runIntake(){
    // need to make it so that these turn on with a button
    FrontIntake.Set(0.25);
    BackIntake.Set(-0.25);
    ArmMotor.Set(0.25);
}

void IntakeSubsystem::stopIntake(){
    FrontIntake.Set(0.0);
    BackIntake.Set(0.0);
    CenterIntake.Set(0.0);
}

bool IntakeSubsystem::GetIntakeFront(){
    return intakeColorSensorFront.Get();
}

bool IntakeSubsystem::GetIntakeRear(){
    return intakeColorSensorRear.Get();
}
