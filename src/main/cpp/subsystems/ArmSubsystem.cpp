// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

ArmSubsystem::ArmSubsystem() {}

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {

}


void ArmSubsystem::setLowerArmAngle(double desiredAngle){
    double currAngle = m_LowerArmEncoder->GetPosition();    //TODO: double check if it gets angle

    double error = desiredAngle - currAngle;
    kiSumLowerArm = kiSumLowerArm + (kiLowerArm * error);

    double motorOutput = error * kpLowerArm + kiSumLowerArm;

    LowerArm.Set(motorOutput);
}

void ArmSubsystem::setUpperArmAngle(double desiredAngle){
    double currAngle = m_UpperArmEncoder->GetPosition();    //same as above 

    double error = desiredAngle - currAngle;
    kiSumUpperArm = kiSumUpperArm + (kiUpperArm * error);

    double motorOutput = error * kpUpperArm + kiSumUpperArm;

    UpperArm.Set(motorOutput);
}


