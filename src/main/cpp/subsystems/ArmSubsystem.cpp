// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

ArmSubsystem::ArmSubsystem()
{}

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {
    frc::SmartDashboard::PutNumber("LowerArmEncoderValue", m_LowerArmEncoder.GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("UpperArmEncoderValue", m_UpperArmEncoder.GetAbsolutePosition()); 
    frc::SmartDashboard::PutNumber("LowerArmEncoderValueOffset", GetOffSetEncoderValueLower());
    frc::SmartDashboard::PutNumber("UpperArmEncoderValueOffset", GetOffSetEncoderValueUpper());
}


void ArmSubsystem::setLowerArmAngle(double desiredAngle){
    //frc::SmartDashboard::PutString("state function: ", "setLowerArmAngle");

    double currAngle = m_LowerArmEncoder.GetAbsolutePosition();    //TODO: double check if it gets angle

    double error = desiredAngle - currAngle;
    kiSumLowerArm = kiSumLowerArm + (kiLowerArm * error);

    double motorOutput = error * kpLowerArm + kiSumLowerArm;

    LowerArm.Set(motorOutput);
}

void ArmSubsystem::setUpperArmAngle(double desiredAngle){
    //frc::SmartDashboard::PutString("state function: ", "setUpperArmAngle");

    double currAngle = m_UpperArmEncoder.GetAbsolutePosition();    //same as above 

    double error = desiredAngle - currAngle;
    kiSumUpperArm = kiSumUpperArm + (kiUpperArm * error);

    double motorOutput = error * kpUpperArm + kiSumUpperArm;

    UpperArm.Set(motorOutput);
}

void ArmSubsystem::setSpeed(double speed){
    LowerArm.Set(speed);
}

void ArmSubsystem::dropNote(){      //TODO: motor direction based on arm pos(?)
    //frc::SmartDashboard::PutString("state function: ", "dropNote");

    //double lowArmAngle = m_LowerArmEncoder.GetPosition();

    //ArmWheels.Set(0.5);
}
void ArmSubsystem::stopDrop(){      //stop armWheels
    //frc::SmartDashboard::PutString("state function: ", "stopDrop");

}

double ArmSubsystem::getLowerEncoderPos(){
    return m_LowerArmEncoder.GetAbsolutePosition();
}

double ArmSubsystem::getUpperEncoderPos(){
    return m_UpperArmEncoder.GetAbsolutePosition();
}

void ArmSubsystem::runArmWheels(double speed){
   ArmWheels.Set(-speed);
}
void ArmSubsystem::stopArmWheels()
{
    ArmWheels.Set(0.0);
}
// bool ArmSubsystem::compareHasNote(bool Other){
//     if(Other && HasNote) {
//         return true;
//     }
//     else{
//         return false;
//     }
// }

void ArmSubsystem::StopWheels()
{
    ArmWheels.Set(0.0);
}

double ArmSubsystem::GetOffSetEncoderValueLower()
{
    double Pose = 0;
    Pose = m_LowerArmEncoder.GetAbsolutePosition() - 0.20;

    if(Pose < 0){
        Pose += 1;
    }

    Pose = fabs(Pose - 1);
    Pose *= 132;

    return Pose;
}

double ArmSubsystem::GetOffSetEncoderValueUpper()
{
    double Pose = 0;
    Pose = m_UpperArmEncoder.GetAbsolutePosition() + .48;

    if(Pose < 0){
        Pose += 1;
    }

    Pose = fabs(Pose - 1);
    Pose *= 132;

    return Pose;
}