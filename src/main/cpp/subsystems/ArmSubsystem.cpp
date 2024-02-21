// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

ArmSubsystem::ArmSubsystem()
{}

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {
    // switch (state) {
        // case INITAL:
        //     setLowerArmAngle(ArmConstants::LowerInitalAngle);
        //     setUpperArmAngle(ArmConstants::UpperInitalAngle);
        //     // if(m_Xbox->getr)
        //     state = LOWER_ARM_EXTEND_INITAL;

        //     break;
        
        // case LOWER_ARM_EXTEND_INITAL:
        //     setLowerArmAngle(ArmConstants::LowerFirstExtentionAngle);
        //     setUpperArmAngle(ArmConstants::UpperFirstExtentionAngle);
        //     HasNote = true;

        //     break;

        // case UPPER_ARM_EXTEND_INITAL:
        //     setLowerArmAngle(ArmConstants::LowerExtentionAngle);
        //     setUpperArmAngle(ArmConstants::UpperExtentionAngle);

        //     break;

        // case ARM_FINAL:
        //     setLowerArmAngle(ArmConstants::LowerFinalExtentionAngle);
        //     setUpperArmAngle(ArmConstants::UpperFinalExtentionAngle);

        //     break;

        // case DROP:
        //     setLowerArmAngle(ArmConstants::LowerDropAngle);
        //     setUpperArmAngle(ArmConstants::UpperDropAngle);
        //     dropNote();
        //     HasNote = false;

        //     break;

        // case ARM_RETRACT_INITAL:
        //     setLowerArmAngle(ArmConstants::LowerFirstRetractionAngle);
        //     setUpperArmAngle(ArmConstants::UpperFirstRetractionAngle);

        //     break;

        // case ARM_RETRACT_FINAL:
        //     setLowerArmAngle(ArmConstants::LowerFullRetractedAngle);
        //     setUpperArmAngle(ArmConstants::UpperFinalExtentionAngle);

        //     break;

    //     default:
    //     state = INITAL;
    //     break;


    // }
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

    double currAngle = m_UpperArmEncoder.GetPosition();    //same as above 

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
    return m_UpperArmEncoder.GetPosition();
}

void ArmSubsystem::runArmWheels(double speed){
   ArmWheels.Set(-speed);
}
void ArmSubsystem::stopArmWheels(){
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

void ArmSubsystem::StopWheels(){
    ArmWheels.Set(0.0);
}