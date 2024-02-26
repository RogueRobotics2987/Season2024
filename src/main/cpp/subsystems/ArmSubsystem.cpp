// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

ArmSubsystem::ArmSubsystem()
{
    ArmWheels.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    LowerArm.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 500);
    UpperArm.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 500);
    LowerArm.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus3, 500);
    UpperArm.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus3, 500);
    LowerArm.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus4, 500);
    UpperArm.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus4, 500);
    LowerArm.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus5, 500);
    UpperArm.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus5, 500); 
    LowerArm.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus6, 500);
    UpperArm.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus6, 500);
}

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {
    frc::SmartDashboard::PutNumber("LowerArmEncoderValue", m_LowerArmEncoder.GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("UpperArmEncoderValue", m_UpperArmEncoder.GetAbsolutePosition()); 
    frc::SmartDashboard::PutNumber("LowerArmEncoderValueOffset", GetOffSetEncoderValueLower());
    frc::SmartDashboard::PutNumber("UpperArmEncoderValueOffset", GetOffSetEncoderValueUpper());

    frc::SmartDashboard::PutNumber("lower arm desired", m_LowerDesired);
    //frc::SmartDashboard::PutNumber("lower desired: limit", m_LowerDesired);

    frc::SmartDashboard::PutNumber("upper arm desired", m_UpperDesired);
   // frc::SmartDashboard::PutNumber("upper desired: limit", m_UpperDesired);

    setLowerArmAngle(m_LowerDesired);
    setUpperArmAngle(m_UpperDesired);
}
/* 
void ArmSubsystem::setVoltage(double speed){ 
    LowerArm.Set(speed);
}
 */

void ArmSubsystem::dropNote(){      //TODO: motor direction based on arm pos(?)
    //frc::SmartDashboard::PutString("state function: ", "dropNote");

    //double lowArmAngle = m_LowerArmEncoder.GetPosition();

    //ArmWheels.Set(0.5);
}

void ArmSubsystem::stopDrop(){      //stop armWheels
    //frc::SmartDashboard::PutString("state function: ", "stopDrop");
    //ArmWheels.Set(0.0);
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
    Pose = m_LowerArmEncoder.GetAbsolutePosition() ; //Offset used to reference a desired zero position with raw encoder value

    // if(Pose < 0){
        // Pose += 1;
    // }

    Pose = fabs(Pose - 1);  //This is the invert
    Pose *= 360;

    return Pose + ArmConstants::LowerArmOffset;
}

double ArmSubsystem::GetOffSetEncoderValueUpper()
{
    double Pose = 0;
    Pose = m_UpperArmEncoder.GetAbsolutePosition() - ArmConstants::UpperArmOffset; //Offset used to reference a desired zero position with raw encoder value

    // if(Pose < 0){
        // Pose += 1;
    // }

    // Pose = fabs(Pose - 1); //This is the invert
    Pose *= 272.72;

    return Pose;
}

double ArmSubsystem::DistanceBetweenAngles(double targetAngle, double sourceAngle)
{
  double a = targetAngle - sourceAngle;
  if(a > 180)
  {
    a = a + -360;
  }
  else if(a < -180)
  {
    a = a + 360;
  }
  else
  {
    a = a;
  }

  return a;
}
void ArmSubsystem::FollowShooter(double error){
    ArmWheels.Set(error * 0.035);
}

void ArmSubsystem::MoveLowerArm(){
    LowerArm.Set(0.5);
}

void ArmSubsystem::setLowerArmAngle(double desiredAngle)
{
    if(desiredAngle >= ArmConstants::LowerArmSoftLimitHigh)
    {
        m_LowerDesired = ArmConstants::LowerArmSoftLimitHigh;
    }
    else if(desiredAngle <= ArmConstants::LowerArmSoftLimitLow)
    {
        m_LowerDesired = ArmConstants::LowerArmSoftLimitLow;
    }
    else
    {
        m_LowerDesired = desiredAngle;
    }


    double LowerangleError = DistanceBetweenAngles(m_LowerDesired, GetOffSetEncoderValueLower());
    if(LowerangleError < 10)
    {
        LoweraccumulatedError += ArmConstants::kiLowerArm * LowerangleError;
    }    
    else {
        LoweraccumulatedError = 0;
    }

    double lowerAngleOutput = ((LowerangleError * ArmConstants::kpLowerArm)) + LoweraccumulatedError;

    LowerArm.Set(lowerAngleOutput); //TODO verify polarity

}

void ArmSubsystem::setUpperArmAngle(double desiredAngle)
{
    if(desiredAngle >= ArmConstants::UpperArmSoftLimitHigh)
    {
        m_UpperDesired = ArmConstants::UpperArmSoftLimitHigh;
    }
    else if(desiredAngle <= ArmConstants::UpperArmSoftLimitLow)
    {
        m_UpperDesired = ArmConstants::UpperArmSoftLimitLow;
    }
   else
    {
        m_UpperDesired = desiredAngle;
    }


    double UpperangleError = DistanceBetweenAngles(m_UpperDesired, GetOffSetEncoderValueUpper());
    if(UpperangleError < 5)
    {
        UpperaccumulatedError += ArmConstants::kiUpperArm * UpperangleError;
    }
    else {
        UpperaccumulatedError = 0;
    }

    double UpperAngleOutput = ((UpperangleError * ArmConstants::kpUpperArm)) + UpperaccumulatedError;

    UpperArm.Set(UpperAngleOutput); //TODO verify polarity

}
