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

    if(m_UpperDesired > ArmConstants::UpperArmSoftLimitHigh){
        m_UpperDesired = ArmConstants::UpperArmSoftLimitHigh;
    }
    else if(m_UpperDesired < ArmConstants::UpperArmSoftLimitLow){
        m_UpperDesired = ArmConstants::UpperArmSoftLimitLow;
    }

    if(m_LowerDesired > ArmConstants::LowerArmSoftLimitHigh){
        m_LowerDesired = ArmConstants::LowerArmSoftLimitHigh;
    }
    else if(m_LowerDesired < ArmConstants::LowerArmSoftLimitLow){ 
        m_LowerDesired = ArmConstants::LowerArmSoftLimitLow;
    }

    frc::SmartDashboard::PutNumber("lower desired: limit", m_LowerDesired);

    if(GetOffSetEncoderValueLower() < 1.0 && GetOffSetEncoderValueLower() > -1.0){
        LowerArmAngle = 0;
    }
    else{
        LowerArmAngle = GetOffSetEncoderValueLower();
    }
    if(GetOffSetEncoderValueUpper() < 1.0 && GetOffSetEncoderValueUpper() > -1.0){
        UpperArmAngle = 0;
    }
    else{
        UpperArmAngle = GetOffSetEncoderValueUpper();
    }


    LowerArm.Set((DistanceBetweenAngles(m_LowerDesired, LowerArmAngle) * ArmConstants::kpLowerArm) * -1);   // questionably tested
    UpperArm.Set((DistanceBetweenAngles(m_UpperDesired, UpperArmAngle) * ArmConstants::kpUpperArm) * -1); 
}


void ArmSubsystem::setLowerArmAngle(double desiredAngle){
    //frc::SmartDashboard::PutString("state function: ", "setLowerArmAngle");

    m_LowerDesired = desiredAngle;

    // double currAngle = m_LowerArmEncoder.GetAbsolutePosition();    //TODO: double check if it gets angle

    // double error = desiredAngle - currAngle; 
    // kiSumLowerArm = kiSumLowerArm + (kiLowerArm * error);

    // double motorOutput = error * kpLowerArm + kiSumLowerArm;

    // LowerArm.Set(motorOutput);
}

void ArmSubsystem::setUpperArmAngle(double desiredAngle){
    //frc::SmartDashboard::PutString("state function: ", "setUpperArmAngle");

    m_UpperDesired = desiredAngle;

    // double currAngle = m_UpperArmEncoder.GetAbsolutePosition();    //same as above 

    // double error = desiredAngle - currAngle;
    // kiSumUpperArm = kiSumUpperArm + (kiUpperArm * error);

    // double motorOutput = error * kpUpperArm + kiSumUpperArm;

    // UpperArm.Set(motorOutput);
}

void ArmSubsystem::setVoltage(double speed){ 
    LowerArm.Set(speed);
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


double ArmSubsystem::GetOffSetEncoderValueLower()
{
    double Pose = 0;
    Pose = m_LowerArmEncoder.GetAbsolutePosition() - ArmConstants::LowerArmOffset;

    // if(Pose < 0){
        // Pose += 1;
    // }

    // Pose = fabs(Pose - 1);  //This is the invert
    Pose *= 360;

    return Pose;
}

double ArmSubsystem::GetOffSetEncoderValueUpper()
{
    double Pose = 0;
    Pose = m_UpperArmEncoder.GetAbsolutePosition() - ArmConstants::UpperArmOffset;

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

// void ArmSubsystem::StopWheels()
// {
//     ArmWheels.Set(0.0);
// }

// void ArmSubsystem::dropNote(){      //TODO: motor direction based on arm pos(?)
//     //frc::SmartDashboard::PutString("state function: ", "dropNote");

//     //double lowArmAngle = m_LowerArmEncoder.GetPosition();

//     //ArmWheels.Set(0.5);
// }

// void ArmSubsystem::stopDrop(){      //stop armWheels
//     //frc::SmartDashboard::PutString("state function: ", "stopDrop");
//     //ArmWheels.Set(0.0);
// }