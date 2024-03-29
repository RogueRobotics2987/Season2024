// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

ArmSubsystem::ArmSubsystem()
{
    ArmWheels.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    LowerArm.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 500);
    LowerArm.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus3, 500);
    LowerArm.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus4, 500);
    LowerArm.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus5, 500);
    LowerArm.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus6, 500);
    sleep(1);

}

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() 
{
    frc::SmartDashboard::PutNumber("LowerArmEncoderValue", m_LowerArmEncoder.GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("LowerArmEncoderValueOffset", GetOffSetEncoderValueLower());
    frc::SmartDashboard::PutNumber("lower arm desired", m_LowerDesired);

    RunLowerArm();
}

double ArmSubsystem::GetLowerEncoderPos()
{
    return m_LowerArmEncoder.GetAbsolutePosition();
}

void ArmSubsystem::RunArmWheels(double speed)
{
   ArmWheels.Set(-speed);
}
void ArmSubsystem::StopArmWheels()
{
    ArmWheels.Set(0.0);
}

double ArmSubsystem::GetOffSetEncoderValueLower()
{
    double Pose = 0;
    Pose = m_LowerArmEncoder.GetAbsolutePosition() ; //Offset used to reference a desired zero position with raw encoder value

    Pose = fabs(Pose - 1);  //This is the invert
    Pose *= 360; //360 for per revolution

    return Pose + ArmConstants::LowerArmOffset;
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

void ArmSubsystem::FollowShooter(double error)
{
    ArmWheels.Set(error * 0.035);
}

void ArmSubsystem::MoveLowerArm()
{
    LowerArm.Set(0.5);
}

double ArmSubsystem::GetLowerArmError()
{
   return m_LowerDesired-GetOffSetEncoderValueLower();
}

void ArmSubsystem::SetLowerArmAngle(double desiredAngle)
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
}

// void ArmSubsystem::AccumulateErrorLower()
// {
//     double LowerangleError = DistanceBetweenAngles(m_LowerDesired, GetOffSetEncoderValueLower());
//     if(LowerangleError < 10)
//     {
//         LoweraccumulatedError += ArmConstants::kiLowerArm * LowerangleError;
//     }    
//     else
//     {
//         LoweraccumulatedError = 0;
//     }
// }

void ArmSubsystem::RunLowerArm()
{
    double LowerangleError = DistanceBetweenAngles(m_LowerDesired, GetOffSetEncoderValueLower());
    double lowerAngleOutput = ((LowerangleError * ArmConstants::kpLowerArm));
    LowerArm.Set(lowerAngleOutput); //TODO verify polarity
}

void ArmSubsystem::ZeroIntergral()
{
    LoweraccumulatedError = 0;
}