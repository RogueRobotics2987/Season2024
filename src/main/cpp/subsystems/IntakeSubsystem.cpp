// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem()
{
    magPIDController.SetP(ShooterConstants::magKp);
    middleRollers.Follow(MagazineMotor, false); //possibly change

    CenterIntake.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 500);
    FrontIntake.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 500);
    BackIntake.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 500);
    sleep(0.1);

    CenterIntake.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus3, 500);
    FrontIntake.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus3, 500);
    BackIntake.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus3, 500);
    sleep(0.1);

    CenterIntake.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus4, 500);
    FrontIntake.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus4, 500);
    BackIntake.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus4, 500);
    sleep(0.1);

    CenterIntake.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus5, 500);
    FrontIntake.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus5, 500);
    BackIntake.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus5, 500);
    sleep(0.1);

    CenterIntake.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus6, 500);
    FrontIntake.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus6, 500);
    BackIntake.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus6, 500);

    sleep(1);

}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic()
{
    if(DebugConstants::debugIntake == true)
    {
        frc::SmartDashboard::PutBoolean("ColorFront", intakeColorSensorFront.Get());
    }

    frc::SmartDashboard::PutNumber("BackIntakeCurrent", GetBackIntakeCurrent());
}

void IntakeSubsystem::Direction(double speed)
{  // get current val to tell which direction motors moving in
    if (intakeColorSensorFront.Get())
    { // need to find actual value and put in constants 
        CenterIntake.Set(-speed); // need to find actual speed
    }
}

void IntakeSubsystem::DirectionNote(double speed)
{
    if(intakeColorSensorFront.Get())
    {
        CenterIntake.Set(-speed);
    }
    else
    {
        CenterIntake.Set(speed);
    }
}

void IntakeSubsystem::RunIntake(double speed)
{
    // need to make it so that these turn on with a button
    FrontIntake.Set(speed);
    BackIntake.Set(-speed);
}

void IntakeSubsystem::StopIntake()
{
    FrontIntake.Set(0.0);
    BackIntake.Set(0.0);
    CenterIntake.Set(0.0);
}

bool IntakeSubsystem::GetIntakeFront()
{
    return intakeColorSensorFront.Get();
}

void IntakeSubsystem::SpitOutIntake()
{
    FrontIntake.Set(-0.2);
    BackIntake.Set(0.2);
    CenterIntake.Set(0.2);
    MagazineMotor.Set(-0.2);
}

bool IntakeSubsystem::GetMagazineSensor()
{
    return MagazineSensor.Get();
}

void IntakeSubsystem::RunMagazine(double speed)
{
    MagazineMotor.Set(speed);
}

void IntakeSubsystem::StopMagazine()
{
    MagazineMotor.Set(0.0);
}

void IntakeSubsystem::HoldMagazine(double pos)
{
    magPIDController.SetReference(pos, rev::ControlType::kPosition);
}

double IntakeSubsystem::GetCurrMagEncoderVal()
{
    double currEncoderVal = MagazineEncoder.GetPosition();
    return currEncoderVal;
}

double IntakeSubsystem::GetBackIntakeCurrent()
{
    return BackIntake.GetOutputCurrent();
}