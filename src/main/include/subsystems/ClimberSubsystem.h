// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include <frc/Servo.h>
#include <frc/smartdashboard/SmartDashboard.h>

class ClimberSubsystem : public frc2::SubsystemBase 
{
  public:
    ClimberSubsystem();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;
    void StartClimber();
    void StopClimber();

  private:
    rev::CANSparkMax m_climberMoter{12, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkRelativeEncoder* m_driveEncoder;
    rev::SparkRelativeEncoder::Type m_EncoderType;

    int m_counts_per_rev;
    bool m_reverseClimberEncoder;
};
