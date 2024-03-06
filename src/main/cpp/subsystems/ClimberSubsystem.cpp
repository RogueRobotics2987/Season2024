// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"

ClimberSubsystem::ClimberSubsystem() {
    m_climberMoter.SetOpenLoopRampRate(0.5);
}
//TODO: Not sure what this does - commented out for now.
  /*int m_MotorController,
  rev::SparkRelativeEncoder::Type m_EncoderType,
  int m_counts_per_rev
)

// {

}
*/

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}

void ClimberSubsystem::startClimber() 
{
    m_climberMoter.Set(0.5);  
}

void ClimberSubsystem::stopClimber() 
{
    m_climberMoter.Set(0.0);
}
