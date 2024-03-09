// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>

class ColorSensorSubsystem : public frc2::SubsystemBase
{
  public:
    ColorSensorSubsystem();

    bool detectNoteIntake1 = false;   // color sensor on the __ of robot
    bool detectNoteIntake2 = false;   // color sensor on the __ of robot
    bool eatenNote = false;      // color sensor between mag and shooter

    /**
    * Will be called periodically whenever the CommandScheduler runs.
    */
    void Periodic() override;

  private:
};
