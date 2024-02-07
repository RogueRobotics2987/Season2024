// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ColorSensorSubsystem.h"

ColorSensorSubsystem::ColorSensorSubsystem() = default;

// This method will be called once per scheduler run
void ColorSensorSubsystem::Periodic() {
    detectNoteIntake1 = colorInput.Get();
    eatenNote = colorInput2.Get();

    frc::SmartDashboard::PutBoolean("Intake colorSensor val: ", detectNoteIntake1);
    frc::SmartDashboard::PutBoolean("Mag colorSensor val: ", eatenNote);

}
