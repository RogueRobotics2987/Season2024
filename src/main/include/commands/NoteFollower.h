// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <iostream>

#include "networktables/NetworkTableInstance.inc"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/LimelightSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LightSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class NoteFollower : public frc2::CommandHelper<frc2::Command, NoteFollower>
{
  public:
    NoteFollower();
    NoteFollower(
      LimelightSubsystem &limelight,
      DriveSubsystem &drivetrain,
      frc::XboxController &driverController,
      IntakeSubsystem &intake,
      LightSubsystem &lights,
      frc::XboxController &m_auxController);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

    float Deadzone(float x);

  private:
    LimelightSubsystem* m_limelight = nullptr;
    DriveSubsystem* m_drivetrain = nullptr;
    IntakeSubsystem* m_intake = nullptr;
    LightSubsystem* m_lights = nullptr; 
    frc::XboxController* m_driverController = nullptr;
    frc::XboxController* m_auxController = nullptr;

    units::angular_velocity::radians_per_second_t rotNote = units::angular_velocity::radians_per_second_t(0);
    double kpNote = 0.04;
    double txNote = 0.0;
    bool NoJoystickInput = false;
    double speedY = 0;  
    double speedX = 0;
    double rot = 0;
    double noteError = 0;
    double tyNote = 0;

    int state = 0;
    int time = 0;
    bool finished = false;

    //if the robot has normal driving be false if it has the note tracker specific driving true
    bool noteDrive = false;

    //if the robot is currently intaking a note
    bool hasNote = false;
};