// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "networktables/NetworkTableInstance.inc"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/LimelightSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AprilTagFollower
    : public frc2::CommandHelper<frc2::Command, AprilTagFollower> 
{
  public:
    AprilTagFollower(); 
    AprilTagFollower(LimelightSubsystem &limelight, DriveSubsystem &drivetrain, frc::XboxController &Xbox, ShooterSubsystem &shooter);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

    double kp = -0.09927;
    double speedX = 0;
    double speedY = 0;
    bool NoJoystickInput = false;

  private:
    LimelightSubsystem* m_limelight = nullptr;
    DriveSubsystem* m_drivetrain = nullptr;
    ShooterSubsystem* m_shooter = nullptr;
    frc::XboxController* m_Xbox = nullptr;
    float Deadzone(float x);

};