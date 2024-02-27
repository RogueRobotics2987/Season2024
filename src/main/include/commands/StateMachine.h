// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "MessengerCommand.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/ColorSensorSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/LimelightSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/FunctionalCommand.h>

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h> 

#include "networktables/NetworkTableInstance.inc"

#include "photon/PhotonUtils.h"
#include "photon/PhotonCamera.h"
#include "photon/PhotonPoseEstimator.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class StateMachine
    : public frc2::CommandHelper<frc2::Command, StateMachine> {
 public:
  StateMachine();
  StateMachine(
    DriveSubsystem &drive,
    LimelightSubsystem &limelight,
    ArmSubsystem &arm,
    ClimberSubsystem &climb,
    ColorSensorSubsystem &color,
    IntakeSubsystem &intake,
    ShooterSubsystem &shooter,
    frc::XboxController &driverController,
    frc::XboxController &auxController,
    MessengerCommand &message
 );


  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  float Deadzone(float x);

  double DistanceBetweenAngles(double targetAngle, double sourceAngle);


  //TODO: WHY ARE THESE PUBLIC? REVIEW THESE AND THEY PROBABLY NEED TO GO TO PRIVATE

 private:
  enum intakeState 
  {
    EMPTY,
    SPIT_OUT,
    PICKUP,
    LOADED,
    SHOOTER_WARMUP,
    SHOOT,
    DROP_ARMS,
    DROP_SHOOTER,
    RAISE_SHOOTER,
    ARMS_EXTEND_INITIAL,
    FORWARD_ARM_AMP,
    BACKWARD_ARM_AMP,
    ARM_TRAP,
    DROP,
    ARM_RETRACT_INITIAL,
    ARM_RETRACT_FINAL,
    BACKUP,
    NOTE_HUNTING,
    NOTE_FOLLOW,
    APRIL_FOLLOW,
    CHAIN_CLIMB
  };

  intakeState state = EMPTY;
  std::vector<double> RedDistVector;
  std::vector<double> BlueDistVector;

  ArmSubsystem* m_arm = nullptr;
  ClimberSubsystem* m_climb = nullptr;
  ColorSensorSubsystem* m_colorSensor = nullptr;
  IntakeSubsystem* m_intake = nullptr;
  ShooterSubsystem* m_shooter = nullptr;
  LimelightSubsystem* m_limelight = nullptr;
  DriveSubsystem* m_drive = nullptr;
  MessengerCommand* m_messager;


  frc::XboxController* m_driverController = nullptr;
  frc::XboxController* m_auxController = nullptr;

  double speedX = 0;
  bool NoJoystickInput = false;
  bool resetLoaded = false;

  double blueDist = 0;
  double redDist = 0;
  int apriltagID = 0;

  bool pickupNote = false;        // if auto/teleop want to pickup a note
  bool chainClimb = false;
  bool raiseHook = false;
  bool raiseRobot = false;
  bool emptyIntake = false;       // self explainitory

  bool warmUpShooter = false;     // warmup shooter
  bool moveNote2Shoot = false;    // move note into shooter
  bool placeInForwardAmp = false;
  bool placeInBackwardsAmp = false;
  bool placeInTrap = false;

  bool pov0 = false;              // spit out note when button pov0 is pressed

  //bool moveArm2Drop = false;      // warmup dropper (move arm into position)
  //bool dropNote = false;          // activate dropper
  //bool waitForArm = false;        // waits for the armSubsystem/dropper state machine

  double tx = 0.0;

  int time = 0;       //keep track of shooter iterations
  int timeDrop = 0;   //keep track of dropper iterations

  double magEncoderPos = 0.0;


  units::meter_t CAMERA_HEIGHT = units::meter_t(0.635);
  units::meter_t TAREGT_HEIGHT = units::meter_t(1.5);
  units::angle::radian_t CAMERA_PITCH = units::angle::radian_t(0.44);

  std::vector<double> targetIDs;
  std::vector<photon::PhotonTrackedTarget> myTargets;
  double targetData = 0;
  photon::PhotonTrackedTarget filteredTarget;
  int filteredTargetID = -1;
  units::meter_t filteredRange = 0_m;

  double rot = 0;
  double kp = 0.02206;//0.0248175;//0.009927;
  double speedY = 0;

//TODO: MOVED OVER FROM DriveStateMachine
  units::angular_velocity::radians_per_second_t rotNote = units::angular_velocity::radians_per_second_t(0);
  units::velocity::meters_per_second_t speedNote = units::velocity::meters_per_second_t(0);
  double kpNote = 0.05;
  double txNote = 0.0;

  units::angular_velocity::radians_per_second_t rotApril = units::angular_velocity::radians_per_second_t(0);
  double kpApril = 0.09;
  double txApril = 0.0;

  bool noteFollowState = false;
  bool aprilFollowState = false;
  bool standard = false;
  bool runIntake = false;
  bool runShooterWarmup = false;
  bool buttonA = false;
  bool buttonB = false;
};
