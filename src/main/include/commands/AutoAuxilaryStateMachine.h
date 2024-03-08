// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// #pragma once

// #include "../cpp/CommandMessenger.cpp"
// #include "subsystems/ArmSubsystem.h"
// #include "subsystems/ClimberSubsystem.h"
// #include "subsystems/ColorSensorSubsystem.h"
// #include "subsystems/IntakeSubsystem.h"
// #include "subsystems/ShooterSubsystem.h"
// #include "subsystems/DriveSubsystem.h"
// #include "subsystems/LimelightSubsystem.h"

// #include <frc2/command/Command.h>
// #include <frc2/command/CommandHelper.h>
// #include <frc2/command/CommandPtr.h>
// #include <frc2/command/FunctionalCommand.h>

// #include <frc/Joystick.h>
// #include <frc/XboxController.h>
// #include <frc/smartdashboard/SmartDashboard.h> 

// #include "photon/PhotonUtils.h"
// #include "photon/PhotonCamera.h"
// #include "photon/PhotonPoseEstimator.h"

// /**
//  * An example command.
//  *
//  * <p>Note that this extends CommandHelper, rather extending Command
//  * directly; this is crucially important, or else the decorator functions in
//  * Command will *not* work!
//  */
// class AutoAuxilaryStateMachine
//     : public frc2::CommandHelper<frc2::Command, AutoAuxilaryStateMachine> {
//  public:
//   AutoAuxilaryStateMachine();
//   AutoAuxilaryStateMachine(ArmSubsystem &arm, ClimberSubsystem &climb, ColorSensorSubsystem &color, 
//                IntakeSubsystem &intake, ShooterSubsystem &shooter, frc::XboxController &driveXbox, frc::XboxController &auxXbox, CommandMessenger &messager);// LimelightSubsystem &limelight, 
//               // DriveSubsystem &drivetrain);

//   units::angular_velocity::radians_per_second_t rot = units::angular_velocity::radians_per_second_t(0);
//   units::velocity::meters_per_second_t speed = units::velocity::meters_per_second_t(0);
//   double kp = 0.02206;//0.0248175;//0.009927;
//   double speedY = 0;
//   bool NoJoystickInput = false;

//   void Initialize() override;

//   void Execute() override;

//   void End(bool interrupted) override;

//   bool IsFinished() override;

//  private:
//   //enum intakeState {EMPTY, SPIT_OUT, PICKUP, LOADED, SHOOTER_WARMUP, SHOOT, /*DROP_WARMUP, DROP*/ WAIT};
//   enum intakeState {EMPTY, SPIT_OUT, PICKUP, LOADED, SHOOTER_WARMUP, SHOOT, DROP_ARMS, DROP_SHOOTER, RAISE_SHOOTER, LOWER_ARM_EXTEND_INITIAL, 
//     UPPER_ARM_EXTEND_INITIAL, ARM_TRAP, ARM_AMP, DROP, ARM_RETRACT_INITIAL, ARM_RETRACT_FINAL, BACKUP, NOTE_HUNTING, NEXT_PATH};
//   intakeState state = EMPTY;

//   std::vector<double> RedDistVector;
//   std::vector<double> BlueDistVector;

//   double blueDist = 0;
//   double redDist = 0;

//   int apriltagID = 0;

//   ArmSubsystem* m_arm = nullptr;
//   ClimberSubsystem* m_climb = nullptr;
//   ColorSensorSubsystem* m_colorSensor = nullptr;
//   IntakeSubsystem* m_intake = nullptr;
//   ShooterSubsystem* m_shooter = nullptr;
//   CommandMessenger* m_messager;
//  // LimelightSubsystem* m_limelight = nullptr;
//   // DriveSubsystem* m_drivetrain = nullptr;

//   frc::XboxController* m_driverController = nullptr;
//   frc::XboxController* m_auxController = nullptr;

//   // bool huntingNote = false;

//   bool pickupNote = false;        // if auto/teleop want to pickup a note (OrangeCheerio)

//   bool emptyIntake = false;       // self explainitory

//   bool warmUpShooter = false;     // warmup shooter (warmMilk)
//   bool moveNote2Shoot = false;    // move note into shooter

//   bool placeInTrap = false;
//   bool placeInAmp = true;

//   units::meter_t CAMERA_HEIGHT = units::meter_t(0.635);
//   units::meter_t TAREGT_HEIGHT = units::meter_t(1.5);
//   units::angle::radian_t CAMERA_PITCH = units::angle::radian_t(0.44);

//   bool pov0 = false; // spit out note when button pov0 is pressed

//   //bool moveArm2Drop = false;      // warmup dropper (move arm into position)
//   //bool dropNote = false;          // activate dropper
//   //bool waitForArm = false;        // waits for the armSubsystem/dropper state machine
//   float Deadzone(float x);
//   double tx = 0.0;
//   int time = 0;       //keep track of shooter iterations
//   std::vector<double> targetIDs;
//   std::vector<photon::PhotonTrackedTarget> myTargets;
//   double targetData = 0;
//   photon::PhotonTrackedTarget filteredTarget;
//   int filteredTargetID = -1;
//   units::meter_t filteredRange = 0_m;

//   std::string lastMessage;

//   int timeDrop = 0;   //keep track of dropper iterations

//   double magEncoderPos = 0.0;
//   };