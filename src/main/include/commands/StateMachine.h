// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// #pragma once

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

// #include "networktables/NetworkTableInstance.inc"

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
// class StateMachine
//     : public frc2::CommandHelper<frc2::Command, StateMachine> {
//  public:
//   StateMachine();
//   StateMachine(
//     DriveSubsystem &drive,
//     LimelightSubsystem &limelight,
//     ArmSubsystem &arm,
//     ClimberSubsystem &climb,
//     ColorSensorSubsystem &color,
//     IntakeSubsystem &intake,
//     ShooterSubsystem &shooter,
//     frc::XboxController &driverController,
//     frc::XboxController &auxController);

//   void Initialize() override;

//   void Execute() override;

//   void End(bool interrupted) override;

//   bool IsFinished() override;

//   float Deadzone(float x);

//   double DistanceBetweenAngles(double targetAngle, double sourceAngle);


//   //TODO: WHY ARE THESE PUBLIC? REVIEW THESE AND THEY PROBABLY NEED TO GO TO PRIVATE

//  private:
//   enum intakeState 
//   {
//     EMPTY,
//     SPIT_OUT,
//     PICKUP,
//     BACKUP,
//     LOADED,
//     SHOOTER_WARMUP,
//     SHOOT,
//     FORWARD_ARM_AMP,
//     BACKWARD_ARM_AMP,
//     ARM_TRAP,
//     // CHAIN_CLIMB,
//     ARMS_RETRACT
//   };

//   intakeState state = EMPTY;
//   std::vector<double> RedDistVector;
//   std::vector<double> BlueDistVector;

//   ArmSubsystem* m_arm = nullptr;
//   ClimberSubsystem* m_climb = nullptr;
//   ColorSensorSubsystem* m_colorSensor = nullptr;
//   IntakeSubsystem* m_intake = nullptr;
//   ShooterSubsystem* m_shooter = nullptr;
//   LimelightSubsystem* m_limelight = nullptr;
//   DriveSubsystem* m_drive = nullptr;

//   frc::XboxController* m_driverController = nullptr;
//   frc::XboxController* m_auxController = nullptr;

//   void SetBoolsFalse();

//   double speedX = 0;
//   bool NoJoystickInput = false;

//   double blueDist = 0;
//   double redDist = 0;
//   int apriltagID = 0;

//   bool pickupNote = false;        // if auto/teleop want to pickup a note (OrangeCheerio)
//   // bool chainClimb = false;
//   bool raiseClimber = false;
//   bool lowerArmTrap = false;
//   bool upperArmTrap = false;
//   bool emptyIntake = false;       // self explainitory  // spit out note when button pov0 is pressed
//   bool warmUpShooter = false;     // warmup shooter (warmMilk)
//   bool moveNote2Shoot = false;    // move note into shooter
//   bool placeInForwardAmp = false;
//   bool placeInBackwardsAmp = false;
//   bool placeInTrap = false;

//   bool resetLoaded = false;

//   bool noteFollowState = false;
//   bool aprilFollowState = false;
//   bool driveButtonA = false;
//   bool driveButtonB = false;
//   bool driveLeftBumperPushed = false;

//   units::meter_t CAMERA_HEIGHT = units::meter_t(0.635);
//   units::meter_t TAREGT_HEIGHT = units::meter_t(1.5);
//   units::angle::radian_t CAMERA_PITCH = units::angle::radian_t(0.44);

//   double tx = 0.0;
//   int time = 0;       //keep track of shooter iterations
//   std::vector<double> targetIDs;
//   std::vector<photon::PhotonTrackedTarget> myTargets;
//   double targetData = 0;
//   photon::PhotonTrackedTarget filteredTarget;
//   int filteredTargetID = -1;
//   units::meter_t filteredRange = 0_m;

//   int timeDrop = 0;   //keep track of dropper iterations

//   double magEncoderPos = 0.0;

//   double rot = 0;
//   double kp = 0.02206;//0.0248175;//0.009927;
//   double speedY = 0;

// //TODO: MOVED OVER FROM DriveStateMachine
//   units::angular_velocity::radians_per_second_t rotNote = units::angular_velocity::radians_per_second_t(0);
//   units::velocity::meters_per_second_t speedNote = units::velocity::meters_per_second_t(0);
//   double kpNote = 0.05;
//   double txNote = 0.0;

//   units::angular_velocity::radians_per_second_t rotApril = units::angular_velocity::radians_per_second_t(0);
//   double kpApril = 0.09;
//   double txApril = 0.0;

//   bool runIntake = false;
//   bool runShooterWarmup = false;
//   bool buttonA = false;
//   bool buttonB = false;

//   double currentHeading = 0;
//   double desiredHeading = 0;
// };