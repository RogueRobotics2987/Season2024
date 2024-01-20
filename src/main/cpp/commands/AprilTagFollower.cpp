// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AprilTagFollower.h"

AprilTagFollower::AprilTagFollower(){}
AprilTagFollower::AprilTagFollower(LimelightPose &limePose, DriveSubsystem &drivetrain) {
  m_limePose = &limePose;
  m_drivetrain = &drivetrain;
  AddRequirements({m_limePose});
  AddRequirements({m_drivetrain});
}

// Called when the command is initially scheduled.
void AprilTagFollower::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AprilTagFollower::Execute() {
  double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("tx",0.0);
    // double ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("ty",0.0);
    units::angular_velocity::radians_per_second_t rot = units::angular_velocity::radians_per_second_t(0);
    // units::velocity::meters_per_second_t speed = units::velocity::meters_per_second_t(0);
    if(tx > 7){
      rot = units::angular_velocity::radians_per_second_t(1);
    }
    else if(tx < -7){
      rot = units::angular_velocity::radians_per_second_t(-1);
    }
    else{
      rot = units::angular_velocity::radians_per_second_t(0);
    }
    // if(ty > -15 && ty < -8){
    //   speed = units::velocity::meters_per_second_t(-0.5);
    // }
    // else if(ty <= -17){
    //   speed = units::velocity::meters_per_second_t(0.5);
    // }
    // else{
    //   speed = units::velocity::meters_per_second_t(0);
    // }
    // m_drivetrain->Drive(speed, units::velocity::meters_per_second_t(0), rot, false, false);
    m_drivetrain->Drive(units::velocity::meters_per_second_t(0), units::velocity::meters_per_second_t(0), rot, false, false);

}

// Called once the command ends or is interrupted.
void AprilTagFollower::End(bool interrupted) {}

// Returns true when the command should end.
bool AprilTagFollower::IsFinished() {
  return false;
}
