// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/followWaypoints.h"

followWaypoints::followWaypoints(DriveSubsystem &drivetrain, std::vector<frc::Pose2d> waypoints, units::meters_per_second_t driveSpeed) 
{
  // Use addRequirements() here to declare subsystem dependencies.
  m_drivetrain = &drivetrain;
  AddRequirements({m_drivetrain});
  m_waypoints.assign(waypoints.begin(), waypoints.end());
  robot_speed = driveSpeed;
  finished = false;
}

// Called when the command is initially scheduled.
void followWaypoints::Initialize() 
{
  desiredPose = m_waypoints.front();
  m_waypoints.pop_front(); 
}

// Called repeatedly when this Command is scheduled to run
void followWaypoints::Execute()
{
  currentPose = m_drivetrain->GetPose(); 

  if((fabs((double)currentPose.X() - (double)desiredPose.X()) < 0.01) && (fabs((double)currentPose.Y() - (double)desiredPose.Y()) < 0.01) /*&& (currentPose.Rotation().Degrees() - desiredPose.Rotation().Degrees() < frc::Rotation2d(5_deg).Degrees())*/)
  {
    if(m_waypoints.size() > 0)
    {
      desiredPose = m_waypoints.front();
      m_waypoints.pop_front();
    }
    else
    {
      finished = true;
    }
  }

  if(!finished)
  {
    robot_speed = 0.25_mps; //0-1.0 aka Z
    alpha = atan2(((double)desiredPose.Y() - (double)currentPose.Y()) , ((double)desiredPose.X() - (double)currentPose.X()));
    xVal = robot_speed * cos(alpha);
    yVal = robot_speed * sin(alpha);
    thetaVal = 0_rad_per_s;
    m_drivetrain->Drive(xVal, yVal, thetaVal, false, false);
  }

  frc::SmartDashboard::PutNumber("DesiredPoseX", (double)desiredPose.X());
  frc::SmartDashboard::PutNumber("DesiredPoseY", (double)desiredPose.Y());
  frc::SmartDashboard::PutNumber("CurrentPoseX", (double)currentPose.X());
  frc::SmartDashboard::PutNumber("CurrentPoseY", (double)currentPose.Y());
  frc::SmartDashboard::PutNumber("AutoDriveAlpha", (double)alpha);
  frc::SmartDashboard::PutNumber("AutoDriveYval", (double)yVal);
  frc::SmartDashboard::PutNumber("AutoDriveXval", (double)xVal);
}

// Called once the command ends or is interrupted.
void followWaypoints::End(bool interrupted) {
  m_drivetrain->Drive(0_mps, 0_mps , 0_rad_per_s, true, false);
}

// Returns true when the command should end.
bool followWaypoints::IsFinished()
{
  return finished;
}