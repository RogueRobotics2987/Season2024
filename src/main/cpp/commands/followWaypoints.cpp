// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/followWaypoints.h"

followWaypoints::followWaypoints(DriveSubsystem &drivetrain, std::vector<frc::Pose2d> waypoints, units::meter_t driveSpeed) 
{
  // Use addRequirements() here to declare subsystem dependencies.
  m_drivetrain = &drivetrain;
  AddRequirements({m_drivetrain});
  m_waypoints = waypoints;
}

// Called when the command is initially scheduled.
void followWaypoints::Initialize() 
{
  desiredPose = m_waypoints.front(); //TODO needs to properly read the first value
}

// Called repeatedly when this Command is scheduled to run
void followWaypoints::Execute()
{
  currentPose = m_drivetrain->GetPose();

  if((currentPose.X() - desiredPose.X() < 0.2_m) && (currentPose.Y() - desiredPose.Y() < 0.2_m) && (currentPose.Rotation() - desiredPose.Rotation() < frc::Rotation2d(5_deg)))
  {
    if(waypoint.size() > 0)
    {
      desiredPose = waypoint.next();
    }
    else
    {
      finished = true;
    }
  }
  if(!finished)
  {
    robot_speed = 0.5; //0-1.0 aka Z
    alpha = atan2((currentPose.Y() - desiredPose.Y()) / (currentPose.X() - desiredPose.X()));
    xVal = robot_speed * cos(alpha);
    yVal = robot_speed * sin(alpha);
    thetaVal = 0;
    m_drivetrain->Drive(xVal, yVal, thetaVal, true, false);
  }

}

// Called once the command ends or is interrupted.
void followWaypoints::End(bool interrupted) {}

// Returns true when the command should end.
bool followWaypoints::IsFinished()
{
  return false;
}
