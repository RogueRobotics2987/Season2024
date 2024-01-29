// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/FollowWaypoints.h"

FollowWaypoints::FollowWaypoints(DriveSubsystem &drivetrain, std::vector<frc::Pose2d> waypoints, units::meters_per_second_t driveSpeed) 
{
  // Use addRequirements() here to declare subsystem dependencies.
  m_drivetrain = &drivetrain;
  AddRequirements({m_drivetrain});
  m_waypoints.assign(waypoints.begin(), waypoints.end());
  maxSpeed = driveSpeed;

  for(int i = 0; i < (int)waypoints.size()-1; ++i){
    deltaY = fabs((double)waypoints[i].Y() - (double)waypoints[i+1].Y());
    deltaX = fabs((double)waypoints[i].X() - (double)waypoints[i+1].X());
    totalDistance = totalDistance + hypot(deltaX, deltaY);
  }
}

// Called when the command is initially scheduled.
void FollowWaypoints::Initialize() 
{
  desiredPose = m_waypoints.front();
  m_waypoints.pop_front(); 
}

// Called repeatedly when this Command is scheduled to run
void FollowWaypoints::Execute()
{
  currentPose = m_drivetrain->GetPose(); 

  if((fabs((double)currentPose.X() - (double)desiredPose.X()) < 0.05) 
    && (fabs((double)currentPose.Y() - (double)desiredPose.Y()) < 0.05) 
    && (fabs((double)currentPose.Rotation().Degrees() - (double)desiredPose.Rotation().Degrees()) < 5))
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

  deltaX = fabs((double)lastPose.X() - (double)currentPose.X());
  deltaY = fabs((double)lastPose.Y() - (double)currentPose.Y());
  distanceTraveled = distanceTraveled + hypot(deltaX, deltaY);

  if(!finished)
  {
    if(distanceTraveled >= totalDistance){
      robotSpeed = 0.1_mps;
    }
    else if((totalDistance-distanceTraveled) <= 0.75){
      double x = (totalDistance-distanceTraveled) / 0.75;
      robotSpeed = 1 * (x) * maxSpeed; //0-100% of max speed aka Z
    }
    else if(distanceTraveled <= 0.5){
      double x = distanceTraveled / 0.5;
      robotSpeed = 1 * x * maxSpeed + 0.1_mps; //0-100% of max speed aka Z
    }
    else{
      robotSpeed = maxSpeed;
    }

    alpha = atan2(((double)desiredPose.Y() - (double)currentPose.Y()) , ((double)desiredPose.X() - (double)currentPose.X()));
    alpha = alpha - (double)currentPose.Rotation().Radians();
    xVal = robotSpeed * cos(alpha);
    yVal = robotSpeed * sin(alpha);
    double thetaDouble = (((double)desiredPose.Rotation().Radians() - (double)currentPose.Rotation().Radians()) * AutoConstants::kPThetaController);
    thetaVal = thetaDouble * 1_rad_per_s;
    m_drivetrain->Drive(xVal, yVal, thetaVal, false, false);
  }

  lastPose = currentPose;
  
  if(DebugConstants::debug == true){
    frc::SmartDashboard::PutNumber("DesiredPoseX", (double)desiredPose.X());
    frc::SmartDashboard::PutNumber("DesiredPoseY", (double)desiredPose.Y());
    frc::SmartDashboard::PutNumber("CurrentPoseX", (double)currentPose.X());
    frc::SmartDashboard::PutNumber("CurrentPoseY", (double)currentPose.Y());
    frc::SmartDashboard::PutNumber("AutoDriveAlpha", (double)alpha);
    frc::SmartDashboard::PutNumber("AutoDriveYval", (double)yVal);
    frc::SmartDashboard::PutNumber("AutoDriveXval", (double)xVal);
    frc::SmartDashboard::PutNumber("robotSpeed", (double)robotSpeed);
    frc::SmartDashboard::PutNumber("totalDistance", (double)totalDistance);
    frc::SmartDashboard::PutNumber("distanceTraveled", (double)distanceTraveled);
    frc::SmartDashboard::PutNumber("percentDistanceTraveled", (double)(distanceTraveled/totalDistance));
  }
}

// Called once the command ends or is interrupted.
void FollowWaypoints::End(bool interrupted) {
  m_drivetrain->Drive(0_mps, 0_mps , 0_rad_per_s, true, false);
  distanceTraveled = 0;
  finished = false;
  totalDistance = 0;
  deltaX = 0;
  deltaY = 0;
}

// Returns true when the command should end.
bool FollowWaypoints::IsFinished()
{
  return finished;
}
