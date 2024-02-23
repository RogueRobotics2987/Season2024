// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoDriveStateMachine.h"

AutoDriveStateMachine::AutoDriveStateMachine() {}
  
AutoDriveStateMachine::AutoDriveStateMachine(
    DriveSubsystem &drive,
    LimelightSubsystem &limelight,
    frc::XboxController &driveXbox,
    frc::XboxController &auxXbox,
    CommandMessenger &message,
  std::vector<frc::Pose2d> waypoints,
  std::vector<units::meters_per_second_t> driveSpeed,
  std::vector<units::meters_per_second_t> cruiseSpeed,
  bool limeLight)
{
  // Use addRequirements() here to declare subsystem dependencies.
  m_drive = &drive;
  AddRequirements(m_drive);
  m_limelight = &limelight;
  AddRequirements(m_limelight);
  m_messager = &message;

  m_driverController = &driveXbox;
  m_auxController = &auxXbox;

  m_waypoints.assign(waypoints.begin(), waypoints.end());
  m_driveSpeed.assign(driveSpeed.begin(), driveSpeed.end());
  m_cruiseSpeed.assign(cruiseSpeed.begin(), cruiseSpeed.end());
  // maxSpeed = driveSpeed;

  for(int i = 0; i < (int)waypoints.size()-1; ++i)
  {
    deltaY = fabs((double)waypoints[i].Y() - (double)waypoints[i+1].Y());
    deltaX = fabs((double)waypoints[i].X() - (double)waypoints[i+1].X());
    m_waypointDistance.push_back(hypot(deltaX, deltaY));
  }

  apriltagBool = limeLight;
}

// Called when the command is initially scheduled.
void AutoDriveStateMachine::Initialize()
{
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->PutNumber("pipeline",1);

  m_messager->setMessage("Empty");

  deltaX = 0;
  deltaY = 0;
  lastPointSpeed = 0_mps;
  m_waypoints.pop_front();
  desiredPose = m_waypoints.front();
  m_waypoints.pop_front(); 
  m_driveSpeed.pop_front();
  pointSpeed = m_driveSpeed.front();
  m_driveSpeed.pop_front();
  m_cruiseSpeed.pop_front();
  cruiseSpeed = m_cruiseSpeed.front();
  m_cruiseSpeed.pop_front();
  lastPose = m_drive->GetPose();

  accumulatedError = 0;
}

// Called repeatedly when this Command is scheduled to run
void AutoDriveStateMachine::Execute()
 {
    frc::SmartDashboard::PutString("Message", m_messager->GetMessage());
  switch (drive_state) 
  {
    case NONE:
      // not sure what should happen

      if(noteFollowState == true){
        drive_state = NOTE_FOLLOW;
        standard = false;

      }
      else if(aprilFollowState == true){
        drive_state = APRIL_FOLLOW;
        standard = false;
      }
      else if(pathFollowState == true){
        drive_state = PATH_FOLLOW;
        standard = false;
      }

    break;

    case NOTE_FOLLOW:
      txNote = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("tx",0.0);

      if(m_messager->GetMessage().compare("Pickup") != 0)   // TODO: DOUBLE CHECK!!!
      {
        drive_state = NONE;
      }

      if(txNote > 7 || txNote < -7)
      {
        rotNote = units::angular_velocity::radians_per_second_t((0 + txNote) * kpNote);
      }
      else
      {
        rotNote = units::angular_velocity::radians_per_second_t(0);
      }
      
      m_drive->Drive(units::velocity::meters_per_second_t(m_driverController->GetLeftY()), units::velocity::meters_per_second_t(0), rotNote, false, false);


      if(nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("tv",0) == 0)
      {
        drive_state = NONE;
        noteFollowState = false;
      }
      
    break;

    case APRIL_FOLLOW:
      txApril = m_limelight->GetAprilTagtx() - 5;   // what is this number?

      if(m_messager->GetMessage().compare("Loaded") != 0 || m_messager->GetMessage().compare("ShooterWarmup"))  // TODO: oduble check!!!
      {
        drive_state = NONE;
      }

      //rotApril = units::angular_velocity::radians_per_second_t(0);
      // if(tx > 7 || tx < -7){
      rotApril = units::angular_velocity::radians_per_second_t((0-txApril) * kpApril);

      speedY = Deadzone(m_driverController->GetLeftY());
      speedX = Deadzone(m_driverController->GetLeftY());

      if((fabs(speedY) + fabs(speedX) + fabs(rotApril.value())) < .05)
      {
        NoJoystickInput = true;
      }
      else
      {
        NoJoystickInput = false;
      }
      
      m_drive->Drive(units::velocity::meters_per_second_t(speedY), units::velocity::meters_per_second_t(speedX), rotApril, false, NoJoystickInput);

      if(standard == true){
        drive_state = NONE;
        aprilFollowState = false;
      }

    break;

    case PATH_FOLLOW:  
      currentPose = m_drive->GetPose();

      if((fabs((double)currentPose.X() - (double)desiredPose.X()) < threshold) 
          && (fabs((double)currentPose.Y() - (double)desiredPose.Y()) < threshold) 
          /*&& ((fabs(DistanceBetweenAngles((double)desiredPose.Rotation().Degrees(), (double)currentPose.Rotation().Degrees()))) < 5)*/)
      {
        m_messager->setMessage("TurnOnIntake");\

        if(m_waypoints.size() > 0) 
        {
          desiredPose = m_waypoints.front();
          m_waypoints.pop_front();
          lastPointSpeed = pointSpeed;
          pointSpeed = m_driveSpeed.front();
          m_driveSpeed.pop_front();
          cruiseSpeed = m_cruiseSpeed.front();
          m_cruiseSpeed.pop_front();
          currentDistance = m_waypointDistance.front();
          m_waypointDistance.pop_front();
          distanceTraveled = 0;

          if(m_waypoints.size() == 0)
          {
              threshold = 0.05;
          }
        }
        else
        {
          finished = true;
          m_drive->Drive(0_mps, 0_mps , 0_rad_per_s, true, false);
          distanceTraveled = 0;
          totalDistance = 0;
          deltaX = 0;
          deltaY = 0;
          threshold = 0.1;
        }
      }

      deltaX = fabs((double)lastPose.X() - (double)currentPose.X());
      deltaY = fabs((double)lastPose.Y() - (double)currentPose.Y());
      distanceTraveled = distanceTraveled + hypot(deltaX, deltaY);

      if(DebugConstants::debugAuto == true)
      {
          frc::SmartDashboard::PutNumber("Hypot(X,Y)", hypot(deltaX, deltaY));
      }

      if(!finished)
      {
        // robotSpeed = 1_mps;
        if((distanceTraveled + (distanceTraveled * 0.1)) >= currentDistance)
        {
        robotSpeed = 0.5_mps;
        }
        else if((currentDistance-distanceTraveled) <= 0.65)
        {
        accumulatedError += 5E-3 * (currentDistance - distanceTraveled);
        double x = (currentDistance-distanceTraveled) / 0.8;
        robotSpeed = 1 * x * (cruiseSpeed - pointSpeed) + pointSpeed + (units::meters_per_second_t)accumulatedError; //0-100% of max speed aka Z

        if(DebugConstants::debugAuto == true)
        {
            frc::SmartDashboard::PutNumber("Xvalue2", x);
        }
        }
        else if(distanceTraveled <= 0.5)
        {
        double x = distanceTraveled / 0.5;
        robotSpeed = 1 * x * (cruiseSpeed - lastPointSpeed) + lastPointSpeed + 0.1_mps; //0-100% of max speed aka Z

        if(DebugConstants::debugAuto == true)
        {
            frc::SmartDashboard::PutNumber("Xvalue1", x);
        }
        }
        else{
        robotSpeed = cruiseSpeed;
        }

        alpha = atan2(((double)desiredPose.Y() - (double)currentPose.Y()) , ((double)desiredPose.X() - (double)currentPose.X()));
        alpha = alpha - (double)currentPose.Rotation().Radians();
        xVal = robotSpeed * cos(alpha);
        yVal = robotSpeed * sin(alpha);

        thetaDouble = (DistanceBetweenAngles((double)desiredPose.Rotation().Degrees(), ((double)currentPose.Rotation().Degrees())));
        thetaDouble = thetaDouble * (3.14/180.0);
        thetaDouble = thetaDouble * AutoConstants::kPThetaController;

        thetaVal = thetaDouble * 1_rad_per_s;
        m_drive->Drive(xVal, yVal, false, false);

        if(apriltagBool == true)
        {
        txApril = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("tx",0);
        units::angular_velocity::radians_per_second_t rot = units::angular_velocity::radians_per_second_t(0);

        if( txApril != -9999 && (txApril > 2.5 || txApril < -2.5))
        {
            rot = units::angular_velocity::radians_per_second_t((0-txApril) * kpApril);
        }
        else
        {
            rot = units::angular_velocity::radians_per_second_t(0);
        }
        
        m_drive->Drive(rot, false, false);
        }
        else
        {
        m_drive->Drive(thetaVal, false, false);

        }
      }

      lastPose = currentPose;
      
      if(DebugConstants::debugAuto == true)
      {
          frc::SmartDashboard::PutNumber("DesiredPoseX", (double)desiredPose.X());
          frc::SmartDashboard::PutNumber("DesiredPoseY", (double)desiredPose.Y());
          frc::SmartDashboard::PutNumber("DesiredAngle", (double)desiredPose.Rotation().Degrees());
          frc::SmartDashboard::PutNumber("CurrentPoseX", (double)currentPose.X());
          frc::SmartDashboard::PutNumber("CurrentPoseY", (double)currentPose.Y());
          frc::SmartDashboard::PutNumber("CurrentAngle", (double)currentPose.Rotation().Degrees());
          frc::SmartDashboard::PutNumber("AutoDriveAlpha", (double)alpha);
          frc::SmartDashboard::PutNumber("AutoDriveYval", (double)yVal);
          frc::SmartDashboard::PutNumber("AutoDriveXval", (double)xVal);
          frc::SmartDashboard::PutNumber("robotSpeed", (double)robotSpeed);
          frc::SmartDashboard::PutNumber("currentDistance", (double)currentDistance);
          frc::SmartDashboard::PutNumber("distanceTraveled", (double)distanceTraveled);
          frc::SmartDashboard::PutNumber("percentDistanceTraveled", (double)(distanceTraveled/currentDistance));
          frc::SmartDashboard::PutNumber("lastPoseX", (double)lastPose.X());
          frc::SmartDashboard::PutNumber("lastPoseY", (double)lastPose.Y());
          frc::SmartDashboard::PutNumber("Threshold", threshold);
          frc::SmartDashboard::PutNumber("accumulatedError", accumulatedError);
          frc::SmartDashboard::PutNumber("cruiseSpeed", (double)cruiseSpeed);
          frc::SmartDashboard::PutNumber("pointSpeed", (double)pointSpeed);
      }

      if(finished == true){
        drive_state = NONE;
        pathFollowState = false;
        m_messager->setMessage("Shoot");
      }

      break;

    default:
      drive_state = NONE;
      break;
  }

}

// Called once the command ends or is interrupted.
void AutoDriveStateMachine::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoDriveStateMachine::IsFinished()
{
  return false;
}


float AutoDriveStateMachine::Deadzone(float x)
{
  if ((x < 0.1) &&  (x > -0.1))
  {
    x=0;
  }
  else if (x >= 0.1)
  {
    x = x - 0.1;
  }
  else if (x <= -0.1)
  {
    x = x + 0.1;
  }
  return(x);
}

double AutoDriveStateMachine::DistanceBetweenAngles(double targetAngle, double sourceAngle)
{
  double a = targetAngle - sourceAngle;
  if(a > 180)
  {
    a = a + -360;
  }
  else if(a < -180)
  {
    a = a + 360;
  }
  else
  {
    a = a;
  }

  return a;
}