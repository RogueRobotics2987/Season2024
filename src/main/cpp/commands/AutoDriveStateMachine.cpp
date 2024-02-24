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
    std::vector<AutoPaths::AutoPath> &path
)
{
  // Use addRequirements() here to declare subsystem dependencies.
  m_drive = &drive;
  AddRequirements(m_drive);
  m_limelight = &limelight;
  AddRequirements(m_limelight);
  m_messager = &message;

  m_driverController = &driveXbox;
  m_auxController = &auxXbox;

  paths.assign(path.begin(), path.end());
}

// Called when the command is initially scheduled.
void AutoDriveStateMachine::Initialize()
{
  m_messager->SetDriveMessage("Shoot"); 
}

// Called repeatedly when this Command is scheduled to run
void AutoDriveStateMachine::Execute()
{
    switch (drive_state) 
    {
    case NONE:
      if(m_messager->GetAuxMessage().compare("NextPath") == 0)
      {
        pathFollowState = true;
      }

      if(noteFollowState == true){
        drive_state = NOTE_FOLLOW;
        standard = false;
      }
      else if(aprilFollowState == true){
        drive_state = APRIL_FOLLOW;
        standard = false;
      }
      else if(pathFollowState == true && paths.empty() == false){
        drive_state = PRE_PATH_FOLLOW;
        standard = false;
      }

    break;

    case NOTE_FOLLOW:
      txNote = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("tx",0.0);

      if(m_messager->GetAuxMessage().compare("Pickup") != 0)   // TODO: DOUBLE CHECK!!!
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

      if(m_messager->GetAuxMessage().compare("Loaded") != 0 || m_messager->GetAuxMessage().compare("ShooterWarmup") !=  0)  // TODO: oduble check!!!
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

    case PRE_PATH_FOLLOW:

      m_waypoints.clear();
      m_driveSpeed.clear();
      m_cruiseSpeed.clear();
      m_command.clear();
      m_limefollowAuto.clear();

      m_waypoints.assign(paths.front().Waypoints.begin(), paths.front().Waypoints.end());
      m_driveSpeed.assign(paths.front().PointSpeed.begin(), paths.front().PointSpeed.end());
      m_cruiseSpeed.assign(paths.front().CruiseSpeed.begin(), paths.front().CruiseSpeed.end());
      m_command.assign(paths.front().Command.begin(), paths.front().Command.end());
      m_limefollowAuto.assign(paths.front().limelightFollow.begin(), paths.front().limelightFollow.end());
      // maxSpeed = driveSpeed;

      for(int i = 0; i < (int)paths.front().Waypoints.size()-1; ++i)
      {
        deltaY = fabs((double)paths.front().Waypoints[i].Y() - (double)paths.front().Waypoints[i+1].Y());
        deltaX = fabs((double)paths.front().Waypoints[i].X() - (double)paths.front().Waypoints[i+1].X());
        m_waypointDistance.push_back(hypot(deltaX, deltaY));
      }

      deltaX = 0;
      deltaY = 0;
      accumulatedError = 0;
      lastPointSpeed = 0_mps;
      
      std::cout << paths.size() << " pathsSize" << std::endl;

      std::cout << m_waypoints.size() << " waypointsVectorSize" << std::endl;

      desiredPose = m_waypoints.front();

      if(m_waypoints.empty() == false){
        m_waypoints.pop_front();
      }

      pointSpeed = m_driveSpeed.front();

      if(m_driveSpeed.empty() == false){
        m_driveSpeed.pop_front();
      }

      cruiseSpeed = m_cruiseSpeed.front();

      if(m_cruiseSpeed.empty() == false){
        m_cruiseSpeed.pop_front();
      }

      command = m_command.front();
      
      if(m_command.empty() == false){
        m_command.pop_front();
      }

      apriltagBool = m_limefollowAuto.front();

      if(m_limefollowAuto.empty() == false){
        m_limefollowAuto.pop_front();
      }

      lastPose = m_drive->GetPose();

      drive_state = PATH_FOLLOW;

    break;

    case PATH_FOLLOW:  
    
      currentPose = m_drive->GetPose();

      if((fabs((double)currentPose.X() - (double)desiredPose.X()) < threshold) 
          && (fabs((double)currentPose.Y() - (double)desiredPose.Y()) < threshold) 
          /*&& ((fabs(DistanceBetweenAngles((double)desiredPose.Rotation().Degrees(), (double)currentPose.Rotation().Degrees()))) < 5)*/)
      {

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
          command = m_command.front();
          m_command.pop_front();
          apriltagBool = m_limefollowAuto.front();
          m_limefollowAuto.pop_front();
          distanceTraveled = 0;

          if(m_waypoints.size() == 0)
          {
              threshold = 0.05;
          }

          // std::cout << "command msg " << command << std::endl;  
          if(command.compare("Intake") == 0){ 
            m_messager->SetDriveMessage("TurnOnIntake");
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
          txApril = m_limelight->FilteredPhotonYaw();

          std::cout << "TX april " << txApril << std::endl;

          units::angular_velocity::radians_per_second_t rot = units::angular_velocity::radians_per_second_t(0);

          rot = units::angular_velocity::radians_per_second_t((0-txApril) * kpApril);
          
          std::cout << "RotationValue " << rot.value() << std:: endl;
          
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
        m_messager->SetDriveMessage("Shoot");
        finished = false;

        std::cout << "paths Size " << paths.size() << std::endl;

          paths.pop_front();
          std::cout << "paths Size " << paths.size() << std::endl;

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