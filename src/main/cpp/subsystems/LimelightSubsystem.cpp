// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LimelightSubsystem.h"

LimelightSubsystem::LimelightSubsystem() = default;

// This method will be called once per scheduler run
void LimelightSubsystem::Periodic()
{
    targetIDs.clear();
    
    //works to read values from the network tables for limelight and pushes them back out to confirm we are reading values
    // botPose = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("botpose", std::span<const double>({1 , 0, 0, 0, 0, 0}));

    //TODO Coment this back in eventually once limelights are put onto the robot

    //AprilTagstx = ;//nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("tx",0.0);
    //AprilTagsty = ;//nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("ty",0.0);
    Notetx = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("tx",0.0);
    Notety = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("ty",0.0);
    auto threeDPose = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumberArray("botpose",std::vector<double>(6));
    auto threeDPose2 = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumberArray("botpose blue",std::vector<double>(6));


    double distanceToTagInMeters = threeDPose[0];
    frc::SmartDashboard::PutNumber("Distance", distanceToTagInMeters);
    frc::SmartDashboard::PutNumber("Distance2", threeDPose2[0]);

    result = camera.GetLatestResult();
    hasTarget = result.HasTargets();

    filteredTargetID = -1;


    if(hasTarget == true){
        tempTargets = result.GetTargets();

        myTargets.assign(tempTargets.begin(), tempTargets.end());

        for(unsigned int i = 0; i < myTargets.size(); i++)
        {
            targetIDs.emplace_back(myTargets.at(i).GetFiducialId()); //TODO remove(?)

            if(myTargets.at(i).GetFiducialId() == 4 || myTargets.at(i).GetFiducialId() == 7)
            {
                filteredTarget = myTargets.at(i);
                filteredTargetID = filteredTarget.GetFiducialId();
                //TODO have the yaw on our robot search for 0. include our specific id into the calc dist to target

                filteredRange = photon::PhotonUtils::CalculateDistanceToTarget(
                CAMERA_HEIGHT, TAREGT_HEIGHT, CAMERA_PITCH,
                units::degree_t{filteredTarget.GetPitch()});

            }
        }   
    }

    if(DebugConstants::debugLimelight == true){
        frc::SmartDashboard::PutNumber("FilteredRange", filteredRange.value());
        frc::SmartDashboard::PutNumber("FilteredYaw", filteredTarget.GetYaw());
        frc::SmartDashboard::PutNumber("FilteredPitch", filteredTarget.GetPitch());
        frc::SmartDashboard::PutNumber("FilteredID", filteredTargetID);
    }
}


bool LimelightSubsystem::PhotonHasTarget(){
    bool hasTargetFiltered = false;

    if(hasTarget == true)
    {
        if(filteredTargetID == 4 || filteredTargetID == 7)
        {
            hasTargetFiltered = true;
        }
    }

    return hasTargetFiltered;
}

double LimelightSubsystem::PhotonYaw(){
    double yaw = filteredTarget.GetYaw();
    return yaw;
}

double LimelightSubsystem::FilteredPhotonYaw()
{
    double staticOffset = 4;

    return PhotonYawMap(filteredTarget.GetYaw()) + staticOffset;
    // return OffsetAngle;
}

double LimelightSubsystem::PhotonYawMap(double originalVal)
{
    double convertedVal = 0.0;
    double expoOffsetVal = 0.0;
    double halfFOV = 31.66;
    double A = 5 / (halfFOV * halfFOV);     // 5 is max error at edge of FOV

    expoOffsetVal = A * (originalVal * originalVal);

    if(originalVal >= 0)
    {
        convertedVal = originalVal - expoOffsetVal;
    }
    else if(originalVal < 0)
    {
        convertedVal = originalVal + expoOffsetVal;
    }

    return convertedVal;
}

double LimelightSubsystem::GetNotetx()
{
    return Notetx;
}

double LimelightSubsystem::GetNotety()
{
    return Notety;
}

double LimelightSubsystem::FilteredDistance()
{
    return filteredRange.value();
}

photon::PhotonTrackedTarget LimelightSubsystem::GetFilteredTarget()
{
    return filteredTarget;
}

//still need to do the hold mag seperate from the subsystem, I think in the command.
//pass in the angle trim from the shooter subsystems function get angletrim
double LimelightSubsystem::GetApriltagShooterTheta(double dist, double angleTrim)
{
    if(dist != 0.0)
    {
        frc::SmartDashboard::PutNumber("Distance AprilTag", dist);
        return -0.2351* pow((dist+angleTrim),3) + 4.38 * pow((dist+angleTrim), 2) - 29 * (dist+angleTrim) + 89.64;
    }
    else
    {
        return ShooterConstants::RestingAngle;
    }
}

double LimelightSubsystem::GetApriltagDriveMotorVal(double currentHeading)
{
    if(filteredTargetID == 4 || filteredTargetID == 7)
    {
        txApril = FilteredPhotonYaw();
        desiredHeading = currentHeading + -txApril;//txApril;   // calculated actual angle instead of the error
    }

    driveError = DistanceBetweenAngles(desiredHeading, currentHeading);

    // std::cout << "txApril " << txApril << std::endl; 
    // std::cout << "driveError " << driveError << std::endl;
    // std::cout << "desiredHeading " << desiredHeading << std::endl; 
    // std::cout << "currentHeading " << currentHeading << std::endl; 

    return (driveError * kpApril);
}

double LimelightSubsystem::GetApriltagDriveError()
{
    return driveError;
}

double LimelightSubsystem::DistanceBetweenAngles(double targetAngle, double sourceAngle)
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
