// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LimelightSubsystem.h"

LimelightSubsystem::LimelightSubsystem() = default;

// This method will be called once per scheduler run
void LimelightSubsystem::Periodic()
{
    targetIDs.clear();
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

    if(hasTarget == true)
    {
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

    if(DebugConstants::debugLimelight == true)
    {
        frc::SmartDashboard::PutNumber("FilteredRange", filteredRange.value());
        frc::SmartDashboard::PutNumber("FilteredYaw", filteredTarget.GetYaw());
        frc::SmartDashboard::PutNumber("FilteredPitch", filteredTarget.GetPitch());
        frc::SmartDashboard::PutNumber("FilteredID", filteredTargetID);
    }
}

double LimelightSubsystem::GetAmptx()
{
    if(filteredTargetID == 5)
    {
        return PhotonYaw();
    } 
    else 
    {
        return 0;
    }
}

bool LimelightSubsystem::PhotonHasTarget()
{
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

double LimelightSubsystem::PhotonYaw()
{
    double yaw = filteredTarget.GetYaw();
    return yaw;
}

double LimelightSubsystem::FilteredPhotonYaw()
{
    double staticOffset = 2;

    return PhotonYawMap(filteredTarget.GetYaw()) + staticOffset;
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
    double recalcDist = sqrt((x2 * x2) + (y2 * y2));

    // if(dist != 0.0)
    // {
    //     frc::SmartDashboard::PutNumber("Distance AprilTag", dist);
    //     return -0.2351* pow((dist+angleTrim),3) + 4.38 * pow((dist+angleTrim), 2) - 29 * (dist+angleTrim) + 89.64;
    // }
    if(recalcDist != 0.0)
    {
        frc::SmartDashboard::PutNumber("Distance AprilTag", recalcDist);
        return -0.2351* pow((recalcDist + angleTrim),3) + 4.38 * pow((recalcDist + angleTrim), 2) - 29 * (recalcDist + angleTrim) + 89.64;
    }
    else
    {
        return ShooterConstants::RestingAngle;
    }
}

double LimelightSubsystem::GetApriltagDriveMotorVal(double currentHeading, double currX, double currY)
{
    double dist = FilteredDistance();

    if(filteredTargetID == 4 || filteredTargetID == 7)
    {
        txApril = FilteredPhotonYaw();
        desiredHeading = currentHeading + -txApril; // calculated actual angle instead of the error

        recalcDeltaX = 0.0;
        recalcDeltaY = 0.0;

        prevX = currX;
        prevY = currY;

        recalcX1 = sin(txApril * M_PI / 180) * dist; 
        recalcY1 = cos(txApril * M_PI / 180) * dist;

        // if(filteredTargetID == 4)
        // {
        //     recalcX1 = 16.5 - currX;    // (red side) ID 4 is at pose (16.5, 5.5)
        //     recalcY1 = 5.5 - currY;
        // }
        // else if(filteredTargetID == 7)
        // {
        //     recalcX1 = 0 - currX;       // (blue side) ID 7 is at pose (0, 5.5)
        //     recalcY1 = 5.5 - currY;
        // }
    }

    recalcDeltaX = currX - prevX;
    recalcDeltaY = currY - prevY;

    x2 = recalcX1 - recalcDeltaX;
    y2 = recalcY1 - recalcDeltaY;


    desiredHeading = atan2(x2, y2) * 180 / M_PI;

    driveError = DistanceBetweenAngles(desiredHeading, currentHeading);

    prevX = currX;
    prevY = currY;
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
