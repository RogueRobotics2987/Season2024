#pragma once
#include <vector>
#include <string>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

class AutoPaths {
    public:  

    struct AutoPath
    {
        std::vector<frc::Pose2d> Waypoints; 
        std::vector<units::meters_per_second_t> PointSpeed; 
        std::vector<units::meters_per_second_t> CruiseSpeed; 
        std::vector<std::string> Command;
        std::vector<bool> limelightFollow;
        std::string EndCommand;
    };   
        
    private:
};
