#pragma once
#include <string>

class CommandMessenger {
    public:  

        void SetAuxMessage(std::string message)
        {
            AuxMessage = message;
        }

        std::string GetAuxMessage()
        {
            return AuxMessage;
        }

        void SetDriveMessage(std::string message)
        {
            DriveMessage = message;
        }

        std::string GetDriveMessage()
        {
            return DriveMessage;
        }

    private:
        std::string DriveMessage;  
        std::string AuxMessage;
};