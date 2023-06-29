#include "subsystems/Compressor.h"

CompressorObject::CompressorObject() {}

// This method will be called once per scheduler run
void CompressorObject::Periodic() {
        if (debugCompressorEnabled){
        bool pressureSwitch = phCompressor.GetPressureSwitchValue();
        isEnabled = phCompressor.IsEnabled(); //checks if the compressor is enabled

        // commented out to test, 2/17
        //frc::SmartDashboard::PutBoolean("Compressor enabled", isEnabled);
        //frc::SmartDashboard::PutBoolean("Compressor pressureSwitch", pressureSwitch);
        //frc::SmartDashboard::PutNumber("Compressor current", current);
    }
}

void CompressorObject::StartCompressor() {
    phCompressor.EnableDigital();
}

void CompressorObject::DisableCompressor() {
    phCompressor.Disable();
}
