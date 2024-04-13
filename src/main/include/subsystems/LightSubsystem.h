#pragma once
#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalOutput.h>
//Roborio DIO port 0 = orange, 1 = blue, 2 = white
//Arduino DIO port 2 = orange, 3 = blue, 4 = white

class LightSubsystem : public frc2::SubsystemBase 
{
    public:
        LightSubsystem();

        void Periodic() override;
        void SetLightsGreen();
        void SetLightsYellow();
        void SetLightsPurple();
        void SetNoColor();
        void SetColorChase();
        void SetFlashPurple();
        void SetLightsInitial();


    private:

    frc::DigitalOutput pin2 {0};
    frc::DigitalOutput pin3 {1};
    frc::DigitalOutput pin4 {2};
};