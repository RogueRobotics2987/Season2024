#include "subsystems/ShooterSubsystem.h"


ShooterSubsystem::ShooterSubsystem(){
ShooterSolenoid1 = new frc::DoubleSolenoid(frc::PneumaticsModuleType::REVPH, 0, 1); 
ShooterSolenoid2 = new frc::DoubleSolenoid(frc::PneumaticsModuleType::REVPH, 2, 3); 
ShooterSolenoid3 = new frc::DoubleSolenoid(frc::PneumaticsModuleType::REVPH, 4, 5); 
ShooterSolenoid4 = new frc::DoubleSolenoid(frc::PneumaticsModuleType::REVPH, 6, 7); 
ShooterSolenoid5 = new frc::DoubleSolenoid(frc::PneumaticsModuleType::REVPH, 8, 9); 
}

void ShooterSubsystem::Periodic() {}
void ShooterSubsystem::Close(int SolenoidNum){
    

    if(SolenoidNum==1){
        std::cout << "Solenoid 1 is closing" << std::endl;
        ShooterSolenoid1->Set(frc::DoubleSolenoid::kForward);
    } else if(SolenoidNum==2){
        std::cout << "Solenoid 2 is closing" << std::endl;
        ShooterSolenoid2->Set(frc::DoubleSolenoid::kForward);
    } else if(SolenoidNum==3){
        std::cout << "Solenoid 3 is closing" << std::endl;
        ShooterSolenoid3->Set(frc::DoubleSolenoid::kForward);
    } else if(SolenoidNum==4){
        std::cout << "Solenoid 4 is closing" << std::endl;
        ShooterSolenoid4->Set(frc::DoubleSolenoid::kForward);
    } else if(SolenoidNum==5){
        std::cout << "Solenoid 5 is closing" << std::endl;
        ShooterSolenoid5->Set(frc::DoubleSolenoid::kForward);
    }
}

void ShooterSubsystem::Open(int SolenoidNum){ 

    if(SolenoidNum==1){
        ShooterSolenoid1->Set(frc::DoubleSolenoid::kReverse);
    } else if(SolenoidNum==2){
        ShooterSolenoid2->Set(frc::DoubleSolenoid::kReverse);
    } else if(SolenoidNum==3){
        ShooterSolenoid3->Set(frc::DoubleSolenoid::kReverse);
    } else if(SolenoidNum==4){
        ShooterSolenoid4->Set(frc::DoubleSolenoid::kReverse);
    } else if(SolenoidNum==5){
        ShooterSolenoid5->Set(frc::DoubleSolenoid::kReverse);
    }
}

ShooterSubsystem::~ShooterSubsystem(){
    delete ShooterSolenoid1;
    delete ShooterSolenoid2;
    delete ShooterSolenoid3;
    delete ShooterSolenoid4;
    delete ShooterSolenoid5;
}

