#include "subsystems/ActuatorSubsystem.h"

ActuatorSubsystem::ActuatorSubsystem(){
ActuatorMotor = new rev::CANSparkMax(m_MotorController, rev::CANSparkMax::MotorType::kBrushless);
ActuatorMotor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
// ActuatorMotor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, 5000); // TODO Change limit number
m_limitSwitch = new rev::SparkMaxLimitSwitch(ActuatorMotor->GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed));
}

void ActuatorSubsystem::Periodic() {
    ForwardLimitSwitch = m_limitSwitch->Get();
    frc::SmartDashboard::PutBoolean("Forward Limit Switch", ForwardLimitSwitch);
    // frc::SmartDashboard::PutBoolean("Reverse Limit Switch", );
}

void ActuatorSubsystem::Retract(){
 ActuatorMotor->SetVoltage(units::voltage::volt_t(2.0));
//  std::cout << "I'm Trying to Retract!" << std::endl;
    
}

void ActuatorSubsystem::Extend(){    
ForwardLimitSwitch = m_limitSwitch->Get();

 ActuatorMotor->SetVoltage(units::voltage::volt_t(-3.0));
  // std::cout << "I'm Trying to Extend!" << std::endl;
}

void ActuatorSubsystem::Neutral(){
ForwardLimitSwitch = m_limitSwitch->Get();   
  // std::cout << "I'm Neutral!" << std::endl;
 ActuatorMotor->SetVoltage(units::voltage::volt_t(0));
}

bool ActuatorSubsystem::GetForwardLimitSwitch(){ // Gets the bottom limit switch state
    return ForwardLimitSwitch;
}

rev::SparkMaxRelativeEncoder ActuatorSubsystem::GetEncoder(rev::SparkMaxRelativeEncoder::Type sensorType, int counts_per_rev){
    return ActuatorMotor->GetEncoder(sensorType, counts_per_rev = counts_per_rev); 
}

ActuatorSubsystem::~ActuatorSubsystem(){
    delete ActuatorMotor;
    delete m_limitSwitch;
}