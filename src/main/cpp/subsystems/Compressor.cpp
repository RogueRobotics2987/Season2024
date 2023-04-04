#include "subsystems/Compressor.h"

CompressorObject::CompressorObject() {
    m_compressor = new frc::Compressor(frc::PneumaticsModuleType::REVPH);
    std::cout << "Created a Copressor Object" << std::endl;
}
CompressorObject::~CompressorObject(){
    // m_compressor->Stop();
    delete m_compressor;
}

// This method will be called once per scheduler run
void CompressorObject::Periodic() {}

void CompressorObject::startCompressor() {
    // m_compressor->Start();
}
