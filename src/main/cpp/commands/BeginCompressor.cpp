#include "commands/BeginCompressor.h"

BeginCompressor::BeginCompressor(CompressorObject &compressor) {
  m_compressor = &compressor;
  AddRequirements(m_compressor);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void BeginCompressor::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void BeginCompressor::Execute() {
    // std::cout << "Runing The Compressor" << std::endl;
  m_compressor->StartCompressor();
}

// Called once the command ends or is interrupted.
void BeginCompressor::End(bool interrupted) {}

// Returns true when the command should end.
bool BeginCompressor::IsFinished() { return false; }