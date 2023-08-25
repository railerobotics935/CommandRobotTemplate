// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>

#include "Constants.h"

SwerveModule::SwerveModule(int driveMotorPort, 
                        int turningMotorPort,
                        int turningEncoderPort, 
                        double turningEncoderOffset)
    : m_driveMotor(driveMotorPort, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
      m_turningMotor(turningMotorPort, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
      m_driveEncoder(m_driveMotor.GetEncoder()),
      m_turningEncoder(turningEncoderPort){

  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  m_driveEncoder.SetVelocityConversionFactor(ModuleConstants::kDriveEncoderConversionFacotr);
  m_driveEncoder.SetPositionConversionFactor(ModuleConstants::kDriveEncoderConversionFacotr);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(
      units::radian_t{-std::numbers::pi}, units::radian_t{std::numbers::pi});
  
  m_kTurningEncoderOffset = turningEncoderOffset;
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},
          units::radian_t{(m_turningEncoder.GetVoltage() * ModuleConstants::ANALOG_TO_RAD_FACTOR) - m_kTurningEncoderOffset}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {units::meter_t{m_driveEncoder.GetPosition()},
          units::radian_t{(m_turningEncoder.GetVoltage() * ModuleConstants::ANALOG_TO_RAD_FACTOR) - m_kTurningEncoderOffset}};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t{(m_turningEncoder.GetVoltage() * ModuleConstants::ANALOG_TO_RAD_FACTOR) - m_kTurningEncoderOffset});

  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      m_driveEncoder.GetVelocity(), state.speed.value());

  // Calculate the turning motor output from the turning PID controller.
  auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t{(m_turningEncoder.GetVoltage() * ModuleConstants::ANALOG_TO_RAD_FACTOR) - m_kTurningEncoderOffset}, state.angle.Radians());

  // Set the motor outputs.
  m_driveMotor.Set(driveOutput);
  m_turningMotor.Set(turnOutput);
}

// No reason to reset encoders since they are absolute
//void SwerveModule::ResetEncoders() {
//  m_driveEncoder.Reset();
//  m_turningEncoder.Reset();
//}
