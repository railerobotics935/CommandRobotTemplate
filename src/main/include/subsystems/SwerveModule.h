// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/AnalogInput.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include "ctre/phoenix.h"
#include "rev/CANSparkMax.h"
#include "rev/SparkMaxAnalogSensor.h"

#include "Constants.h"

class SwerveModule {
 public:
  SwerveModule(int driveMotorPort, int turningMotorPort,
                const int turningEncoderPort, const double turningEncoderOffset);

  frc::SwerveModuleState GetState();

  frc::SwerveModulePosition GetPosition();

  void SetDesiredState(const frc::SwerveModuleState& state);

  // void ResetEncoders(); currently using absolute encoders, so you can't reset them digitaly

 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.

  static constexpr auto kModuleMaxAngularVelocity = 18.0 * std::numbers::pi * 1_rad_per_s;  // radians per second
  static constexpr auto kModuleMaxAngularAcceleration = 8.0 * std::numbers::pi * 2_rad_per_s / 1_s;  // radians per second^2
  
  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turningMotor;
  
  rev::SparkMaxRelativeEncoder m_driveEncoder;
  frc::AnalogInput m_turningEncoder;
  
  double m_kTurningEncoderOffset;
  
  frc2::PIDController m_drivePIDController{1.0, 0.0, 0.0 };

  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      12.5, 190.0, 0.15,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};
};
