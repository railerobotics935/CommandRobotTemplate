// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

// NOTE: These numbers are still NOT ACCURATE these numbers need to be calibrated to the robot being used
// current Configuration is being aimed for 2023 robot Karrot
namespace DriveConstants {
constexpr int kFrontLeftDriveMotorPort = 0;
constexpr int kRearLeftDriveMotorPort = 2;
constexpr int kFrontRightDriveMotorPort = 4;
constexpr int kRearRightDriveMotorPort = 6;

constexpr int kFrontLeftTurningMotorPort = 1;
constexpr int kRearLeftTurningMotorPort = 3;
constexpr int kFrontRightTurningMotorPort = 5;
constexpr int kRearRightTurningMotorPort = 7;

constexpr int kFrontLeftTurningEncoderPort = 1;
constexpr int kRearLeftTurningEncoderPort = 3;
constexpr int kFrontRightTurningEncoderPort = 5;
constexpr int kRearRightTurningEncoderPort = 7;

constexpr int kFrontLeftDriveEncoderPort = 9;
constexpr int kRearLeftDriveEncoderPort = 11;
constexpr int kFrontRightDriveEncoderPort = 13;
constexpr int kRearRightDriveEncoderPort = 15;

constexpr bool kFrontLeftDriveEncoderReversed = false;
constexpr bool kRearLeftDriveEncoderReversed = true;
constexpr bool kFrontRightDriveEncoderReversed = false;
constexpr bool kRearRightDriveEncoderReversed = true;

// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
// These characterization values MUST be determined either experimentally or
// theoretically for *your* robot's drive. The SysId tool provides a convenient
// method for obtaining these values for your robot.
constexpr auto ks = 1_V;
constexpr auto kv = 0.8 * 1_V * 1_s / 1_m;
constexpr auto ka = 0.15 * 1_V * 1_s * 1_s / 1_m;

// Example value only - as above, this must be tuned for your drive!
constexpr double kPFrontLeftVel = 0.5;
constexpr double kPRearLeftVel = 0.5;
constexpr double kPFrontRightVel = 0.5;
constexpr double kPRearRightVel = 0.5;
}  // namespace DriveConstants

namespace ModuleConstants {
// This is something to try and get rid of
// If feels like there should be something like this built into the systems
constexpr double ANALOG_TO_RAD_FACTOR = 1.2566;     // 0 to 5.0 volt = 2PI rad
constexpr double SPARK_MAX_ANALOG_TO_RAD_FACTOR = 1.9040;     // 0 to 3.3 volt = 2PI rad

constexpr int kEncoderCPR = 1024;
constexpr double kWheelDiameterMeters = 0.15;
constexpr int kEncoderResolution = 42;
constexpr double kGearRatio = 6.75;

constexpr double kDriveEncoderConversionFacotr = (2.0 * std::numbers::pi * kWheelDiameterMeters 
                                                / (kGearRatio * kEncoderResolution));

constexpr double kPModuleTurningController = 1;
constexpr double kPModuleDriveController = 1;
}  // namespace ModuleConstants

namespace AutoConstants {

constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

//

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
}  // namespace OIConstants
