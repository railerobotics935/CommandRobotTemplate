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

namespace DriveConstants {

// CAN Sparkmax id numbers
constexpr int kFrontLeftDriveMotorPort = 11;
constexpr int kRearLeftDriveMotorPort = 9;
constexpr int kFrontRightDriveMotorPort = 19;
constexpr int kRearRightDriveMotorPort = 21;

constexpr int kFrontLeftTurningMotorPort = 12;
constexpr int kRearLeftTurningMotorPort = 10;
constexpr int kFrontRightTurningMotorPort = 20;
constexpr int kRearRightTurningMotorPort = 2;

// Anolog input ports on roborio
constexpr int kFrontLeftTurningEncoderPort = 0;
constexpr int kRearLeftTurningEncoderPort = 1;
constexpr int kFrontRightTurningEncoderPort = 2;
constexpr int kRearRightTurningEncoderPort = 3;

// Offsets in radians for the encoders. the first number to to make zero forward, after that we
// subtract an additional pi to make the full range -pi to pi instead of 0 to 2pi
constexpr double kFrontLeftDriveEncoderOffset = 1.113 - std::numbers::pi;
constexpr double kRearLeftDriveEncoderOffset = 2.787 - std::numbers::pi;
constexpr double kFrontRightDriveEncoderOffset = -0.155 - std::numbers::pi;
constexpr double kRearRightDriveEncoderOffset = -1.850 - std::numbers::pi;

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

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace ControllerContstants{

// Controller Constants for the flight elite drive controller

// Axis indexes
constexpr int kDriveLeftYIndex = 2; // An imput UP creates a NEGATIVE output
constexpr int kDriveLeftXIndex = 4; // An imput RIGHT creates a NEGATIVE output
constexpr int kDriveRightYIndex = 1; // An imput UP creates a NEGATIVE output
constexpr int kDriveRightXIndex = 0; // An imput RIGHT creates a NEGATIVE output

// Button/Swtich indexes
constexpr int kFeildReletiveSwitchIndex = 0;
constexpr int kParkSwitchIndex = 1;
constexpr int kSlowStateSwtichIndex = 4;
constexpr int kResetGyroButtonIndex = 2;

} // namespace ControllerContstants

namespace IOConstants {
constexpr int kDriverControllerPort = 0;
}  // namespace OIConstants
