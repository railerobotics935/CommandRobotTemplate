

#pragma once

#include <frc/Encoder.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>

#include "Constants.h"


// Subsystem for the Arm NOT including the wrist

class ArmSubsystem : frc2::SubsystemBase {
public:

  ArmSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
  /**
   * These will set the motors to either the percent, angle and adjusted angle
  */
  void SetLowerArmMotor(double percent);
  void SetPushRodArmMotor(double percent);
  void SetTurretMotor(double percent);
  void SetIntakeMotor(double percent);
  void SetWristServo(double angle);
  void SetWristAngle(double angle);
  void SetLowerArmAngle(double angle);
  void SetPushRodArmAdjustedAngle(double angle);
  void SetPushRodArmRawAngle(double angle);

  void SetLowerArmMotor(double percent);
  void SetPushRodArmMotor(double percent);
  void SetTurretMotor(double percent);
  void SetIntakeMotor(double percent);
  void SetWristServo(double angle);
  void SetWristAngle(double angle);
  void SetTurretAngle(double angle);
  void SetLowerArmAngle(double angle);
  void SetPushRodArmAdjustedAngle(double angle);
  void SetPushRodArmRawAngle(double angle);

private:
  // Lengths of the parts of the arm in meters
  double lengthOfLowerArm = 0.722;
  double lengthOfPushRodArm = 0.787;
  double lengthOfFullWrist = 0.02815;

  // 3D translation of the center of the Robot(Unsure - was first turret), at the height of the lower arm pivot point.
  double xTranslationOfArm = -0.089;
  double yTranslationOfArm = -0.325;
  double zTranslationOfArm = +0.287;

  // measured offsets DON'T TOUCH
  double turretEncoderOffset = -0.972;
  double lowerArmEncoderOffset = 2.00 + (0.5 * std::numbers::pi); // measured at pi/2
  double pushRodArmEcoderOffset = 3.428 + (0.5 * std::numbers::pi); // measured at pi/2
  double wristServoOffset = 0.46;

  // Arm Limits. the push rod arm is using the raw angle
  double lowerArmLimit = 1.7;
  double pushRodArmLimit = 0.85;
  double turretLimit = (2 * std::numbers::pi / 180.0);
  double wristServoRange = (std::numbers::pi * 3) / 2;
  double idleLowerArmThreshold = 0.19; // in radians
  double idlePushRodArmThreshold = 3.0415; // in radians
  bool lowerArmIdleMode = true;
  bool pushRodArmIdleMode = true;

};