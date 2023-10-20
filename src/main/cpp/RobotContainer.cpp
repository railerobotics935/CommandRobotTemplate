// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc2/command/button/JoystickButton.h>

#include "pathplanner/lib/path/PathPlannerPath.h"
#include "pathplanner/lib/auto/AutoBuilder.h"
#include "pathplanner/lib/commands/PathPlannerAuto.h"

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;
using namespace pathplanner;

RobotContainer::RobotContainer() {
    // Initialize all of your commands and subsystems here

    // Configure the button bindings
    ConfigureButtonBindings();

    // Set up default drive command
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    m_drive.SetDefaultCommand(frc2::RunCommand(
        [this] {
            m_drive.Drive(
                units::meters_per_second_t{m_driveController.GetRawAxis(ControllerConstants::kDriveLeftYIndex)},
                units::meters_per_second_t{m_driveController.GetRawAxis(ControllerConstants::kDriveLeftXIndex)},
                units::radians_per_second_t{m_driveController.GetRawAxis(ControllerConstants::kDriveRightXIndex)}, 
                m_driveController.GetRawButton(ControllerConstants::kFieldRelativeSwitchIndex));
        },
        {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
    
    frc2::JoystickButton resetButton(&m_driveController, ControllerConstants::kResetGyroButtonIndex); // Creates a new JoystickButton object for the "reset" button on Drive Controller    
    frc2::JoystickButton slowSwitch(&m_driveController, ControllerConstants::kSlowStateSwitchIndex); // Creates a new JoystickButton object for the slow switch on Drive Controller    
    frc2::JoystickButton parkSwitch(&m_driveController, ControllerConstants::kParkSwitchIndex); // Creates a new JoystickButton object for the brake switch on Drive Controller    

    // I don't exactly know why this works, but the documentation for command based c++ is kinda bad 
    resetButton.OnTrue(frc2::cmd::Run([&] {m_drive.ZeroHeading();}, {&m_drive}));
    slowSwitch.WhileTrue(frc2::cmd::Run([&] {            
            m_drive.Drive(
                (0.5 * units::meters_per_second_t{m_driveController.GetRawAxis(ControllerConstants::kDriveLeftYIndex)}),
                (0.5 * units::meters_per_second_t{m_driveController.GetRawAxis(ControllerConstants::kDriveLeftXIndex)}),
                (0.5 *  units::radians_per_second_t{m_driveController.GetRawAxis(ControllerConstants::kDriveRightXIndex)}), 
                m_driveController.GetRawButton(ControllerConstants::kFieldRelativeSwitchIndex)); 
        }, 
        {&m_drive}));
    parkSwitch.WhileTrue(frc2::cmd::Run([&] {m_drive.Park();}, {&m_drive}));
}

// TODO: Update Path planner. It keeps geting updated and has refined the system to make alot more sence
frc2::Command* RobotContainer::GetAutonomousCommand() {
  //frc2::CommandPtr m_command = PathPlannerAuto("ExampleAuto1").ToPtr();
  //return m_command.get();
}
