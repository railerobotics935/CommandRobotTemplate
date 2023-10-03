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

#include "pathplanner/lib/auto/SwerveAutoBuilder.h"
#include "pathplanner/lib/PathPlanner.h"

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
                units::meters_per_second_t{m_driveController.GetRawAxis(ControllerContstants::kDriveLeftYIndex)},
                units::meters_per_second_t{m_driveController.GetRawAxis(ControllerContstants::kDriveLeftXIndex)},
                units::radians_per_second_t{m_driveController.GetRawAxis(ControllerContstants::kDriveRightXIndex)}, 
                m_driveController.GetRawButton(ControllerContstants::kFieldRelativeSwitchIndex));
        },
        {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
    
    frc2::JoystickButton resetButton(&m_driveController, ControllerContstants::kResetGyroButtonIndex); // Creates a new JoystickButton object for the "reset" button on Drive Controller    
    frc2::JoystickButton slowSwitch(&m_driveController, ControllerContstants::kSlowStateSwtichIndex); // Creates a new JoystickButton object for the slow switch on Drive Controller    

    // I don't exactly know why this works, but the documentation for command based c++ is kinda bad 
    resetButton.OnTrue(frc2::cmd::Run([&] {m_drive.ZeroHeading();}, {&m_drive}));
    slowSwitch.OnTrue(frc2::cmd::Run([&] {            
            m_drive.Drive(
                (0.5 * units::meters_per_second_t{m_driveController.GetRawAxis(ControllerContstants::kDriveLeftYIndex)}),
                (0.5 * units::meters_per_second_t{m_driveController.GetRawAxis(ControllerContstants::kDriveLeftXIndex)}),
                (0.5 *  units::radians_per_second_t{m_driveController.GetRawAxis(ControllerContstants::kDriveRightXIndex)}), 
                m_driveController.GetRawButton(ControllerContstants::kFieldRelativeSwitchIndex)); 
        }, 
        {&m_drive}));

}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    /**
     * CURENTLY DOES NOT RETURN ANYTHING
    */

    // Set up config for trajectory
    frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                                AutoConstants::kMaxAcceleration);
    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(m_drive.kDriveKinematics);


    std::vector<PathPlannerTrajectory> pathGroup = PathPlanner::loadPathGroup("ExampleAuto1", {PathConstraints(4_mps, 3_mps_sq)});


    frc::ProfiledPIDController<units::radians> thetaController{
        AutoConstants::kPThetaController, 0, 0,
        AutoConstants::kThetaControllerConstraints};

    thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                            units::radian_t{std::numbers::pi});

    std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

    // Swerve Command builder for pathplanner
    SwerveAutoBuilder autoBuilder(
        [this]() { return m_drive.GetPose(); }, // Function to supply current robot pose
        [this](auto initPose) { m_drive.ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
        m_drive.kDriveKinematics,
        PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        [this](auto states) { m_drive.SetModuleStates(states); }, // Output function that accepts field relative ChassisSpeeds
        eventMap, // Our event map
        { &m_drive }, // Drive requirements, usually just a single drive subsystem
        true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    );

    frc2::CommandPtr fullAuto = autoBuilder.fullAuto(pathGroup);
/*
frc2::SwerveControllerCommand<4> swerveControllerCommand(
    exampleTrajectory, [this]() { return m_drive.GetPose(); },

    m_drive.kDriveKinematics,

    frc2::PIDController{AutoConstants::kPXController, 0, 0},
    frc2::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

    [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

    {&m_drive});

// Reset odometry to the starting pose of the trajectory.
m_drive.ResetOdometry(exampleTrajectory.InitialPose());

// no auto
return new frc2::SequentialCommandGroup(
    std::move(swerveControllerCommand),
    frc2::InstantCommand(
        [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); }, {}));

        */
}
