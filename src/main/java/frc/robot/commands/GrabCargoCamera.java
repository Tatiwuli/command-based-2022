// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.vision.FollowCargoRunner;

public class GrabCargoCamera extends ParallelCommandGroup {

    // private final FollowCargoRunner followCargoRunner;
    private final IntakeSubsystem m_intakeSubsystem;
    private final ElevatorSubsystem m_elevatorSubsystem;
    
    public GrabCargoCamera(FollowCargoRunner followCargoRunner, DriveSubsystem driveSubsystem, 
            IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) { 
        // this.followCargoRunner = followCargoRunner;
        this.m_intakeSubsystem = intakeSubsystem;
        this.m_elevatorSubsystem = elevatorSubsystem;

        addCommands(
            new FollowCargo(followCargoRunner, driveSubsystem),
            new PrintCommand("GrabCargoCamera")
            // new InstantCommand(
            //     () -> intakeSubsystem.intakeStart(), intakeSubsystem).withTimeout(
            //         Constants.kIntakeTimeout),
            // new InstantCommand(
            //     () -> elevatorSubsystem.elevatorStart(), elevatorSubsystem).withTimeout(
            //         Constants.kIntakeTimeout)
        );

    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        this.m_elevatorSubsystem.elevatorEnd();
        this.m_intakeSubsystem.intakeEnd();
        
    }

    @Override
    public boolean isFinished() {
        return this.m_elevatorSubsystem.cargoDetected();
    }
}
