// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private IntakeSubsystem m_intakeSubsystem;
    private ElevatorSubsystem m_elevatorSubsystem;
    private boolean firstCargo;

    /** Creates a new IntakeCommand. */
    public IntakeCommand(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, boolean firstCargo) {
        m_intakeSubsystem = intakeSubsystem;
        m_elevatorSubsystem = elevatorSubsystem;
        firstCargo = firstCargo;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intakeSubsystem.intakeStart();
        m_elevatorSubsystem.elevatorStart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return firstCargo && m_elevatorSubsystem.cargoDetect();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.intakeEnd();
        m_elevatorSubsystem.elevatorEnd();
    }
}
