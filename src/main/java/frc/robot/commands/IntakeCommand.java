// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private IntakeSubsystem m_intakeSubsystem;
    private boolean firstCargo;

    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.m_intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.intakeStart();
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.intakeEnd();
    }
}
