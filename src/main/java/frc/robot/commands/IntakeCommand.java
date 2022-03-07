package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private IntakeSubsystem m_intakeSubsystem;

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
