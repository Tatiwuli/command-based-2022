package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeWithElevatorCommand extends CommandBase {

    private IntakeSubsystem m_intakeSubsystem;
    private ElevatorSubsystem m_elevatorSubsystem;
    private boolean firstCargo;

    public IntakeWithElevatorCommand(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, boolean firstCargo) {
        this.m_intakeSubsystem = intakeSubsystem;
        this.m_elevatorSubsystem = elevatorSubsystem;
        this.firstCargo = firstCargo;
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.intakeStart();
        m_elevatorSubsystem.elevatorStart();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return firstCargo && m_elevatorSubsystem.cargoDetected();
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.intakeEnd();
        m_elevatorSubsystem.elevatorEnd();
    }
}
