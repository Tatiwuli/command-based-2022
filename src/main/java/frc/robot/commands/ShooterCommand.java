package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {

    private ShooterSubsystem m_shooterSubsystem;
    private ElevatorSubsystem m_elevatorSubsystem;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem) {
        m_shooterSubsystem = shooterSubsystem;
        m_elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        m_shooterSubsystem.shooterStart();
        m_elevatorSubsystem.elevatorStart();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.shooterEnd();
        m_elevatorSubsystem.elevatorEnd();
    }

    @Override
    public boolean isFinished() {
        return m_elevatorSubsystem.cargoDetected();
    }
}
