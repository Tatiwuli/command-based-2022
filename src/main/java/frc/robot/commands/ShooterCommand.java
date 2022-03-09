package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {

    // NOTE: THIS COMMAND DOES NOT RUN ALONE. Please use PrepareAndShootCommand instead.

    private ShooterSubsystem m_shooterSubsystem;
    private ElevatorSubsystem m_elevatorSubsystem;
    private boolean m_activateElevator;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem, boolean activateElevator) {
        this.m_shooterSubsystem = shooterSubsystem;
        this.m_elevatorSubsystem = elevatorSubsystem;
        this.m_activateElevator = activateElevator;
    }
    
    public ShooterCommand(ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this(shooterSubsystem, elevatorSubsystem, true);
    }
    
    @Override
    public void initialize() {
        m_shooterSubsystem.shooterStart();
        if (m_activateElevator) {
            m_elevatorSubsystem.elevatorStart();
        } 
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
