package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PrepareAndShootCommand extends SequentialCommandGroup {

    public PrepareAndShootCommand(ShooterSubsystem shooterSubsystem, 
            ElevatorSubsystem elevatorSubsystem, double prepareTime, double shooterTime) {

        addCommands(
            new InstantCommand(() -> shooterSubsystem.shooterStart(), shooterSubsystem),
            new WaitCommand(prepareTime),
            new ShooterCommand(shooterSubsystem, elevatorSubsystem).withTimeout(shooterTime)
        );
    }

    public PrepareAndShootCommand(ShooterSubsystem shooterSubsystem, 
            ElevatorSubsystem elevatorSubsystem, double prepareTime) {
        this(shooterSubsystem, elevatorSubsystem, prepareTime, Constants.Shooter.kShooterTimeout);
    }

    @Override
    public void execute() {
        super.execute();
    }
}
