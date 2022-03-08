package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterWithElevatorCommand extends SequentialCommandGroup {

    public ShooterWithElevatorCommand(ShooterSubsystem shooterSubsystem, 
            ElevatorSubsystem elevatorSubsystem, double prepareTime) {

        addCommands(
            new InstantCommand(() -> shooterSubsystem.shooterStart(), shooterSubsystem),
            new WaitCommand(prepareTime),
            new ShooterCommand(shooterSubsystem, elevatorSubsystem).withTimeout(Constants.Shooter.kShooterTimeout)
        );
    }

    @Override
    public void execute() {
        System.out.println("ShooterWithElevatorCommand");
        super.execute();
    }
}
