package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.FindCargo;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoDriveAndShootCommand extends SequentialCommandGroup {

    public AutoDriveAndShootCommand(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem) {

        addCommands(
            new DriveStraight(-100, driveSubsystem),
            new ShooterCommand(shooterSubsystem, elevatorSubsystem).withTimeout(4)
        );
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
