package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.FindCargo;
import frc.robot.commands.FollowCargo;
import frc.robot.commands.IntakeWithElevatorCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoDriveShootAndGrabTwoCommand extends SequentialCommandGroup {

    public AutoDriveShootAndGrabTwoCommand(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {

        addCommands(
            new DriveStraight(-100, driveSubsystem),
            new ShooterCommand(shooterSubsystem, elevatorSubsystem).withTimeout(4),
            new FindCargo(driveSubsystem).withTimeout(4),
            new FollowCargo(driveSubsystem),
                new IntakeWithElevatorCommand(intakeSubsystem, elevatorSubsystem, true)
                        .withTimeout(Constants.kIntakeTimeout),
            new FindCargo(driveSubsystem).withTimeout(4),
            new FollowCargo(driveSubsystem),
                new IntakeWithElevatorCommand(intakeSubsystem, elevatorSubsystem, true)
                        .withTimeout(Constants.kIntakeTimeout));
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

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
