package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.FindCargo;
import frc.robot.commands.FollowCargo;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterWithElevatorCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoDriveShootAndGrabOneAndStopCommand extends SequentialCommandGroup {

    public AutoDriveShootAndGrabOneAndStopCommand(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, 
            ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, boolean useCamera) {

        if (useCamera) {
            addCommands(
                new DriveStraight(Constants.Auto.initialDistance, driveSubsystem),
                new ShooterWithElevatorCommand(shooterSubsystem, elevatorSubsystem),
                new FindCargo(driveSubsystem).withTimeout(6),
                new FollowCargo(driveSubsystem).alongWith(new IntakeCommand(intakeSubsystem)).withTimeout(3));
        } else {
            addCommands(
                new DriveStraight(Constants.Auto.initialDistance, driveSubsystem).alongWith(new IntakeCommand(intakeSubsystem)),
                new ShooterWithElevatorCommand(shooterSubsystem, elevatorSubsystem));
        }
    }

}
