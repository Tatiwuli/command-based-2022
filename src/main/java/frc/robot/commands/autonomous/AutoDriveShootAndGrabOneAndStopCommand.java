package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.FindCargo;
import frc.robot.commands.GrabCargo;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.PrepareAndShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoDriveShootAndGrabOneAndStopCommand extends SequentialCommandGroup {

    public AutoDriveShootAndGrabOneAndStopCommand(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, 
            ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, boolean useCamera) {

        if (useCamera) {
            addCommands(
                new DriveStraight(Constants.Auto.autoDriveDistance, driveSubsystem)
                        .alongWith(new IntakeCommand(intakeSubsystem))
                        .alongWith(new ShooterCommand(shooterSubsystem, elevatorSubsystem, false))
                        .withTimeout(5),
                new PrepareAndShootCommand(shooterSubsystem, elevatorSubsystem, 0).withTimeout(5),
                new FindCargo(driveSubsystem).withTimeout(6),
                new GrabCargo(driveSubsystem, elevatorSubsystem, intakeSubsystem).withTimeout(10));
        } else {
            addCommands(
                new DriveStraight(Constants.Auto.autoDriveDistance, driveSubsystem)
                        .alongWith(new IntakeCommand(intakeSubsystem)).withTimeout(6),
                new PrepareAndShootCommand(shooterSubsystem, elevatorSubsystem, 0).withTimeout(6));
        }
    }

}
