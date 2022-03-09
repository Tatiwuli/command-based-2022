package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.PrepareAndShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoDriveShootCommand extends SequentialCommandGroup {

    public AutoDriveShootCommand(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, 
            ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, boolean intakeOn) {

        if (intakeOn) {
            addCommands(
                new DriveStraight(Constants.Auto.autoDriveDistance, driveSubsystem)
                        .alongWith(new IntakeCommand(intakeSubsystem))
                        .alongWith(new ShooterCommand(shooterSubsystem, elevatorSubsystem, false))
                        .withTimeout(5),
                new PrepareAndShootCommand(shooterSubsystem, elevatorSubsystem, 0, 10).withTimeout(10)
            );
        } else {
            addCommands(
                new DriveStraight(Constants.Auto.autoDriveDistance, driveSubsystem)
                    .alongWith(new ShooterCommand(shooterSubsystem, elevatorSubsystem, false))
                    .withTimeout(5),
                new PrepareAndShootCommand(shooterSubsystem, elevatorSubsystem, 0, 10).withTimeout(10)
            );
        }
    }

    public AutoDriveShootCommand(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, 
            ElevatorSubsystem elevatorSubsystem) {
        this(driveSubsystem, shooterSubsystem, elevatorSubsystem, null, false);
    }
}
