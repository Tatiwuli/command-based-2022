package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.FindCargo;
import frc.robot.commands.FollowCargo;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeWithElevatorCommand;
import frc.robot.commands.ShooterWithElevatorCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoDriveShootAndGrabTwoCommand extends SequentialCommandGroup {

    public AutoDriveShootAndGrabTwoCommand(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, 
            ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {

        addCommands(
            new DriveStraight(-100, driveSubsystem),
            new ShooterWithElevatorCommand(shooterSubsystem, elevatorSubsystem),
            new FindCargo(driveSubsystem).withTimeout(3),
            new FollowCargo(driveSubsystem).alongWith(new IntakeCommand(intakeSubsystem)),
                new IntakeWithElevatorCommand(intakeSubsystem, elevatorSubsystem, true)
                        .withTimeout(1),
            new FindCargo(driveSubsystem).withTimeout(3),
            new FollowCargo(driveSubsystem).alongWith(new IntakeCommand(intakeSubsystem)),
                new IntakeWithElevatorCommand(intakeSubsystem, elevatorSubsystem, false)
                        .withTimeout(1));
    }

}
