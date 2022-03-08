package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.FindCargo;
import frc.robot.commands.FollowCargo;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.IntakeWithElevatorCommand;
import frc.robot.commands.ShooterWithElevatorCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoDriveShootAndGrabOneCommand extends SequentialCommandGroup {

    public AutoDriveShootAndGrabOneCommand(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, 
            ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {

        addCommands(
            new DriveStraight(Constants.Auto.autoDriveDistance, driveSubsystem)
                    .alongWith(new ShooterCommand(shooterSubsystem, elevatorSubsystem))
                    .alongWith(new IntakeCommand(intakeSubsystem)),
            new ShooterWithElevatorCommand(shooterSubsystem, elevatorSubsystem, 0),
            new FindCargo(driveSubsystem).withTimeout(6),
            new FollowCargo(driveSubsystem, elevatorSubsystem, true, true).alongWith(new IntakeCommand(intakeSubsystem)),
            new IntakeWithElevatorCommand(intakeSubsystem, elevatorSubsystem, false)
                    .withTimeout(6));
    }

}
