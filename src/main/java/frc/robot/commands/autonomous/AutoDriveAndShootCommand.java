package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.ShooterWithElevatorCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoDriveAndShootCommand extends SequentialCommandGroup {

    public AutoDriveAndShootCommand(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, 
            ElevatorSubsystem elevatorSubsystem) {

        addCommands(
            new DriveStraight(Constants.Auto.initialDistance, driveSubsystem).withTimeout(8),
            new ShooterWithElevatorCommand(shooterSubsystem, elevatorSubsystem)
        );
    }
}