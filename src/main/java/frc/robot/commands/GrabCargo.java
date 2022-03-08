package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class GrabCargo extends ParallelCommandGroup {

    private ElevatorSubsystem m_elevatorSubsystem;

    public GrabCargo(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem,
            IntakeSubsystem intakeSubsystem) {
        addCommands(
            new FollowCargo(driveSubsystem, elevatorSubsystem, false, true),
            new IntakeWithElevatorCommand(intakeSubsystem, elevatorSubsystem, true)
        );
        this.m_elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void execute() {
        System.out.println("GrabCargo");
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        this.m_elevatorSubsystem.elevatorEnd();
        super.end(interrupted);
    }
}
