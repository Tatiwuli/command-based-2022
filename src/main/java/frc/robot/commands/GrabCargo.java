package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class GrabCargo extends ParallelCommandGroup {

    private ElevatorSubsystem m_elevatorSubsystem;
    private boolean m_useElevator = true;

    public GrabCargo(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem,
            IntakeSubsystem intakeSubsystem, boolean useElevator) {
        this.m_useElevator = useElevator;
        if (m_useElevator) {
            addCommands(
                new FollowCargo(driveSubsystem, elevatorSubsystem, false, true),
                new IntakeWithElevatorCommand(intakeSubsystem, elevatorSubsystem, true)
            );
            this.m_elevatorSubsystem = elevatorSubsystem;
        } else {
            addCommands(
                new FollowCargo(driveSubsystem, elevatorSubsystem, false, true),
                new IntakeCommand(intakeSubsystem)
            );
        }
    }

    public GrabCargo(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem,
            IntakeSubsystem intakeSubsystem) {
        this(driveSubsystem, elevatorSubsystem, intakeSubsystem, true);
    }

    @Override
    public void execute() {
        System.out.println("GrabCargo");
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        if (this.m_useElevator) {
            this.m_elevatorSubsystem.elevatorEnd();
        }
        super.end(interrupted);
    }
}
