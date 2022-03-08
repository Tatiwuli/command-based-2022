package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class FollowCargo extends PIDCommand {

    boolean stopWithElevator;
    private ElevatorSubsystem m_elevatorSubsystem;

    public FollowCargo(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, 
            boolean keepGoingIfNotDetected, boolean stopWithElevator) {
        super(
                new PIDController(0.08, 0.03, 0.03),
                Robot.followCargoRunner::getCenterX,
                0,
                output -> {
                    if (Robot.followCargoRunner.isDetected()) {
                        driveSubsystem.arcadeDrive(-0.75, output, 0.6);
                    } else if (keepGoingIfNotDetected) {
                        driveSubsystem.tankDrive(-0.65, -0.58);  
                    }
                },
                driveSubsystem);
        this.m_elevatorSubsystem = elevatorSubsystem;
        getController().setTolerance(1, 10);
    }

    public FollowCargo(DriveSubsystem driveSubsystem) {
        this(driveSubsystem, null, true, false);
    }

    public FollowCargo(DriveSubsystem driveSubsystem, boolean keepGoingIfNotDetected) {
        this(driveSubsystem, null, keepGoingIfNotDetected, false);
    }

    @Override
    public boolean isFinished() {
        if (stopWithElevator) {
            return m_elevatorSubsystem.cargoDetected();
        }
        return super.isFinished();
    }

}
