package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class FollowCargoStopWithElevator extends FollowCargo {

    private ElevatorSubsystem m_elevatorSubsystem;

    public FollowCargoStopWithElevator(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem) {
        super(driveSubsystem);
        this.m_elevatorSubsystem = m_elevatorSubsystem;
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || m_elevatorSubsystem.cargoDetected();
    }
}
