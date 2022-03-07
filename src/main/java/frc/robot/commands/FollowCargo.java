package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class FollowCargo extends PIDCommand {

    public FollowCargo(DriveSubsystem driveSubsystem, boolean keepGoingIfNotDetected) {
        super(
                new PIDController(0.08, 0.03, 0.03),
                Robot.followCargoRunner::getCenterX,
                0,
                output -> {
                    System.out.println("output");
                    if (Robot.followCargoRunner.isDetected()) {
                        driveSubsystem.arcadeDrive(-0.75, output, 0.6);
                    } else if (keepGoingIfNotDetected) {
                        driveSubsystem.tankDrive(-0.65, -0.58);  
                    }
                },
                driveSubsystem);
        getController().setTolerance(1, 10);
    }

    public FollowCargo(DriveSubsystem driveSubsystem) {
        this(driveSubsystem, true);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint() && Robot.followCargoRunner.getArea() > 5000;
    }
}
