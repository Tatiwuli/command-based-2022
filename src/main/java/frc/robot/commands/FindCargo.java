package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class FindCargo extends PIDCommand {
    
    public FindCargo(DriveSubsystem drive) {
        super(
                new PIDController(0.03, 0.02, 0.02),
                Robot.followCargoRunner::getCenterX,
                0,
                output -> {
                    if (Robot.followCargoRunner.isDetected()) {
                        drive.arcadeDrive(0, output, 0.6);
                    } else {
                        drive.tankDrive(0.55, -0.55);
                    }
                },
                drive);
        getController().setTolerance(1, 30);
    }
    
    @Override
    public void execute() {
        System.out.println("FindCargo");
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return Robot.followCargoRunner.isDetected();
    }
}
