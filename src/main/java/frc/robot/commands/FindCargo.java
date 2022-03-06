// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.vision.FollowCargoRunner;

public class FindCargo extends PIDCommand {
    
    public FindCargo(DriveSubsystem drive) {
        super(
                new PIDController(0.08, 0.03, 0.03),
                Robot.followCargo::getCenterX,
                0,
                output -> {
                    if (Robot.followCargo.detected) {
                        drive.arcadeDrive(0, output, 0.8);
                    } else {
                        drive.arcadeDrive(0, 0.6);
                    }
                },
                drive);
        getController().setTolerance(1, 10);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint() && Robot.followCargo.area > 5000;
    }
}
