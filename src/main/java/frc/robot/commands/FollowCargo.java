// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.vision.FollowCargoRunner;

public class FollowCargo extends PIDCommand {

    FollowCargoRunner followCargoRunner;

    public FollowCargo(FollowCargoRunner followCargoRunner, DriveSubsystem drive) {
        super(
                new PIDController(0.08, 0.03, 0.03),
                followCargoRunner::getCenterX,
                0,
                output -> {
                    if (followCargoRunner.detected) drive.arcadeDrive(-0.8, output, 0.6);
                },
                drive);
        this.followCargoRunner = followCargoRunner;
        getController().setTolerance(1, 10);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint() && followCargoRunner.area > 5000;
    }
}
