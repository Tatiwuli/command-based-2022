// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngle extends PIDCommand {

    public TurnToAngle(double targetAngleDegrees, DriveSubsystem drive, boolean resetGyro, double forwardVelocity) {
        super(new PIDController(Constants.Drive.kTurnP, Constants.Drive.kTurnI, Constants.Drive.kTurnD),
                drive::getHeading,
                targetAngleDegrees,
                output -> drive.arcadeDrive(-forwardVelocity, -output),
                drive);

        if (resetGyro)
            drive.resetGyro();
        getController().enableContinuousInput(-180, 180);
        getController()
                .setTolerance(Constants.Drive.kToleranceDegrees, Constants.Drive.kTurnRateToleranceDegPerS);
    }

    public TurnToAngle(double targetAngleDegrees, DriveSubsystem drive, boolean resetGyro) {
        this(targetAngleDegrees, drive, resetGyro, 0);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
