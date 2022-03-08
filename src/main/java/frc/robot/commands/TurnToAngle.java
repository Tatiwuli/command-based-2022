package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngle extends PIDCommand {

    private DriveSubsystem m_driveSubsystem;

    public TurnToAngle(double targetAngleDegrees, DriveSubsystem driveSubsystem, double forwardVelocity) {
        super(new PIDController(Constants.Drive.kTurnP, Constants.Drive.kTurnI, Constants.Drive.kTurnD),
                driveSubsystem::getHeading,
                targetAngleDegrees,
                output -> driveSubsystem.arcadeDrive(-forwardVelocity, -output),
                driveSubsystem);
        
        this.m_driveSubsystem = driveSubsystem;
        getController().enableContinuousInput(-180, 180);
        getController()
                .setTolerance(Constants.Drive.kToleranceDegrees, Constants.Drive.kTurnRateToleranceDegPerS);
    }

    public TurnToAngle(double targetAngleDegrees, DriveSubsystem drive) {
        this(targetAngleDegrees, drive, 0);
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.tankDrive(0, 0);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
