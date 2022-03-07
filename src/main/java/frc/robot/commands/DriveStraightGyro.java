package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraightGyro extends PIDCommand {

    private DriveSubsystem m_driveSubsystem;

    public DriveStraightGyro(DriveSubsystem driveSubsystem, double forwardVelocity) {

        super(new PIDController(Constants.Drive.kTurnP, Constants.Drive.kTurnI, Constants.Drive.kTurnD),
                driveSubsystem::getHeading,
                () -> driveSubsystem.getForwardOrient(),
                // () -> driveSubsystem.getHeading(),
                output -> driveSubsystem.arcadeDrive(-forwardVelocity, -output),
                driveSubsystem);
            
        this.m_driveSubsystem = driveSubsystem;
        getController().enableContinuousInput(-180, 180);
        getController()
                .setTolerance(Constants.Drive.kToleranceDegrees, Constants.Drive.kTurnRateToleranceDegPerS);
    }

    @Override
    public void end(boolean interrupted) {
        // m_driveSubsystem.resetForwardOrient();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
