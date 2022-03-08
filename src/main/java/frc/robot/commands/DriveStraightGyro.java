package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Util;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraightGyro extends CommandBase {

    private DriveSubsystem m_driveSubsystem;
    private PIDController m_turnPidController;
    private double m_velocity;

    public DriveStraightGyro(DriveSubsystem driveSubsystem, double velocity) {
        addRequirements(driveSubsystem);
        this.m_driveSubsystem = driveSubsystem;
        this.m_velocity = velocity;
        this.m_turnPidController = new PIDController(Constants.Drive.kTurnP, Constants.Drive.kTurnI, 
                Constants.Drive.kTurnD);
        this.m_turnPidController.setTolerance(5);
    }

    public DriveStraightGyro(DriveSubsystem driveSubsystem) {
        this(driveSubsystem, 0.7);
    }
    
    @Override
    public void initialize() {
        this.m_driveSubsystem.resetGyro();
        this.m_turnPidController.setSetpoint(m_driveSubsystem.getHeading());
    }

    @Override
    public void execute() {
        double output = this.m_turnPidController.calculate(this.m_driveSubsystem.getHeading());
        m_driveSubsystem.arcadeDrive(-this.m_velocity, -output);
    }
}
