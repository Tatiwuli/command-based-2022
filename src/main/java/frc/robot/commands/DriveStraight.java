package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraight extends CommandBase {

    private double PWM_CORRECTION_FACTOR_LEFT = 0.8;
    private double PWM_CORRECTION_FACTOR_RIGHT = 0.73;
    private DriveSubsystem m_driveSubsystem;
    private PIDController m_leftPidController;
    private PIDController m_rightPidController;
    private double distance;

    public DriveStraight(double distance, DriveSubsystem driveSubsystem) {
        addRequirements(driveSubsystem);
        this.m_driveSubsystem = driveSubsystem;

        this.distance = distance;
        this.m_leftPidController = new PIDController(0.03, 0, 0);
        this.m_rightPidController = new PIDController(0.05, 0, 0);
        this.m_leftPidController.setTolerance(1);
        this.m_rightPidController.setTolerance(1);
    }
    
    @Override
    public void initialize() {
        this.m_driveSubsystem.reset();
        this.m_leftPidController.setSetpoint(distance);
        this.m_rightPidController.setSetpoint(distance);
    }

    @Override
    public void execute() {
        double leftOutput = this.m_leftPidController.calculate(this.m_driveSubsystem.getLeftDistance());
        double rightOutput = this.m_rightPidController.calculate(this.m_driveSubsystem.getRightDistance());
        m_driveSubsystem.tankDrive(Util.limit(-leftOutput, PWM_CORRECTION_FACTOR_LEFT), Util.limit(-rightOutput, PWM_CORRECTION_FACTOR_RIGHT));
    }

    @Override
    public boolean isFinished() {
        return m_rightPidController.atSetpoint() && m_leftPidController.atSetpoint();
    }
}
