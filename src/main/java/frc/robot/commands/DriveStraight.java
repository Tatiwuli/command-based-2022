// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraight extends CommandBase {

    private DriveSubsystem m_driveSubsystem;
    private PIDController m_leftPidController;
    private PIDController m_rightPidController;
    private double distance;

    public DriveStraight(double distance, DriveSubsystem driveSubsystem) {
        addRequirements(driveSubsystem);
        this.m_driveSubsystem = driveSubsystem;

        this.m_driveSubsystem.reset();
        this.distance = distance;
        this.m_leftPidController = new PIDController(0.03, 0, 0);
        this.m_rightPidController = new PIDController(0.05, 0, 0);
        this.m_leftPidController.setTolerance(1);
        this.m_rightPidController.setTolerance(1);
    }

    @Override
    public void initialize() {
        this.m_leftPidController.setSetpoint(distance);
        this.m_rightPidController.setSetpoint(distance);
    }

    @Override
    public void execute() {
        System.out.println("Executando drive straight");
        double leftOutput = this.m_leftPidController.calculate(this.m_driveSubsystem.getLeftDistance());
        double rightOutput = this.m_rightPidController.calculate(this.m_driveSubsystem.getRightDistance());
        m_driveSubsystem.tankDrive(Util.limit(-leftOutput, 0.8), Util.limit(-rightOutput, 0.73));
        // m_driveSubsystem.tankDrive(Util.limit(-leftOutput, 0.9), Util.limit(-rightOutput, 0.84));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return m_rightPidController.atSetpoint() && m_leftPidController.atSetpoint();
    }
}
