// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
    Spark m_left1;
    Spark m_left2;
    Spark m_right1;
    Spark m_right2;

    MotorControllerGroup m_leftMotors;
    MotorControllerGroup m_rightMotors;
    DifferentialDrive drive;

    public DriveTrainSubsystem() {
        m_left1 = new Spark(Constants.kleft1Port);
        m_left2 = new Spark(Constants.kleft2Port);
        m_right1 = new Spark(Constants.kright1Port);
        m_right2 = new Spark(Constants.kright2Port);

        m_leftMotors = new MotorControllerGroup(m_left1, m_left2);
        m_rightMotors = new MotorControllerGroup(m_right1, m_right2);

        m_leftMotors.setInverted(true);
        drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    }

    @Override
    public void periodic() {

    }

    public void setLeftMotors(double speed) {
        m_leftMotors.set(speed);
    }

    public void setRightMotors(double speed) {
        m_rightMotors.set(speed);
    }

    public void drive(double left, double right) {
        drive.tankDrive(left, right);
    }
}
