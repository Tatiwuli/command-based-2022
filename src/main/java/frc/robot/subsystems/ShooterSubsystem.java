// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class ShooterSubsystem extends SubsystemBase {

    private VictorSP m_motorLeft;
    private VictorSP m_motorRight;

    MotorControllerGroup m_motors;

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {
        m_motorLeft = new VictorSP(Constants.kShooterLeftPort);
        m_motorRight = new VictorSP(Constants.kShooterRightPort);

        m_motors = new MotorControllerGroup(m_motorLeft, m_motorRight);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setMotors(double speed) {
        m_motors.set(speed);
    }

    public void shooterStart() {
        m_motors.set(Constants.kShooterSpeed);
    }

    public void shooterEnd() {
        m_motors.set(0);
    }

    public void shooter(double left, double right) {
        m_motorLeft.set(left);
        m_motorRight.set(right);
    }
}
