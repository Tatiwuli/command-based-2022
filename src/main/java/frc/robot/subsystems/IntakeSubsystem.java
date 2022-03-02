// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private VictorSP m_motor;

    public IntakeSubsystem() {
        m_motor = new VictorSP(Constants.kIntakePort);
        m_motor.setInverted(true);
    }

    public void intakeStart() {
        m_motor.set(Constants.kIntakeSpeed);
    }

    public void intakeEnd() {
        m_motor.set(0);
    }
}
