// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    private VictorSP m_motor;

    AnalogInput sensorElevator = new AnalogInput(0);

    public ElevatorSubsystem() {
        m_motor = new VictorSP(Constants.kElevatorPort);
    }

    public boolean cargoDetected() {
        if (sensorElevator.getAverageValue() > 4000) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        // System.out.println(sensorElevator.getAverageValue());
    }

    public void elevatorStart() {
        m_motor.set(Constants.kElevatorSpeed);
    }

    public void elevatorReverseStart() {
        m_motor.set(-Constants.kElevatorSpeed);
    }

    public void elevatorEnd() {
        m_motor.set(0);
    }
}
