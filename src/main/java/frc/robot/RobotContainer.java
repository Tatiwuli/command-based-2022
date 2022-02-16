// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveTrainSubsystem;

public class RobotContainer {
    private final DriveTrainSubsystem m_driveTrainSubsystem = new DriveTrainSubsystem();
    private final Joystick m_stick1 = new Joystick(0);
    private final Joystick m_stick2 = new Joystick(0);

    public RobotContainer() {
        m_stick2.setYChannel(5);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        m_driveTrainSubsystem.setDefaultCommand(new RunCommand(
                () -> m_driveTrainSubsystem.drive(m_stick1.getY(), m_stick2.getY()),
                m_driveTrainSubsystem));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
