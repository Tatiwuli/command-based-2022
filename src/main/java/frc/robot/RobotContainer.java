// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

    private final DriveTrainSubsystem m_driveTrainSubsystem = new DriveTrainSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

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

        new JoystickButton(m_stick1, Constants.kButtonIntakeFirstCargo).whileHeld(
                new IntakeCommand(m_intakeSubsystem, m_elevatorSubsystem, true).withTimeout(5));

        new JoystickButton(m_stick1, Constants.kButtonIntake).whileHeld(
                new IntakeCommand(m_intakeSubsystem, m_elevatorSubsystem, false).withTimeout(5));

        new JoystickButton(m_stick1, Constants.kButtonElevator).whileHeld(
                new StartEndCommand(
                        () -> m_elevatorSubsystem.elevatorStart(),
                        () -> m_elevatorSubsystem.elevatorEnd(), m_elevatorSubsystem));

        new JoystickButton(m_stick1, Constants.kButtonReverseElevator).whileHeld(
                new StartEndCommand(
                        () -> m_elevatorSubsystem.elevatorReverseStart(),
                        () -> m_elevatorSubsystem.elevatorEnd(), m_elevatorSubsystem));

    }

    public Command getAutonomousCommand() {
        return null;
    }
}
