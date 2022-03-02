// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    private final DriveTrainSubsystem m_driveTrainSubsystem = new DriveTrainSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    
    private final Joystick m_stick1Left = new Joystick(0);
    private final Joystick m_stick1Right = new Joystick(0);
    private final Joystick m_stickClimb = new Joystick(1);

    private final JoystickButton m_getFirstCargoCommandButton = new JoystickButton(
        m_stick1Left, Constants.kButtonIntakeFirstCargo);
    private final JoystickButton m_getCargoCommandButton = new JoystickButton(
        m_stick1Left, Constants.kButtonIntake);
    private final JoystickButton m_shooterCommandButton = new JoystickButton(
        m_stick1Left, Constants.kButtonShooter);
    private final JoystickButton m_elevatorCommandButton = new JoystickButton(
        m_stick1Left, Constants.kButtonElevator);
    private final JoystickButton m_elevatorReverseCommandButton = new JoystickButton(
        m_stick1Left, Constants.kButtonReverseElevator);

    public RobotContainer() {
        m_stick1Right.setYChannel(5);
        m_stick1Right.setThrottleChannel(2);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        m_driveTrainSubsystem.setDefaultCommand(new RunCommand(
                () -> m_driveTrainSubsystem.drive(m_stick1Left.getY(), m_stick1Right.getY()),
                m_driveTrainSubsystem));

        m_shooterSubsystem.setDefaultCommand(new RunCommand(
                () -> m_shooterSubsystem.shooter(
                    m_stick1Left.getThrottle(), 
                    m_stick1Right.getThrottle()), 
                m_shooterSubsystem));
        
        m_getFirstCargoCommandButton.whenPressed(
            new IntakeCommand(m_intakeSubsystem, m_elevatorSubsystem, true).withTimeout(Constants.kIntakeTimeout));
        m_getCargoCommandButton.whileHeld(new IntakeCommand(m_intakeSubsystem, m_elevatorSubsystem, false));

        m_elevatorCommandButton.whileHeld(
                new StartEndCommand(
                        () -> m_elevatorSubsystem.elevatorStart(),
                        () -> m_elevatorSubsystem.elevatorEnd(), m_elevatorSubsystem));

        m_elevatorReverseCommandButton.whileHeld(
                new StartEndCommand(
                        () -> m_elevatorSubsystem.elevatorReverseStart(),
                        () -> m_elevatorSubsystem.elevatorEnd(), m_elevatorSubsystem));

        m_shooterCommandButton.whenPressed(
            new RunCommand(() -> m_shooterSubsystem.shooterStart(), m_shooterSubsystem)
            .withTimeout(Constants.kShooterTimeout)
            .andThen(new InstantCommand(() -> m_shooterSubsystem.shooterEnd(), m_shooterSubsystem)));
        m_shooterCommandButton.whileHeld(new StartEndCommand(
            () -> m_elevatorSubsystem.elevatorStart(),
            () -> m_elevatorSubsystem.elevatorEnd(), 
            m_elevatorSubsystem));

    }

    public Command getAutonomousCommand() {
        return null;
    }
}
