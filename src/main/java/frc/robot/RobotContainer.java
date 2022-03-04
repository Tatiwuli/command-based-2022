// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.FollowCargo;
import frc.robot.commands.GrabCargoCamera;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeWithElevatorCommand;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();

    private final Joystick m_stick1Left = new Joystick(0);
    private final Joystick m_stick1Right = new Joystick(0);
    private final Joystick m_stickClimb1 = new Joystick(1);
    private final Joystick m_stickClimb2 = new Joystick(1);

    private final JoystickButton m_reverseIntake = new JoystickButton(
            m_stick1Right, Constants.kButtonReverseIntake);
    private final JoystickButton m_grabCargo = new JoystickButton(
            m_stick1Right, Constants.kButtonGrabCargo);
    private final JoystickButton m_grabCargoCamera = new JoystickButton(
            m_stick1Right, Constants.kButtonGrabCargoCamera);
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

    private final JoystickButton m_climbAuxButton = new JoystickButton(
            m_stickClimb1, Constants.kStickClimb1);
    private final JoystickButton m_climbButton = new JoystickButton(
            m_stickClimb1, Constants.kStickClimb1);
    private final JoystickButton m_climbAuxSlowButton = new JoystickButton(
            m_stickClimb1, Constants.kStickClimb2);
    private final JoystickButton m_climbMainButton = new JoystickButton(
            m_stickClimb1, Constants.kStickClimb2);

    // private final JoystickButton m_resetDrive = new JoystickButton(m_stick1Right,
    // Constants.kButtonResetDriveSensors);

    public RobotContainer() {
        m_stick1Right.setYChannel(5);
        m_stick1Right.setThrottleChannel(2);
        m_stickClimb1.setYChannel(5);
        m_stickClimb2.setThrottleChannel(2);
        configureButtonBindings();
    }

    private void configureButtonBindings() {

        m_grabCargo.toggleWhenPressed(new IntakeCommand(m_intakeSubsystem));
        // m_grabCargoCamera.whileHeld(new FollowCargo(Robot.followCargo,
        // m_driveSubsystem));
        m_grabCargoCamera.whileHeld(new FollowCargo(Robot.followCargo, m_driveSubsystem).alongWith(
                new IntakeWithElevatorCommand(m_intakeSubsystem, m_elevatorSubsystem, true)
                        .withTimeout(Constants.kIntakeTimeout)));

        m_reverseIntake.whileHeld(new StartEndCommand(
                () -> m_intakeSubsystem.intakeReverseStart(),
                () -> m_intakeSubsystem.intakeEnd(), m_intakeSubsystem));
        // m_resetDrive.whenPressed(new InstantCommand(() -> m_driveSubsystem.reset(),
        // m_driveSubsystem));

        m_driveSubsystem.setDefaultCommand(new RunCommand(
                () -> m_driveSubsystem.tankDrive(m_stick1Left.getY(), m_stick1Right.getY()),
                m_driveSubsystem));

        m_climbSubsystem.setDefaultCommand(new RunCommand(
                () -> m_climbSubsystem.set(m_stickClimb1.getY(), m_stickClimb2.getY()),
                m_climbSubsystem));

        m_shooterSubsystem.setDefaultCommand(new RunCommand(
                () -> m_shooterSubsystem.shooter(
                        m_stick1Left.getThrottle(),
                        m_stick1Right.getThrottle()),
                m_shooterSubsystem));

        m_climbSubsystem.setDefaultCommand(new RunCommand(
                () -> m_climbSubsystem.set(
                        m_stickClimb1.getThrottle(),
                        m_stickClimb2.getThrottle()),
                m_climbSubsystem));

        m_getFirstCargoCommandButton.whenPressed(
                new IntakeWithElevatorCommand(m_intakeSubsystem, m_elevatorSubsystem, true)
                        .withTimeout(Constants.kIntakeTimeout));
        m_getCargoCommandButton.whileHeld(new IntakeWithElevatorCommand(m_intakeSubsystem, m_elevatorSubsystem, false));

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
        m_shooterCommandButton.whileHeld(new WaitCommand(Constants.kShooterTimeout).andThen(new StartEndCommand(
                () -> m_elevatorSubsystem.elevatorStart(),
                () -> m_elevatorSubsystem.elevatorEnd(),
                m_elevatorSubsystem)));

        m_climbAuxButton.whileHeld(new StartEndCommand(
                () -> m_climbSubsystem.auxStart(),
                () -> m_climbSubsystem.auxEnd(),
                m_climbSubsystem));

        m_climbButton.whileHeld(new StartEndCommand(
                () -> m_climbSubsystem.start(),
                () -> m_climbSubsystem.end(),
                m_climbSubsystem));

        m_climbAuxSlowButton.whenPressed(new StartEndCommand(
                () -> m_climbSubsystem.auxSlow(),
                () -> m_climbSubsystem.auxEnd(),
                m_climbSubsystem));

        m_climbMainButton.whenPressed(new StartEndCommand(
                () -> m_climbSubsystem.mainStart(),
                () -> m_climbSubsystem.mainEnd(),
                m_climbSubsystem));

        for (int angle = 0; angle < 360; angle += 45) {
            new POVButton(m_stick1Right, angle)
                    .whileHeld(new TurnToAngle(angle, m_driveSubsystem, true, 1.0));
            new POVButton(m_stick1Right, angle)
                    .whenReleased(new TurnToAngle(angle, m_driveSubsystem, false).withTimeout(5));
        }

    }

    public Command getAutonomousCommand() {
        // return new DriveStraight(50, m_driveSubsystem);
        // return new DriveStraight(100, m_driveSubsystem);
        return null;
    }
}
