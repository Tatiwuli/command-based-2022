// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.FollowCargo;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeWithElevatorCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.autonomous.AutoDriveShootAndGrabOneCommand;
import frc.robot.commands.autonomous.AutoDriveShootAndGrabTwoCommand;
import frc.robot.commands.autonomous.AutoDriveAndShootCommand;
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

    private final JoystickButton m_climbButtonForward = new JoystickButton(
            m_stickClimb1, Constants.kButtonClimbForward);
    private final JoystickButton m_climbButtonReverse = new JoystickButton(
            m_stickClimb1, Constants.kButtonClimbReverse);
    private final JoystickButton m_climbButtonForwardSlow = new JoystickButton(
            m_stickClimb1, Constants.kButtonForwardSlow);
    private final JoystickButton m_climbButtonReverseSlow = new JoystickButton(
            m_stickClimb1, Constants.kButtonClimbReverseSlow);

    // private final JoystickButton m_resetDrive = new JoystickButton(m_stick1Right,
    // Constants.kButtonResetDriveSensors);

    SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    Command m_driveAndShoot = new AutoDriveAndShootCommand(m_driveSubsystem, m_shooterSubsystem, m_elevatorSubsystem);
    Command m_driveShootGrabOne = new AutoDriveShootAndGrabOneCommand(m_driveSubsystem, m_shooterSubsystem, m_elevatorSubsystem, m_intakeSubsystem);
    Command m_driveShootGrabTwo = new AutoDriveShootAndGrabTwoCommand(m_driveSubsystem, m_shooterSubsystem, m_elevatorSubsystem, m_intakeSubsystem);

    public RobotContainer() {
        m_stick1Right.setYChannel(5);
        m_stick1Right.setThrottleChannel(2);
        m_stickClimb1.setYChannel(5);
        m_stickClimb2.setThrottleChannel(2);
        m_autoChooser.setDefaultOption("Simple Auto", m_driveAndShoot);
        m_autoChooser.addOption("Drive, shoot and grab one cargo", m_driveShootGrabOne);
        m_autoChooser.addOption("Drive, shoot and grab one cargo", m_driveShootGrabOne);
        m_autoChooser.addOption("Drive forward", new DriveStraight(-100, m_driveSubsystem));
        SmartDashboard.putData(m_autoChooser);
        configureButtonBindings();
    }

    private void configureButtonBindings() {

        m_grabCargo.toggleWhenPressed(new IntakeCommand(m_intakeSubsystem));
        // m_grabCargoCamera.whileHeld(new FollowCargo(Robot.followCargo,
        // m_driveSubsystem));
        m_grabCargoCamera.whileHeld(new FollowCargo(m_driveSubsystem).alongWith(
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

        m_shooterCommandButton.whenPressed(new InstantCommand(() -> m_shooterSubsystem.shooterStart(), m_shooterSubsystem)
                .andThen(new WaitCommand(2))
                .andThen(new ShooterCommand(m_shooterSubsystem, m_elevatorSubsystem).withTimeout(Constants.kShooterTimeout))
        );

        m_climbButtonForward.whileHeld(new StartEndCommand(
                () -> m_climbSubsystem.forward(),
                () -> m_climbSubsystem.end(),
                m_climbSubsystem));

        m_climbButtonForwardSlow.whenPressed(new StartEndCommand(
                () -> m_climbSubsystem.forwardSlow(),
                () -> m_climbSubsystem.end(),
                m_climbSubsystem));

        m_climbButtonReverse.whenPressed(new StartEndCommand(
                () -> m_climbSubsystem.reverse(),
                () -> m_climbSubsystem.end(),
                m_climbSubsystem));

        m_climbButtonReverseSlow.whenPressed(new StartEndCommand(
                () -> m_climbSubsystem.reverseSlow(),
                () -> m_climbSubsystem.end(),
                m_climbSubsystem));

        for (int angle = 0; angle < 360; angle += 45) {
            new POVButton(m_stick1Right, angle)
                    .whileHeld(new TurnToAngle(angle, m_driveSubsystem, true, 1.0));
            new POVButton(m_stick1Right, angle)
                    .whenReleased(new TurnToAngle(angle, m_driveSubsystem, false).withTimeout(5));
        }

    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}
