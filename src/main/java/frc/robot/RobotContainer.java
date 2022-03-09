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
import frc.robot.commands.DriveStraightGyro;
import frc.robot.commands.FindCargo;
import frc.robot.commands.FollowCargo;
import frc.robot.commands.GrabCargo;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeWithElevatorCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.PrepareAndShootCommand;
import frc.robot.commands.autonomous.AutoDriveShootAndGrabOneAndStopCommand;
import frc.robot.commands.autonomous.AutoDriveShootAndGrabOneOnlyIfDetectedAndStopCommand;
import frc.robot.commands.autonomous.AutoDriveShootAndGrabOneCommand;
import frc.robot.commands.autonomous.AutoDriveShootAndGrabTwoAndStopCommand;
import frc.robot.commands.autonomous.AutoDriveShootCommand;
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

    private final Joystick m_stick1Left = new Joystick(Constants.Stick1.kStick1Port);
    private final Joystick m_stick1Right = new Joystick(Constants.Stick1.kStick1Port);
    private final Joystick m_stickClimb1 = new Joystick(Constants.Stick2.kStick2Port);
    private final Joystick m_stickClimb2 = new Joystick(Constants.Stick2.kStick2Port);

    private final JoystickButton m_reverseIntake = new JoystickButton(
            m_stick1Right, Constants.Stick1.kButtonReverseIntake);
    private final JoystickButton m_grabCargo = new JoystickButton(
            m_stick1Right, Constants.Stick1.kButtonGrabCargo);
    private final JoystickButton m_grabCargoCamera = new JoystickButton(
            m_stick1Right, Constants.Stick1.kButtonGrabCargoCamera);
    private final JoystickButton m_getFirstCargoCommandButton = new JoystickButton(
            m_stick1Left, Constants.Stick1.kButtonIntakeFirstCargo);
    private final JoystickButton m_getCargoCommandButton = new JoystickButton(
            m_stick1Left, Constants.Stick1.kButtonIntake);
    private final JoystickButton m_shooterCommandButton = new JoystickButton(
            m_stick1Left, Constants.Stick1.kButtonShooter);
    private final JoystickButton m_elevatorCommandButton = new JoystickButton(
            m_stick1Left, Constants.Stick1.kButtonElevator);
    private final JoystickButton m_elevatorReverseCommandButton = new JoystickButton(
            m_stick1Left, Constants.Stick1.kButtonReverseElevator);

    private final JoystickButton m_climbButtonForward = new JoystickButton(
            m_stickClimb1, Constants.Stick2.kButtonClimbForward);
    private final JoystickButton m_climbButtonReverse = new JoystickButton(
            m_stickClimb1, Constants.Stick2.kButtonClimbReverse);
    // private final JoystickButton m_resetDrive = new JoystickButton(m_stick1Right,
    // Constants.kButtonResetDriveSensors);

    SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    Command m_driveAndShoot = new AutoDriveShootCommand(m_driveSubsystem, m_shooterSubsystem, m_elevatorSubsystem);
    Command m_driveWithIntakeOnAndShoot = new AutoDriveShootCommand(m_driveSubsystem, m_shooterSubsystem, m_elevatorSubsystem, 
            m_intakeSubsystem, true);
    Command m_driveShootGrabOne = new AutoDriveShootAndGrabOneCommand(m_driveSubsystem, m_shooterSubsystem,
            m_elevatorSubsystem, m_intakeSubsystem);
    Command m_driveShootGrabTwo = new AutoDriveShootAndGrabTwoAndStopCommand(m_driveSubsystem, m_shooterSubsystem,
            m_elevatorSubsystem, m_intakeSubsystem);
    Command m_driveShootGrabOneAndStop = new AutoDriveShootAndGrabOneAndStopCommand(m_driveSubsystem, m_shooterSubsystem,
            m_elevatorSubsystem, m_intakeSubsystem, true);
    Command m_driveShootGrabOneWithoutCamAndStop = new AutoDriveShootAndGrabOneAndStopCommand(m_driveSubsystem, m_shooterSubsystem,
            m_elevatorSubsystem, m_intakeSubsystem, false);
    Command m_driveShootGrabTwoAndStop = new AutoDriveShootAndGrabTwoAndStopCommand(m_driveSubsystem, m_shooterSubsystem,
            m_elevatorSubsystem, m_intakeSubsystem);
    Command m_driveShootGrabOneOnlyIfDetectedAndStop = new AutoDriveShootAndGrabOneOnlyIfDetectedAndStopCommand(
            m_driveSubsystem, m_shooterSubsystem, m_elevatorSubsystem, m_intakeSubsystem);
            

    public RobotContainer() {
        m_stick1Right.setYChannel(5);
        m_stick1Right.setThrottleChannel(2);
        m_stickClimb1.setYChannel(5);
        m_stickClimb2.setThrottleChannel(2);
        m_autoChooser.setDefaultOption("1 - [AUTO MAIN] Drive and shoot", 
                m_driveAndShoot); /* OK */
        m_autoChooser.setDefaultOption("2 - [AUTO MAIN] Drive with intake on and shoot", 
                m_driveWithIntakeOnAndShoot); /* OK */
        m_autoChooser.addOption("3 - [AUTO] Drive, shoot and grab one cargo",
                m_driveShootGrabOne); /* OK */
        m_autoChooser.addOption("4 - [AUTO MAIN] Drive, shoot, grab one cargo and stop",   
                m_driveShootGrabOneAndStop); /* OK */
        m_autoChooser.addOption("5 - [AUTO] Drive, shoot and grab one cargo (without camera) and stop",  
                m_driveShootGrabOneWithoutCamAndStop); /* NOT TESTED */ 
        m_autoChooser.addOption("6 - [AUTO] Drive, shoot, grab one cargo (only if detected) and stop",  
                m_driveShootGrabOneOnlyIfDetectedAndStop);  /* NOT TESTED */ 
        m_autoChooser.addOption("7 - [AUTO] Drive, shoot and grab two cargos",
                m_driveShootGrabTwo); /* NOT TESTED */ 
        m_autoChooser.addOption("8 - [AUTO MAIN] Drive, shoot, grab two cargos and stop",  
                m_driveShootGrabTwoAndStop);  /* FAILED (second cargo) */
        m_autoChooser.addOption("9 - [AUTO] Drive 200ft backward with intake on",
                new DriveStraight(200, m_driveSubsystem).withTimeout(8)); 
        m_autoChooser.addOption("10 - [TEST] Drive forward", 
                new DriveStraight(100, m_driveSubsystem));
        m_autoChooser.addOption("11 - [TEST] Drive forward (using gyro)", 
                new DriveStraightGyro(m_driveSubsystem, 0.7).withTimeout(5));
        m_autoChooser.addOption("12 - [TEST] Find and grab one cargo", 
                new FindCargo(m_driveSubsystem).withTimeout(6).andThen(
                new GrabCargo(m_driveSubsystem, m_elevatorSubsystem, m_intakeSubsystem).withTimeout(6)));       
        m_autoChooser.addOption("13 - [TEST] Shooter with elevator command", 
                new PrepareAndShootCommand(m_shooterSubsystem, m_elevatorSubsystem, 2));
        m_autoChooser.setDefaultOption("14 - [TEST] Grab cargo if detected", 
                new GrabCargo(m_driveSubsystem, m_elevatorSubsystem, m_intakeSubsystem).withTimeout(5));
        SmartDashboard.putData(m_autoChooser);
        configureButtonBindings();
    }

    private void configureButtonBindings() {

        // INTAKE
        m_grabCargo.toggleWhenPressed(new IntakeCommand(m_intakeSubsystem));
        // m_grabCargoCamera.whileHeld(new FollowCargo(Robot.followCargo,
        // m_driveSubsystem));
        m_grabCargoCamera.whileHeld(new FollowCargo(m_driveSubsystem).alongWith(
                new IntakeWithElevatorCommand(m_intakeSubsystem, m_elevatorSubsystem, true)
                        .withTimeout(Constants.Intake.kIntakeTimeout)));
        m_reverseIntake.whileHeld(new StartEndCommand(
                () -> m_intakeSubsystem.intakeReverseStart(),
                () -> m_intakeSubsystem.intakeEnd(), m_intakeSubsystem));

        m_getFirstCargoCommandButton.whenPressed(
                new IntakeWithElevatorCommand(m_intakeSubsystem, m_elevatorSubsystem, true)
                        .withTimeout(Constants.Intake.kIntakeTimeout));
        m_getCargoCommandButton.whileHeld(new IntakeWithElevatorCommand(m_intakeSubsystem, m_elevatorSubsystem, false));

        // DRIVE
        m_driveSubsystem.setDefaultCommand(new RunCommand(
                () -> m_driveSubsystem.tankDrive(m_stick1Left.getY(), m_stick1Right.getY()),
                m_driveSubsystem));
                
        new POVButton(m_stick1Right, 0)
                .whileHeld(new DriveStraightGyro(m_driveSubsystem, 1));
        new POVButton(m_stick1Right, 0).whenReleased(new WaitCommand(1)
                .andThen(new InstantCommand(() -> m_driveSubsystem.resetForwardOrient(), m_driveSubsystem)));
        new POVButton(m_stick1Right, 180)
                .whileHeld(new DriveStraightGyro(m_driveSubsystem, -1));
        new POVButton(m_stick1Right, 180).whenReleased(new WaitCommand(1)
                .andThen(new InstantCommand(() -> m_driveSubsystem.resetForwardOrient(), m_driveSubsystem)));

        // CLIMB
        m_climbSubsystem.setDefaultCommand(new RunCommand(
                () -> {
                    // System.out.println(m_stickClimb1.getY() + " " + m_stickClimb2.getY());
                    m_climbSubsystem.set(m_stickClimb1.getY(), m_stickClimb2.getY());
                },
                m_climbSubsystem));
        m_climbButtonForward.whileHeld(new StartEndCommand(
                () -> m_climbSubsystem.forward(),
                () -> m_climbSubsystem.end(),
                m_climbSubsystem));
        m_climbButtonReverse.whenPressed(new StartEndCommand(
                () -> m_climbSubsystem.reverse(),
                () -> m_climbSubsystem.end(),
                m_climbSubsystem));

        // SHOOTER
        m_shooterSubsystem.setDefaultCommand(new RunCommand(
                () -> m_shooterSubsystem.shooter(
                        m_stick1Left.getThrottle(),
                        m_stick1Right.getThrottle()),
                m_shooterSubsystem));

        m_shooterCommandButton
                .whenPressed(new InstantCommand(() -> m_shooterSubsystem.shooterStart(), m_shooterSubsystem)
                        .andThen(new WaitCommand(2))
                        .andThen(new ShooterCommand(m_shooterSubsystem, m_elevatorSubsystem)
                                .withTimeout(Constants.Shooter.kShooterTimeout)));

        // ELEVATOR
        m_elevatorCommandButton.whileHeld(
                new StartEndCommand(
                        () -> m_elevatorSubsystem.elevatorStart(),
                        () -> m_elevatorSubsystem.elevatorEnd(), m_elevatorSubsystem));

        m_elevatorReverseCommandButton.whileHeld(
                new StartEndCommand(
                        () -> m_elevatorSubsystem.elevatorReverseStart(),
                        () -> m_elevatorSubsystem.elevatorEnd(), m_elevatorSubsystem));

        // m_resetDrive.whenPressed(new InstantCommand(() -> m_driveSubsystem.reset(),
        // m_driveSubsystem));

    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}
