// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.vision.CargoColor;
import frc.robot.vision.FollowCargoRunner;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private SendableChooser<CargoColor> m_colorChooser = new SendableChooser<>();
    public static final FollowCargoRunner followCargoRunner = new FollowCargoRunner(0, CargoColor.BLUE);
    private Thread followCargoThread = new Thread(followCargoRunner);
    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        m_colorChooser.addOption("Blue Cargo", CargoColor.BLUE);
        m_colorChooser.addOption("Red Cargo", CargoColor.RED);
        m_colorChooser.addOption("Yellow Cargo", CargoColor.YELLOW);
        m_colorChooser.setDefaultOption("Blue Cargo", CargoColor.BLUE);
        SmartDashboard.putData(m_colorChooser);
        m_robotContainer = new RobotContainer();
        followCargoThread.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        followCargoRunner.setCargoColor(m_colorChooser.getSelected());
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        followCargoRunner.setCargoColor(m_colorChooser.getSelected());
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }
}
