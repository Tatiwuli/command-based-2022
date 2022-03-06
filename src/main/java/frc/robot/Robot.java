// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.vision.CargoColor;
import frc.robot.vision.FollowCargoRunner;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    SendableChooser<CargoColor> m_colorChooser = new SendableChooser<>();
    public static FollowCargoRunner followCargo = new FollowCargoRunner(0, CargoColor.BLUE);
    private RobotContainer m_robotContainer;
    Thread followCargoThread;

    @Override
    public void robotInit() {
        m_colorChooser.addOption("Blue Cargo", CargoColor.BLUE);
        m_colorChooser.addOption("Blue Cargo", CargoColor.RED);
        m_colorChooser.setDefaultOption("Blue Cargo", CargoColor.BLUE);
        m_robotContainer = new RobotContainer();
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
        followCargo = new FollowCargoRunner(0, m_colorChooser.getSelected());
        if (followCargoThread != null) followCargoThread.interrupt();
        followCargoThread = new Thread(followCargo);
        followCargoThread.start();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        followCargo = new FollowCargoRunner(0, m_colorChooser.getSelected());
        if (followCargoThread != null) followCargoThread.interrupt();
        followCargoThread = new Thread(followCargo);
        followCargoThread.start();
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
