// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {

    private ShooterSubsystem m_shooterSubsystem;
    private ElevatorSubsystem m_elevatorSubsystem;

  public ShooterCommand(ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem ) {
      
    m_shooterSubsystem = shooterSubsystem;
    m_elevatorSubsystem = elevatorSubsystem; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.shooterStart();
    m_elevatorSubsystem.elevatorStart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_shooterSubsystem.shooterEnd();
      m_elevatorSubsystem.elevatorEnd();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevatorSubsystem.cargoDetected();
  }
}
