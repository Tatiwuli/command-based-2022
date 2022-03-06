// package frc.robot.commands.autonomous;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.StartEndCommand;
// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.commands.DriveStraight;
// import frc.robot.commands.FindCargo;
// import frc.robot.commands.FollowCargo;
// import frc.robot.commands.IntakeWithElevatorCommand;
// import frc.robot.commands.ShooterCommand;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.vision.FollowCargoRunner;

// public class AutoNoShoot extends SequentialCommandGroup {

//     public AutoNoShoot(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem, 
//     FollowCargoRunner followCargoRunner, DriveSubsystem drive, IntakeSubsystem m_intakeSubsystem) {

//         addCommands(
//             new DriveStraight(-100, driveSubsystem),
//             new InstantCommand(() -> elevatorSubsystem.elevatorStart(), elevatorSubsystem),
//             new InstantCommand(() ->elevatorSubsystem.elevatorEnd(), elevatorSubsystem),
//             new FindCargo(followCargoRunner, drive),
//             new FollowCargo(Robot.followCargo, drive),
//                 new InstantCommand(() -> m_intakeSubsystem.intakeStart(), m_intakeSubsystem),
//                 new InstantCommand(() -> m_intakeSubsystem.intakeEnd(), m_intakeSubsystem)
//                 .);
//     }

//     @Override
//     public void initialize() {
//     }

//     @Override
//     public void execute() {
//     }

//     @Override
//     public void end(boolean interrupted) {
//     }

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
