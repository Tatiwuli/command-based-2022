package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private VictorSP m_motor;

    public IntakeSubsystem() {
        m_motor = new VictorSP(Constants.PWMPorts.kIntakePort);
        m_motor.setInverted(true);
    }

    public void intakeStart() {
        m_motor.set(Constants.Intake.kIntakeSpeed);
    }
    
    public void intakeReverseStart() {
        m_motor.set(-Constants.Intake.kIntakeSpeed);
    }

    public void intakeEnd() {
        m_motor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("[Intake] ON", m_motor.get() > 0);
    }
}
