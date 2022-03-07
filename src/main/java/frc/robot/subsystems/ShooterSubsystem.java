package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {

    private VictorSP m_motorLeft;
    private VictorSP m_motorRight;
    private MotorControllerGroup m_motors;

    public ShooterSubsystem() {
        m_motorLeft = new VictorSP(Constants.PWMPorts.kShooterLeftPort);
        m_motorRight = new VictorSP(Constants.PWMPorts.kShooterRightPort);
        m_motorLeft.setInverted(true);
        m_motors = new MotorControllerGroup(m_motorLeft, m_motorRight);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("[Shooter] ON", m_motors.get() > 0);
    }

    public void setMotors(double speed) {
        m_motors.set(speed);
    }

    public void shooterStart() {
        m_motors.set(Constants.Shooter.kShooterSpeed);
    }

    public void shooterEnd() {
        m_motors.set(0);
    }

    public void shooter(double left, double right) {
        m_motorLeft.set(left);
        m_motorRight.set(right);
    }
}
