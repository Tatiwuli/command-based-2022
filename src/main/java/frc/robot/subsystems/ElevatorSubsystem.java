package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    private VictorSP m_motor;

    AnalogInput sensorElevator = new AnalogInput(Constants.DIOPorts.kElevatorPhotoeletricPort);

    public ElevatorSubsystem() {
        m_motor = new VictorSP(Constants.PWMPorts.kElevatorPort);
    }

    public boolean cargoDetected() {
        if (sensorElevator.getAverageValue() > 4000) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("[Elevator] Cargo Detected", (sensorElevator.getAverageValue() > 4000));
    }

    public void elevatorStart() {
        m_motor.set(Constants.Elevator.kElevatorSpeed);
    }

    public void elevatorReverseStart() {
        m_motor.set(-Constants.Elevator.kElevatorSpeed);
    }

    public void elevatorEnd() {
        m_motor.set(0);
    }
}
