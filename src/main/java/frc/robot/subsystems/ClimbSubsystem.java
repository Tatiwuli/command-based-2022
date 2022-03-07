package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

    VictorSP m_climbAux;
    VictorSP m_climbMain;
    AnalogInput sensorClimb = new AnalogInput(Constants.DIOPorts.kClimbPhotoeletricPort);

    public ClimbSubsystem() {
        m_climbAux = new VictorSP(Constants.PWMPorts.kClimbAuxPort);
        m_climbAux.setInverted(true);
        m_climbMain = new VictorSP(Constants.PWMPorts.kClimbMainPort);
    }

    public boolean climbDone() {
        if (sensorClimb.getAverageValue() > 4000) {
            return false;
        } 
        return true;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("[Climb] Climb Done", climbDone());
    }
   
    public void reverse(){
        m_climbAux.set(Constants.Climb.kclimbAuxReverseSpeed);
        m_climbMain.set(Constants.Climb.kclimbReverseSpeed);
    }

    public void forward(){
        m_climbAux.set(Constants.Climb.kclimbAuxSpeed);
        m_climbMain.set(Constants.Climb.kclimbSpeed);
    }

    public void set(double speedAux, double speedMain) {
        m_climbAux.set(speedAux);
        m_climbMain.set(speedMain);
    }

    public void end(){
        m_climbAux.set(0);
        m_climbMain.set(0);
    }

}
