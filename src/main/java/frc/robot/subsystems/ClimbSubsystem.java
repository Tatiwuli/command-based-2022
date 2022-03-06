// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

    VictorSP m_climbAux;
    VictorSP m_climbMain;
    AnalogInput sensorClimb = new AnalogInput(1);

    public ClimbSubsystem() {
        m_climbAux = new VictorSP(Constants.kClimbAuxPort);
        m_climbAux.setInverted(true);
        m_climbMain = new VictorSP(Constants.kClimbMainPort);
    }

    public boolean climbDone() {
        if (sensorClimb.getAverageValue() > 4000) {
            return false;
        } 
        return true;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climb Done", climbDone());
    }
   
    public void reverse(){
        m_climbAux.set(Constants.kclimbAuxReverseSpeed);
        m_climbMain.set(Constants.kclimbReverseSpeed);
    }

    public void reverseSlow(){
        m_climbAux.set(Constants.kclimbAuxReverseSlowSpeed);
        m_climbMain.set(Constants.kclimbReverseSlowSpeed);
    }

    public void forward(){
        m_climbAux.set(Constants.kclimbAuxSpeed);
        m_climbMain.set(Constants.kclimbSpeed);
    }

    public void forwardSlow(){
        m_climbAux.set(Constants.kclimbAuxSlowSpeed);
        m_climbMain.set(Constants.kclimbSlowSpeed);
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
