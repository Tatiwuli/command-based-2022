// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

    VictorSP m_climbAux;
    VictorSP m_climbMain;

    public ClimbSubsystem() {
        m_climbAux = new VictorSP(Constants.kClimbAuxPort);
        m_climbMain = new VictorSP(Constants.kClimbMainPort);
    }

    @Override
    public void periodic() {
    }

    public void auxStart() {
        m_climbAux.set(Constants.kclimbAuxSpeed);
    }

    public void auxEnd() {
        m_climbAux.set(0.0);
    }

    public void auxSlow() {
        m_climbAux.set(Constants.kclimbAuxSlowSpeed);
    }

    public void start(){
        m_climbAux.set(Constants.kclimbAuxSlowSpeed);
        m_climbMain.set(Constants.kclimbMainSpeed);
    
    }
    public void end(){
        m_climbAux.set(0.0);
        m_climbMain.set(0.0);
    
    }
    public void mainStart() {
        m_climbMain.set(Constants.kclimbMainSpeed);

    }

    public void mainEnd() {
        m_climbAux.set(0.0);
    }

    public void set(double speedAux, double speedMain) {
        m_climbAux.set(speedAux);
        m_climbMain.set(speedMain);
    }

}
