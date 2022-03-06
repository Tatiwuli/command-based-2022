// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


public final class Constants {

    public static final int kleft1Port = 0;
    public static final int kleft2Port = 1;
    public static final int kright1Port = 2;
    public static final int kright2Port = 3;
    public static final double kleftSpeed = 0.4 ;

    public static final int kStick1 = 0;
    
    public static final int kIntakePort = 8;
    public static final double kIntakeSpeed = 1;
    public static final double kIntakeTimeout = 5;

    public static final int kElevatorPort = 6;
    public static final double kElevatorSpeed = 1;

    public static final int kShooterRightPort = 7;
    public static final int kShooterLeftPort = 5;
    public static final double kShooterSpeed = 1;
    public static final double kShooterTimeout = 2;

    public static final int kButtonGrabCargo = 1;
    public static final int kButtonGrabCargoCamera = 8;
    // public static final int kButtonResetDriveSensors = ;
    public static final int kButtonReverseIntake = 7;
    public static final int kButtonElevator = 2;
    public static final int kButtonReverseElevator = 3;
    public static final int kButtonShooter = 4;
    public static final int kButtonIntake = 5;
    public static final int kButtonIntakeFirstCargo = 6;

    
    public static final int kClimbAuxPort = 9;
    public static final int kClimbMainPort = 4;
    
    public static final int kStickClimb1 = 1;
    public static final int kStickClimb2 = 1;
    public static final int kButtonClimb = 0;
    public static final int kButtonAuxSlow = 0;
    public static final int kButtonClimbForward = 1;
    public static final int kButtonClimbReverse = 2;
    public static final int kButtonForwardSlow = 3;
    public static final int kButtonClimbReverseSlow = 4;

    public static final double kclimbSpeed = 0.6;
    public static final double kclimbReverseSpeed = -0.5;
    public static final double kclimbAuxSpeed = 1;
    public static final double kclimbAuxReverseSpeed = -1;
    public static final double kclimbSlowSpeed = 0.6;
    public static final double kclimbReverseSlowSpeed = -0.3;
    public static final double kclimbAuxReverseSlowSpeed = 0.8;
    public static final double kclimbAuxSlowSpeed = -0.8;
    public static final double kclimbMainSpeed = 0.8;
    
    public class Drive {
        public static final double kTurnP = 0.03;
        public static final double kTurnI = 0.06;
        public static final double kTurnD = 0.01;
        public static final double kToleranceDegrees = 5.0f;
        public static final double kTurnRateToleranceDegPerS = 10.0f;
    }

}
