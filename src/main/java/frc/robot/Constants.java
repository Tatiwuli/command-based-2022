package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

public final class Constants {

    public class Auto {
        public static final int autoDriveDistance = 50;
    }

    public class Stick1 {
        public static final int kStick1Port = 0;
        public static final int kButtonGrabCargo = 1;
        public static final int kButtonElevator = 2;
        public static final int kButtonReverseElevator = 3;
        public static final int kButtonShooter = 4;
        public static final int kButtonIntake = 5;
        public static final int kButtonIntakeFirstCargo = 6;
        public static final int kButtonReverseIntake = 7;
        public static final int kButtonGrabCargoCamera = 8;
        // public static final int kButtonResetDriveSensors = ;
    }

    public class Stick2 {
        public static final int kStick2Port = 1;
        public static final int kButtonClimbForward = 1;
        public static final int kButtonClimbReverse = 2;
    }

    public class Intake {
        public static final double kIntakeSpeed = 1;
        public static final double kIntakeTimeout = 5;
    }

    public class Elevator {
        public static final double kElevatorSpeed = 1;
    }
    
    public class Shooter {
        public static final double kShooterSpeed = 1;
        public static final double kShooterTimeout = 3;
    }
    
    public class Climb {
        public static final double kclimbSpeed = 0.6;
        public static final double kclimbReverseSpeed = -0.5;
        public static final double kclimbAuxSpeed = 1;
        public static final double kclimbAuxReverseSpeed = -1;;
    }
    
    public class PWMPorts {
        public static final int kLeft1Port = 0;
        public static final int kLeft2Port = 1;
        public static final int kRight1Port = 2;
        public static final int kRight2Port = 3;
        public static final int kClimbMainPort = 4;
        public static final int kShooterLeftPort = 5;
        public static final int kElevatorPort = 6;
        public static final int kShooterRightPort = 7;
        public static final int kIntakePort = 8;
        public static final int kClimbAuxPort = 9;
    }

    public class DIOPorts {
        public static final int kEncoderLeftPortA = 4;
        public static final int kEncoderLeftPortB = 5;
        public static final int kEncoderRightPortA = 6;
        public static final int kEncoderRightPortB = 7;
        public static final int kElevatorPhotoeletricPort = 0;
        public static final int kClimbPhotoeletricPort = 1;
    }
    
    public static final SerialPort.Port kGyroPort = Port.kUSB;
    public class Drive {
        public static final double kTurnP = 0.04;
        public static final double kTurnI = 0.06;
        public static final double kTurnD = 0.01;
        public static final double kToleranceDegrees = 5.0f;
        public static final double kTurnRateToleranceDegPerS = 10.0f;
        public static final double kEncoderLeftP = 0.03;
        public static final double kEncoderLeftI = 0.01;
        public static final double kEncoderLeftD = 0;
        public static final double kEncoderRightP = 0.03;
        public static final double kEncoderRightI = 0.01;
        public static final double kEncoderRightD = 0;
    }
}
