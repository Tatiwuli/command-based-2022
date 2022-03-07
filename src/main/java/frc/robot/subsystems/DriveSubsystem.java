package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util;

public class DriveSubsystem extends SubsystemBase {
    private Spark m_left1;
    private Spark m_left2;
    private Spark m_right1;
    private Spark m_right2;

    private Encoder m_leftEncoder = new Encoder(
        Constants.DIOPorts.kEncoderLeftPortA, Constants.DIOPorts.kEncoderLeftPortB);
    private Encoder m_rightEncoder = new Encoder(
        Constants.DIOPorts.kEncoderRightPortA, Constants.DIOPorts.kEncoderRightPortB);

    private MotorControllerGroup m_leftMotors;
    private MotorControllerGroup m_rightMotors;
    private DifferentialDrive m_drive;

    private static final double FORWARD_ORIENT_NOT_SET = -1;
    private double forwardOrient = FORWARD_ORIENT_NOT_SET;

    private AHRS m_gyro;
    // PIDController m_turnController;

    public DriveSubsystem() {
        m_left1 = new Spark(Constants.PWMPorts.kLeft1Port);
        m_left2 = new Spark(Constants.PWMPorts.kLeft2Port);
        m_right1 = new Spark(Constants.PWMPorts.kRight1Port);
        m_right2 = new Spark(Constants.PWMPorts.kRight2Port);

        m_rightEncoder.setReverseDirection(true);
        m_leftEncoder.setDistancePerPulse(0.00920361328125);
        m_rightEncoder.setDistancePerPulse(0.00920361328125);

        m_leftMotors = new MotorControllerGroup(m_left1, m_left2);
        m_rightMotors = new MotorControllerGroup(m_right1, m_right2);

        m_leftMotors.setInverted(true);
        m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

        try {
            m_gyro = new AHRS(Constants.kGyroPort);
            m_gyro.setAngleAdjustment(180);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }
        // m_turnController = new PIDController(Constants.Drive.kTurnP, Constants.Drive.kTurnI, Constants.Drive.kTurnD);
        // m_turnController.enableContinuousInput(-180.0f, 180.0f);
        // m_turnController.setTolerance(Constants.Drive.kToleranceDegrees);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Angle", m_gyro.getAngle() + "");
        // SmartDashboard.putBoolean("At Setpoint", m_turnController.atSetpoint());
        SmartDashboard.putNumber("Left Distance", m_leftEncoder.getDistance());
        SmartDashboard.putNumber("Right Distance", m_rightEncoder.getDistance());
        SmartDashboard.putNumber("Left Speed", m_leftEncoder.getRate());
        SmartDashboard.putNumber("Right Speed", m_rightEncoder.getRate());
        SmartDashboard.putNumber("Left Raw", m_leftEncoder.getRaw());
        SmartDashboard.putNumber("Right Raw", m_rightEncoder.getRaw());
    }

    public void reset() {
        if (m_gyro != null)
            m_gyro.reset();
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    public void resetGyro() {
        if (m_gyro != null){
            System.out.println("Reset gyro");
            m_gyro.reset();
        }
    }

    public double getForwardOrient() {
        System.out.println(forwardOrient);
        if (forwardOrient == FORWARD_ORIENT_NOT_SET) {
            this.forwardOrient = getHeading();
        }
        return forwardOrient;
    }

    public void resetForwardOrient() {
        forwardOrient = FORWARD_ORIENT_NOT_SET;
    }

    public double getDistance() {
        // return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2;
        if (m_leftEncoder.getDistance() > m_rightEncoder.getDistance()) return m_leftEncoder.getDistance();
        return m_rightEncoder.getDistance();
    }

    public double getLeftDistance() {
        return m_leftEncoder.getDistance();
    }

    public double getRightDistance() {
        return m_rightEncoder.getDistance();
    }

    public void setLeftMotors(double speed) {
        m_leftMotors.set(speed);
    }

    public void setRightMotors(double speed) {
        m_rightMotors.set(speed);
    }

    public void tankDrive(double left, double right) {
        m_drive.tankDrive(left, right);
    }

    public double getHeading() {
        if (m_gyro != null)
            return m_gyro.getYaw();
        return 0;
    }

    public void arcadeDrive(double xSpeed, double zRotation, double limit) {
        m_drive.arcadeDrive(xSpeed, Util.limit(zRotation, limit));
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        m_drive.arcadeDrive(xSpeed, zRotation);
    }
}
