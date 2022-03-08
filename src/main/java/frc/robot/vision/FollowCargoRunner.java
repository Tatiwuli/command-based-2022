package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FollowCargoRunner implements Runnable {
    
    private CargoColor cargoColor;
    private FollowCargoPipeline pipeline = new FollowCargoPipeline();
    private double centerX, centerY, area;
    private boolean detected;
    private CvSink cvSink;
    private CvSource outputStream, outputStreamTresh;
    private static final int HEIGTH = 120;
    private static final int WIDTH = 160;
    private double[] hslThresholdHue = new double[2];
    private double[] hslThresholdSaturation = {68.0, 255.0};
    private double[] hslThresholdLuminance = {45.0, 180.0}; // Todo Testar na competição: 160, 200?
    
    public FollowCargoRunner(CargoColor cargoColor) {
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(160, 120);
        this.cargoColor = cargoColor;

        cvSink = CameraServer.getVideo();
        outputStream = CameraServer.putVideo("Cargo", HEIGTH, WIDTH);
        outputStreamTresh = CameraServer.putVideo("Tresh", HEIGTH, WIDTH);
    }

    public boolean isDetected() {
        return detected;
    }

    public double getArea() {
        return area;
    }

    public double getCenterY() {
        return centerY-(HEIGTH/2);
    }

    public double getCenterX() {
        return centerX-(WIDTH/2);
    }

    public void setCargoColor(CargoColor cargoColor) {
        this.cargoColor = cargoColor;
    }
    
    @Override
    public void run() {
        Point cvAnchor = new Point(-1, -1);
		Scalar cvBordervalue = new Scalar(-1);
        Mat frame = new Mat();
        Mat mat = new Mat();
        
        while (!Thread.interrupted()) {
            if (cargoColor == CargoColor.BLUE) {
                hslThresholdHue[0] = 100;
                hslThresholdHue[1] = 135;
            } else if (cargoColor == CargoColor.RED) {
                hslThresholdHue[0] = 0;
                hslThresholdHue[1] = 30;
            } else if (cargoColor == CargoColor.YELLOW) {
                hslThresholdHue[0] = 30;
                hslThresholdHue[1] = 80;
            } 

            if (cvSink.grabFrame(frame) == 0) {
              outputStream.notifyError(cvSink.getError());
              continue;
            }

            pipeline.hslThreshold(frame, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, mat);
            // pipeline.blur(mat, BlurType.GAUSSIAN, 1, mat);
            pipeline.cvErode(mat, null, cvAnchor, 2, Core.BORDER_CONSTANT, cvBordervalue, mat);
            pipeline.cvDilate(mat, null, cvAnchor, 7, Core.BORDER_CONSTANT, cvBordervalue, mat);
            outputStreamTresh.putFrame(mat);

            Mat hierarchey = new Mat();
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mat, contours, hierarchey, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Point center = new Point();
            float[] radius = {0};
            if (contours.size() > 0) {
                MatOfPoint2f contour = new MatOfPoint2f(contours.get(0).toArray());
                for (MatOfPoint m : contours) {
                    if (Imgproc.contourArea(new MatOfPoint2f(m.toArray())) > Imgproc.contourArea(contour)) {
                        contour = new MatOfPoint2f(m.toArray());
                    }
                }
                Imgproc.minEnclosingCircle(contour, center, radius);
                double area = Imgproc.contourArea(contour);
                double arcLength = Imgproc.arcLength(contour, true);
                double circulariy =  4 * Math.PI * area / (arcLength * arcLength); 
                SmartDashboard.putNumber("[Camera frontal] Area", area);
                SmartDashboard.putNumber("[Camera frontal] Circularidade", circulariy);
                this.area = area;
                if (circulariy > 0.5) {
                    Moments moments = Imgproc.moments(contour);
                    center = new Point((int)(moments.m10 / moments.m00), (int)(moments.m01 / moments.m00));
                    if (radius[0] > 1) {
                        Imgproc.circle(frame, center, (int) radius[0], new Scalar(0, 255, 0));
                    } else {
                        detected = false;
                    }
                    if ((area > 250 && circulariy > 0.8) || area > 2000 && circulariy > 0.55) {
                        detected = true;
                        this.centerX = center.x;
                        this.centerY = center.y;
                    } else {
                        detected = false;
                    }
                } else {
                    detected = false;
                }
            } else {
                detected = false;
            }
            SmartDashboard.putBoolean("[Camera frontal] Detected", detected);
            SmartDashboard.putNumber("[Camera frontal] Center", centerX);
            SmartDashboard.putNumber("[Camera frontal] Center PID", getCenterX());
            outputStream.putFrame(frame);
        } // End While
                
    }
}
