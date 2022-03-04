package frc.robot;

public class Util {
    
    public static double limit(double velocity, double limit) {
        if (velocity > limit)
            return limit;
        if (velocity < -limit)
            return -limit;
        return velocity;
    }

}
