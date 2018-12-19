package frc.robot;

public final class Constants {
    public static final double wheelBase = 0.71;

    public static final int leftMotorChannel = 10;
    public static final int rightMotorChannel = 11;

    public static final double frequency = 0.02;
    public static final double maxSpeed = 3.3;
    public static final double acceleration = 1.0;
    public static final double jerk = 10.0;
    
    public static final int encoderTicksPerRotation = 4096;
    public static final double wheelDiameter = 0.1016;

    public static final double kP = 1.0; // The proportional term. This is usually quite high (0.8 - 1.0 are common values)
    public static final double kI = 0.0; // The integral term. Currently unused.
    public static final double kD = 0.0; // The derivative term. Adjust this if you are unhappy with the tracking of the follower. 0.0 is the default
    public static final double kV = 1.0 / Constants.maxSpeed; // The velocity ratio. This should be 1 over your maximum velocity @ 100% throttle.  This converts m/s given by the algorithm to a scale of -1..1 to be used by your motor controllers
    public static final double kA = 0.0;  // The acceleration term. Adjust this if you want to reach higher or lower speeds faster. 0.0 is the default
}