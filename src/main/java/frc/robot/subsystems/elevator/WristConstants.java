package frc.robot.subsystems.elevator;

public class WristConstants {
    public static final int wristCanId = 12;

    public static final double wristKp = 1.0;
    public static final double wristKi = 0.0;
    public static final double wristKd = 0.0;

    public static final double wristKs = 0.0;
    public static final double wristKg = 0.0;
    public static final double wristKv = 0.0;
    public static final double wristKa = 0.0;

    // Wrist angles in radians
    public static final double STARTING = Math.toRadians(90.0); // max angle
    public static final double ALGAE_IDLE = Math.toRadians(30.0);
    public static final double INTAKE = Math.toRadians(-40.0); // min angle
    public static final double L1 = Math.toRadians(-30.0);
    public static final double L2 = Math.toRadians(-35.0);
    public static final double L3 = Math.toRadians(-35.0);
    public static final double L4 = Math.toRadians(-25.0);
    public static final double LOWER_ALGAE_REMOVAL = Math.toRadians(0.0);
    public static final double UPPER_ALGAE_REMOVAL = Math.toRadians(0.0);
    public static final double NET = Math.toRadians(45.0);
    public static final double PROCESSOR = Math.toRadians(0);

    // inches
    public static final double WRIST_LENGTH = 11.875;

    // distance from wrist axle to L1 reef edge
    public static final double REEF_MIN_DISTANCE = 10.625;

    // the height of the reef panel april tag is attached on
    public static final double REEF_PANEL_HEIGHT = 14.25;

    // the distance from the base of the elevator (@ zero) to the center of the wrist axle
    public static final double WRIST_AXLE_HEIGHT = 14.824724;

    public static final double maxWristPosition = STARTING;
    public static final double minWristPosition = INTAKE;
    public static final double wristMaxVelocity = 10.0;
    public static final double wristMaxAcceleration = 10.0;

    // Tolerance of the wrist subsystem
    // degrees, rad/s
    public static final double wristAngularTolerance = 2;
    public static final double wristVelocityTolerance = 0.1;

    public static final double wristMotorReduction = 28.0 / 18.0;
    public static final double wristEncoderPositionFactor =
            2 * Math.PI / wristMotorReduction; // Rotations => Radians
    public static final double wristEncoderVelocityFactor =
            2 * Math.PI / 60.0 / wristMotorReduction; // RPM => Radians per second
}
