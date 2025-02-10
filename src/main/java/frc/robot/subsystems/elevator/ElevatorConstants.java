package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    // Device CAN IDs
    // todo is this facing from the front or the back?
    public static final int leftElevatorCanId = 10;
    public static final int rightElevatorCanId = 11;
    public static final int wristCanId = 12;

    // Limit Switch DIO #
    // update when robot built
    public static final int limitSwitchPort = 20;

    // Elevator PID Configuration
    // Manually tune PID?
    // TODO: FIGURE OUT
    public static final double elevatorKp = 1.0;
    public static final double elevatorKi = 0.0;
    public static final double elevatorKd = 0.0;
    // Elevator FF can be found through SysID
    public static final double elevatorKs = 0.0;
    public static final double elevatorKg = 0.0;
    public static final double elevatorKv = 0.0;
    public static final double elevatorKa = 0.0;

    public static final double wristKp = 1.0;
    public static final double wristKi = 0.0;
    public static final double wristKd = 0.0;

    public static final double wristKs = 0.0;
    public static final double wristKg = 0.0;
    public static final double wristKv = 0.0;
    public static final double wristKa = 0.0;

    // Elevator Levels in inches
    public static final double ELEVATOR_ZERO = 0.0; // min height
    public static final double ELEVATOR_L1 = 3.0;
    public static final double ELEVATOR_L2 = 8.0;
    public static final double ELEVATOR_L3 = 13.5;
    public static final double ELEVATOR_L4 = 21.0; // max height

    // Wrist angles in radians
    public static final double WRIST_ANGLE_ZERO = Math.toRadians(90.0); // min height
    public static final double WRIST_ANGLE_INTAKE = Math.toRadians(-40.0);
    public static final double WRIST_ANGLE_L1 = Math.toRadians(-30.0);
    public static final double WRIST_ANGLE_L2 = Math.toRadians(-35.0);
    public static final double WRIST_ANGLE_L3 = Math.toRadians(-35.0);
    public static final double WRIST_ANGLE_L4 = Math.toRadians(-25.0); // max height

    // TODO: MEASURE
    public static final double elevatorMaxVelocity = 10.0; // in/s
    public static final double elevatorMaxAcceleration = 10.0; // in/s
    public static final double minHeight = ELEVATOR_ZERO;
    public static final double maxHeight = ELEVATOR_L4;
    public static final double countsPerInch = 35;

    public static final double maxWristPosition = WRIST_ANGLE_ZERO;
    public static final double minWristPosition = WRIST_ANGLE_INTAKE;
    public static final double wristMaxVelocity = 10.0;
    public static final double wristMaxAcceleration = 10.0;

    public static final double wristMotorReduction = 28.0 / 18.0;
    public static final double wristEncoderPositionFactor = 2 * Math.PI / wristMotorReduction; // Rotations => Radians
    public static final double wristEncoderVelocityFactor = 2 * Math.PI / 60.0 / wristMotorReduction; // RPM => Radians per second

}
