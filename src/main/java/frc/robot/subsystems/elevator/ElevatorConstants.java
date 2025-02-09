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

    public static final double maxWristPosition = 100.0;
    public static final double minWristPosition = 0.0;
    public static final double wristMaxVelocity = 10.0;
    public static final double wristMaxAcceleration = 10.0;



    // Elevator Levels in inches
    public static final double ZERO = 0.0; // min height
    public static final double L1 = 3.0;
    public static final double L2 = 8.0;
    public static final double L3 = 13.5;
    public static final double L4 = 21.0; // max height

    // TODO: MEASURE
    public static final double elevatorMaxVelocity = 10.0;
    public static final double elevatorMaxAcceleration = 10.0;
    public static final double minHeight = ZERO;
    public static final double maxHeight = L4;
    public static final double countsPerInch = 35;
}
