package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    // Device CAN IDs
    // todo is this facing from the front or the back?
    public static final int leftElevatorCanId = 10;
    public static final int rightElevatorCanId = 11;

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

    // Elevator Levels in inches
    public static final double ELEVATOR_ZERO = 0.0; // min height
    public static final double ELEVATOR_L1 = 3.0;
    public static final double ELEVATOR_L2 = 8.0;
    public static final double ELEVATOR_L3 = 13.5;
    public static final double ELEVATOR_L4 = 21.0; // max height

    public static final double elevatorMaxVelocity = 10.0; // in/s
    public static final double elevatorMaxAcceleration = 10.0; // in/s
    public static final double minHeight = ELEVATOR_ZERO;
    public static final double maxHeight = ELEVATOR_L4;
    public static final double countsPerInch = 35;
}
