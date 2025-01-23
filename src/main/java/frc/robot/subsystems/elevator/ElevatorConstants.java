package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    // Device CAN IDs
    // update when robot built
    public static final int leftElevatorCanId = 10;
    public static final int rightElevatorCanId = 11;

    // Limit Switch DIO #
    // update when robot built
    public static final int limitSwitchDIO = 20;

    // Elevator PID Configuration
    // Manually tune PID?
    public static final double elevatorKp = 0.0;
    public static final double elevatorKi = 0.0;
    public static final double elevatorKd = 0.0;

    // Elevator Levels
    public static final double zero = 0.0;
    public static final double intake = 0.0;
    public static final double L1 = 0.0;
    public static final double L2 = 0.0;
    public static final double L3 = 0.0;
    public static final double L4 = 0.0;
}
