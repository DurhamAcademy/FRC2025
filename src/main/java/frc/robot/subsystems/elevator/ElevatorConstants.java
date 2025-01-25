package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    // Device CAN IDs
    // update when robot built
    public static final int leftElevatorCanId = 10;
    public static final int rightElevatorCanId = 11;

    // Limit Switch DIO #
    // update when robot built
    public static final int limitSwitchPort = 20;

    // Elevator PID Configuration
    // Manually tune PID?
    // TODO: FIGURE OUT
    public static final double elevatorKp = 0.0;
    public static final double elevatorKi = 0.0;
    public static final double elevatorKd = 0.0;
    public static final double elevatorFF = 0.0;

    // Elevator Levels
    // TODO: level heights
    public static final double ZERO = 0.0; // min height
    public static final double L1 = 0.0;
    public static final double L2 = 0.0;
    public static final double L3 = 0.0;
    public static final double L4 = 0.0; // max height

    // TODO: MEASURE
    public static final double maxVelocity = 0.0;
    public static final double maxAcceleration = 0.0;
    public static final double minHeight =
            ZERO; // i wanted to add just for code clarity but idk if i should remove or nah
    public static final double maxHeight = L4;
    public static final double countsPerInch = 0.1;
}
