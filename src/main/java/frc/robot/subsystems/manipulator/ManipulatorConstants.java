package frc.robot.subsystems.manipulator;

public class ManipulatorConstants {
    // Device CAN IDs
    public static final int leftManipulatorRollerCanId = 31;
    public static final int rightManipulatorRollerCanId = 30;
    public static final int manipulatorDistanceSensorPort = 3;

    // the gear ratio of the gear box
    public static final double manipulatorGearRatio = 1;

    // Manipulator PID Configuration
    public static final double manipulatorKp = 1;
    public static final double manipulatorKi = 0.0;
    public static final double manipulatorKd = 0.0;

    public static final double manipulatorKs = 0.0;
    public static final double manipulatorKv = 0.0;
    public static final double manipulatorKa = 0.0;

    // TODO: MEASURE
    public static final double maxVelocity = 3.0;
    public static final double maxAcceleration = 3.0;

    public static final double maxCoralSensorDistance = 2.65;
    public static final double sensorDistanceTolerance = 0.15; // allow some tolerance
}
