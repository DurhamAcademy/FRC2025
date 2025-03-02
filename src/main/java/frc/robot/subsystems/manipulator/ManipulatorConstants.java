package frc.robot.subsystems.manipulator;

public class ManipulatorConstants {
    // Device CAN IDs
    // update when robot built
    public static final int MANIPULATOR_ROLLERL_CanId = 31;
    public static final int MANIPULATOR_ROLLERR_CanId = 30;
    public static final int MANIPULATOR_BEAM_ID = 7;
    public static final int MANIPULATOR_DISTANCE_SENSOR_ID = 0;

    // the gear ratio of the gear box
    public static final double manipulatorGearRatio = 1;

    // Manipulator PID Configuration
    // TODO: FIGURE OUT
    public static final double manipulatorKp = 1;
    public static final double manipulatorKi = 0.0;
    public static final double manipulatorKd = 0.0;
    // Manipulator FF can be found through SysID
    public static final double manipulatorKs = 0.0;
    public static final double manipulatorKg = 0.0;
    public static final double manipulatorKv = 0.0;
    public static final double manipulatorKa = 0.0;

    // TODO: MEASURE
    public static final double maxVelocity = 0.0;
    public static final double maxAcceleration = 0.0;

    public static final double maxCoralSensorDistance = 0.0;
}
