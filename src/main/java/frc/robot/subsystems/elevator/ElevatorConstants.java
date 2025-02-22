package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    // Device CAN IDs
    public static final int leftElevatorCanId = 11;
    public static final int rightElevatorCanId = 10;

    // Limit Switch DIO #
    public static final int limitSwitchPort = 0;

    // Elevator PID Configuration
    // Manually tune PID?
    // TODO: FIGURE OUT
    public static final double elevatorKp = 1.0;
    public static final double elevatorKi = 0.0;
    public static final double elevatorKd = 0.0;
    // TODO: Elevator FF can be found through SysID
    public static final double elevatorKs = 0.0;
    public static final double elevatorKg = 0.09;
    public static final double elevatorKv = 0.14;
    public static final double elevatorKa = 0.0;

    public static final double wristKp = 1.0;
    public static final double wristKi = 0.0;
    public static final double wristKd = 0.0;

    public static final double maxWristPosition = 100.0;
    public static final double minWristPosition = 0.0;
    public static final double wristMaxVelocity = 1;
    public static final double wristMaxAcceleration = 1;

    // Elevator Levels in inches from master sketch
    public static final double ZERO = 0.0; // min height
    public static final double L1 = 12;
    public static final double L2 = 21.75;
    public static final double L3 = 37.5;
    public static final double L4 = 66; // max height

    // the distance between the floor and the bottom of the elevator
    public static final double elevatorBaseHeight = 4.750;

    public static final double elevatorMaxVelocity = 20;
    public static final double elevatorMaxAcceleration = 20.0;
    public static final double minHeight = ZERO;
    public static final double maxHeight = L4;

    public static final double elevatorMotorReduction =
            10.0; // Motor gear reduction 5:1 (NEO) and 2:1 (22t -> 44t)
    public static final double elevatorEffectiveDrumRadius =
            0.955 * 3; // Drum radius in inches multiplied by 3 because 3-stage elevator
    public static final double elevatorEncoderPositionFactor =
            (2 * Math.PI * elevatorEffectiveDrumRadius)
                    / (elevatorMotorReduction); // rotations -> inches
    public static final double elevatorEncoderVelocityFactor =
            (2 * Math.PI * elevatorEffectiveDrumRadius)
                    / (60 * elevatorMotorReduction); // RPM -> inches/sec
}
