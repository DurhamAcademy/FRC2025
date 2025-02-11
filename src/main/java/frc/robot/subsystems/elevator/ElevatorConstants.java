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
    public static final double elevatorKs = 0.0; // static friction
    public static final double elevatorKg = 0.0; // gravity
    public static final double elevatorKv = 0.0; // velocity
    public static final double elevatorKa = 0.0; // acceleration

    public static final double wristKp = 1.0;
    public static final double wristKi = 0.0;
    public static final double wristKd = 0.0;

    public static final double maxWristPosition = 100.0;
    public static final double minWristPosition = 0.0;
    public static final double wristMaxVelocity = 10.0;
    public static final double wristMaxAcceleration = 10.0;

    // Elevator Levels in inches from master sketch
    public static final double ZERO = 0.0; // min height
    public static final double L1 = 9.271975;
    public static final double L2 = 17.031579;
    public static final double L3 = 32.041099;
    public static final double L4 = 58.211229; // max height

    public static final double elevatorMotorReduction = 2.0; // Motor gear reduction 22t -> 44t
    public static final double elevatorEffectiveDrumRadius =
            0.955 * 3; // Drum radius in inches multiplied by 3 because 3-stage elevator

    public static final double elevatorEncoderPositionFactor =
            (2 * Math.PI * elevatorEffectiveDrumRadius)
                    / elevatorMotorReduction; // rotations -> inches
    public static final double elevatorEncoderVelocityFactor =
            (2 * Math.PI * elevatorEffectiveDrumRadius)
                    / (60 * elevatorMotorReduction); // RPM -> inches/sec
    public static final double elevatorMaxVelocity = 10;
    public static final double elevatorMaxAcceleration = 10.0;
    public static final double minHeight = ZERO;
    public static final double maxHeight = L4;

    public static final double carriageWeightKg = 1.0;
}
