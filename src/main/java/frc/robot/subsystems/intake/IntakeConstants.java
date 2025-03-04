package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

public class IntakeConstants {
    public static final double rotatorKs = 0.0;
    public static final double rotatorKg = 0.0;
    public static final double rotatorKv = 0.0;
    public static final double rotatarKa = 0.0;

    public static final double rotatorGearRatio = 2.0;
    public static final double rotatorKp = 0.0;
    public static final double rotatorKi = 0.0;
    public static final double rotatorKd = 0.0;

    public static final double rotatorMaxVelocity = 1.0;
    public static final double rotatorMaxAcceleration = 1.0;

    // the intake's natural position
    // todo: update this
    public static final Rotation2d INTAKE_RESTING_ROTATION = new Rotation2d(0);
    // the intake's "up" position, to reveal climber
    // todo: update this
    public static final Rotation2d INTAKE_UP_ROTATION = new Rotation2d(0);

    public static final int intakeMotorId = 21;
    public static final int rotatorMotorId = 20;
    public static final int beamBreakId = 7;

    // sim stuff
    // TODO: update for calculating sim intake
    public static final Distance intakeWidth = Meters.of(1);
    public static final Distance intakeExtensionLength = Meters.of(0.1);
    public static final Translation3d intakePositionOnRobot = new Translation3d();
    public static final Pose3d intakeRange = new Pose3d();
}
