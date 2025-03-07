package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

public class IntakeConstants {
    public static final int intakeMotorId = 21;
    public static final int beamBreakId = 7;

    // sim stuff
    // TODO: update for calculating sim intake
    public static final Distance intakeWidth = Meters.of(1);
    public static final Distance intakeExtensionLength = Meters.of(0.1);
    public static final Translation3d intakePositionOnRobot = new Translation3d();
    public static final Pose3d intakeRange = new Pose3d();
}
