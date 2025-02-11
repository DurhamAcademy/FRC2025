package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class IntakeConstants {
    public static final double intakeKp = 0.0;
    public static final double intakeKi = 0.0;
    public static final double intakeKd = 0.0;

    public static final double intakeKs = 0.0;
    public static final double intakeKv = 0.0;
    public static final double intakeKa = 0.0;

    public static final int motorId = 0; // TODO: update

    // sim stuff
    public static final Distance intakeWidth = Meters.of(1); // TODO: make real
    public static final Distance intakeExtensionLength =
            Meters.of(0.1); // TODO: make this real, if you set it 0 it is not happy
}
