package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double velocityRadPerSec;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double temperature = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {}

    /** Set intake wheel voltage. */
    public default void setIntakePercent(double percent) {}

    public default void setIntakeVoltage(double volts) {}

    public default void simAddCoral(Pose2d robotPose) {}
}
