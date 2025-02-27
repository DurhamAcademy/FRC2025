package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double intakeVelocityRadPerSec;
        public double intakeAppliedVolts = 0.0;
        public double intakeCurrentAmps = 0.0;
        public double intakeTemperature = 0.0;

        public double rotatorVelocityRadPerSec = 0.0;
        public double rotatorAppliedVolts = 0.0;
        public double rotatorCurrentAmps = 0.0;
        public double rotatorTemperature = 0.0;
        public double rotatorPosRad = 0.0;

        public boolean isBeamBroken = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputsAutoLogged inputs) {}

    /** Set intake wheel voltage. */
    public default void setIntakePercent(double percent) {}

    public default void setIntakeVoltage(double volts) {}

    public default void setRotatorVoltage(double volts) {}

    public default void setRotatorReference(double position, double ffVolts) {}

    public default void simAddCoral(Pose2d robotPose) {}
}
