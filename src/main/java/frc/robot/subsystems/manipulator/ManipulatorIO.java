package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO {
    @AutoLog
    public static class ManipulatorIOInputs {
        public double rollerLVelocityRadPerSec;
        public double rollerLAppliedVolts = 0.0;
        public double rollerLCurrentAmps = 0.0;
        public double rollerLTemperature = 0.0;

        public double rollerRVelocityRadPerSec;
        public double rollerRAppliedVolts = 0.0;
        public double rollerRCurrentAmps = 0.0;
        public double rollerRTemperature = 0.0;

        public double sensorDistance = 0.0;
    }

    public default void updateInputs(ManipulatorIOInputs inputs) {}

    /** Set intake wheel voltage. */
    public default void setRollerPercent(double percent) {}

    public default void setRollerVoltage(double volts) {}

    public default void updateProfile() {}
}
