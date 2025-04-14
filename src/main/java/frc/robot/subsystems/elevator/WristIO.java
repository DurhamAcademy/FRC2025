package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    /**
     * Represents a data container for the inputs to the wrist subsystem. Used to track the state of
     * the wrist, including its current angle, target angle, velocity, voltage, and whether it has
     * reached its target angle.
     */
    @AutoLog
    public static class WristIOInputs {
        public double angle;
        public double targetAngle;
        public double velocity;
        public double voltage;
        public boolean isAtTargetAngle;
        public double wristTemperature = 0;
    }

    /** Update the set of loggable inputs. */
    public default void updateInputs(WristIOInputs inputs) {}

    /** Sets the motors' power */
    public default void setVoltage(double voltage) {}

    /** Sets the target angle */
    public default void setTargetAngle(double targetAngle) {}

    /** Stop motors */
    public default void stopMotors() {}

    /** Updating trapezoid profiler and reference height using the profiler */
    public default void updateStates() {}
}
