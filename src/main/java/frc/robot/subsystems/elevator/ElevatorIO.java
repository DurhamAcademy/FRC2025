package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean isLimitSwitchPressed;
        public double heightInches;
        public double targetHeightInches;
        public double velocityInches;
        public boolean isAtTargetLevel;
    }

    /** Update the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    /** Sets the target position of the elevator */
    public default void setTargetHeightInches(double targetHeightInches) {}

    /** Sets the motors' power */
    public default void setVoltage(double voltage) {}

    /** Stop motors */
    public default void stopMotors() {}

    /** Resets encoder */
    public default void resetEncoder() {}
}
