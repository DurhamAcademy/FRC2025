package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean isLimitSwitchPressed;
        public double leftHeightInches;
        public double rightHeightInches;
        public double targetHeightInches;
        public double velocityInches;
        public double profilerHeightInches;
        public double profilerVelocityInches;
        public boolean isAtTargetLevel;
        public double leftVoltage;
        public double rightVoltage;
        public double current;
    }

    /** Update the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    /** Sets the motors' power */
    public default void setPower(double power) {}

    /** Sets the motors' power */
    public default void setVoltage(double voltage) {}

    /** Sets the motors' velocity */
    public default void setTargetHeightInches(double targetHeightInches) {}

    /** Stop motors */
    public default void stopMotors() {}

    /** Resets encoder */
    public default void setEncoder(double position) {}

    public default void updateStates() {}
}
