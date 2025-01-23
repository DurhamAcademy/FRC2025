package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {}

    /** Update the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double voltage) {}
}
