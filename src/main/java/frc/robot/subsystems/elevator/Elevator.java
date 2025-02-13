package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    private final WristIO wristIO;
    private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

    private boolean hasZeroed = true;

    public enum ElevatorLevel {
        ZERO(ElevatorConstants.ELEVATOR_ZERO, WristConstants.WRIST_ANGLE_ZERO),
        INTAKE(ElevatorConstants.ELEVATOR_ZERO, WristConstants.WRIST_ANGLE_INTAKE),
        L1(ElevatorConstants.ELEVATOR_L1, WristConstants.WRIST_ANGLE_L1),
        L2(ElevatorConstants.ELEVATOR_L2, WristConstants.WRIST_ANGLE_L2),
        L3(ElevatorConstants.ELEVATOR_L3, WristConstants.WRIST_ANGLE_L3),
        L4(ElevatorConstants.ELEVATOR_L4, WristConstants.WRIST_ANGLE_L4);

        public final double heightInches;
        public final double angleRadians;

        ElevatorLevel(double heightInches, double angleRadians) {
            this.heightInches = heightInches;
            this.angleRadians = angleRadians;
        }
    }

    public Elevator(ElevatorIO elevatorIO, WristIO wristIO) {
        this.elevatorIO = elevatorIO;
        this.wristIO = wristIO;
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorInputs);
        wristIO.updateInputs(wristInputs);
        Logger.processInputs("Elevator", elevatorInputs);

        //        if (inputs.isLimitSwitchPressed) {
        //            io.setEncoder(ElevatorConstants.minHeight * ElevatorConstants.countsPerInch);
        //            hasZeroed = true;
        //        }

        elevatorIO.updateProfile();
        wristIO.updateProfile();
    }

    public void setElevatorTargetHeight(double heightInches) {
        elevatorIO.setTargetHeightInches(heightInches);
    }

    /**
     * Sets the target angle of the wrist
     *
     * @param targetAngle in radians, 0 being horizontal with the ground
     */
    public void setWristTargetAngle(double targetAngle) {
        wristIO.setTargetAngle(targetAngle);
    }

    /**
     * Sets the speed of the wrist
     * @param speed The desired speed level for the motor: -1.0 => full reverse, 1.0 => full forward, 0.0 => no speed.
     */
    public void setWristSpeed(double speed) {
        wristIO.setSpeed(speed);
    }

    /**
     * Sets the voltage of the wrist
     * @param voltage in volts
     */
    public void setWristVoltage(double voltage) {
        wristIO.setVoltage(voltage);
    }

    public void setElevatorPower(double power) {
        elevatorIO.setPower(power);
    }

    public boolean hasZeroed() {
        return hasZeroed;
    }

    public boolean isZeroed() {
        return elevatorInputs.isLimitSwitchPressed;
    }
}
