package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private boolean hasZeroed = true;

    public enum ElevatorLevel {
        ZERO(ElevatorConstants.ZERO),
        L1(ElevatorConstants.L1),
        L2(ElevatorConstants.L2),
        L3(ElevatorConstants.L3),
        L4(ElevatorConstants.L4);

        public final double heightInches;

        ElevatorLevel(double heightInches) {
            this.heightInches = heightInches;
        }
    }

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        //        if (inputs.isLimitSwitchPressed) {
        //            io.setEncoder(ElevatorConstants.minHeight * ElevatorConstants.countsPerInch);
        //            hasZeroed = true;
        //        }

        io.updateState();
    }

    public void setTargetHeight(double heightInches) {
        io.setTargetHeightInches(heightInches);
    }

    public void setPower(double power) {
        io.setPower(power);
    }

    public boolean hasZeroed() {
        return hasZeroed;
    }

    public boolean isZeroed() {
        return inputs.isLimitSwitchPressed;
    }
}
