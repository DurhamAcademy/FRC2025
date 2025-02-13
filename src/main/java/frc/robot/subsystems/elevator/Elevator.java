package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private boolean hasZeroed = false;

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

    SysIdRoutine sysIdRoutine;

    public Elevator(ElevatorIO io) {
        this.io = io;
        sysIdRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state ->
                                        Logger.recordOutput(
                                                "Elevator/SysIdTestState", state.toString()))),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> io.setVoltage(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        if (isZeroed()) {
            io.setEncoder(ElevatorConstants.minHeight);
            if(!hasZeroed) hasZeroed = true;
        }

        io.updateProfile();
    }

    public void setTargetHeight(double heightInches) {
        io.setTargetHeightInches(heightInches);
    }

    public void setPower(double power) {
        io.setPower(power);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    /**
     * Whether the elevator has found its zero at least once
     * @return boolean
     */
    public boolean hasZeroed(){
        return hasZeroed;
    }

    /**
     * Whether the elevator is currently zeroed
     * @return boolean
     */
    public boolean isZeroed() {
        return inputs.isLimitSwitchPressed;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    /*
    TODO this is sys ID stuff to do later
    operatorController
                .povUp()
                .whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        operatorController
                .povDown()
                .whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        operatorController
                .povLeft()
                .whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
        operatorController
                .povRight()
                .whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
     */
}
