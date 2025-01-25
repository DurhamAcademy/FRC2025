package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final TrapezoidProfile profile;
    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goalState;
    private TrapezoidProfile.State currentState;

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

    public Elevator(ElevatorIO io) {
        this.io = io;

        constraints = new TrapezoidProfile.Constraints(
                ElevatorConstants.maxVelocity,
                ElevatorConstants.maxAcceleration
        );
        goalState = new TrapezoidProfile.State(0, 0);
        currentState = new TrapezoidProfile.State(0, 0);
        profile = new TrapezoidProfile(constraints);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        if (hasZeroed) {
            runElevator();
        }

        if (inputs.isLimitSwitchPressed) {
            io.resetEncoder();
            hasZeroed = true;
        }

        if (inputs.heightInches > ElevatorConstants.maxHeight) {
            io.stopMotors();
        }



    }

    public void setTargetHeight(double inches) {

    }

    public void runElevator() {
        if (inputs.heightInches > ElevatorConstants.maxHeight ||
                inputs.heightInches < ElevatorConstants.minHeight) {
            io.stopMotors();
            return;
        }

        currentState = profile.calculate(0.020, currentState, goalState);

        double targetVoltage = currentState.velocity * ElevatorConstants.elevatorFF;

        io.setVoltage(targetVoltage);
    }
}
