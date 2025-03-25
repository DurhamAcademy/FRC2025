package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
    private final ManipulatorIO io;
    private final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();
    public double rollerHoldPositionRad = 0.0;
    private final double INTAKE_NUM_ROTATIONS = 5.0;

    public Manipulator(ManipulatorIO io) {
        this.io = io;
    }

    public void setGoalState(double goalPosition, double goalVelocity) {
        io.setGoalState(goalPosition, goalVelocity);
    }

    public void updateProfile() {
        io.updateProfile();
    }

    public void lockToCurrentPosition() {
        rollerHoldPositionRad = inputs.rollerRPosRad;
        setGoalState(rollerHoldPositionRad, 0);
    }

    public void setIntakingRollerPosition() {
        rollerHoldPositionRad = inputs.rollerRPosRad + INTAKE_NUM_ROTATIONS;
        setGoalState(rollerHoldPositionRad, 0);
    }

    public boolean isAtSetpoint() {
        return inputs.isAtSetpoint;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Manipulator", inputs);
        Logger.recordOutput("Manipulator/beamBroken", beamBroken());
    }

    public void setVoltage(double voltage) {
        io.setRollerVoltage(voltage);
    }

    public void stopMotors() {
        io.stopMotors();
    }

    public void setPower(double power) {
        io.setRollerPercent(power);
    }

    /*@AutoLogOutput
    public BooleanSupplier beamBroken() {
        return () -> inputs.beamObstructed;
    }*/

    public boolean beamBroken() {
        return inputs.sensorDistance
                > ManipulatorConstants.maxCoralSensorDistance
                        - ManipulatorConstants.sensorDistanceTolerance; // sensor is inverted
    }
}
