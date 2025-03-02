package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
    private final ManipulatorIO io;
    private final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();

    public Manipulator(ManipulatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Manipulator", inputs);
    }

    public void setVoltage(double voltage) {
        io.setRollerVoltage(voltage);
    }

    public void setPower(double power) {
        io.setRollerPercent(power);
    }

    /*@AutoLogOutput
    public BooleanSupplier beamBroken() {
        return () -> inputs.beamObstructed;
    }*/

    public boolean beamBroken() {
        return inputs.sensorDistance < ManipulatorConstants.maxCoralSensorDistance;
    }
}
