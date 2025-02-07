package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        Logger.processInputs("Elevator", inputs);
        io.updateProfile();
    }

    public void setVoltage(double voltage) {
        io.setRollerVoltage(voltage);
    }

    public void setPower(double power) {
        io.setRollerPercent(power);
    }
}
