package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    IntakeIO io;
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public boolean getBeamBroken() {
        return inputs.isBeamBroken; // if this becomes noisy we can add a debouncer
    }

    public void setVoltage(double voltage) {
        io.setIntakeVoltage(voltage);
    }

    public void stopMotors() {
        io.stopMotors();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.recordOutput("Intake/beamBreakBroken", getBeamBroken());
        Logger.processInputs("Intake", inputs);
    }
}
