package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private double intakeVoltageSetpoint = 0.0;
    IntakeIO io;
    IntakeIO.IntakeIOInputs inputs;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void setVoltage(double voltage) {
        intakeVoltageSetpoint = voltage;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.setIntakeVoltage(intakeVoltageSetpoint);
    }
}
