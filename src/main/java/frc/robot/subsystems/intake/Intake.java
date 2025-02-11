package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private double intakeVoltageSetpoint = 0.0;
    IntakeIO io;
    IntakeIOInputsAutoLogged inputs;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void setVoltage(double voltage) {
        intakeVoltageSetpoint = voltage;
    }

    public void simAddCoral(Pose2d robotPose) {
        io.simAddCoral(robotPose);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.setIntakeVoltage(intakeVoltageSetpoint);
        Logger.recordOutput("Intake/isRunning", intakeVoltageSetpoint != 0);
    }
}
