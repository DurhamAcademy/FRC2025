package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOSparkMax implements IntakeIO {
    private final SparkMax intakeMotor;
    private final SparkMaxConfig intakeResetConfig = new SparkMaxConfig();
    private final RelativeEncoder intakeEncoder;
    private final DigitalInput beamBreakSensor;

    public IntakeIOSparkMax() {
        intakeMotor =
                new SparkMax(IntakeConstants.intakeMotorId, SparkLowLevel.MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();

        intakeResetConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        intakeResetConfig.smartCurrentLimit(50);
        intakeResetConfig.inverted(true);

        intakeMotor.configure(intakeResetConfig, SparkBase.ResetMode.kResetSafeParameters, null);

        beamBreakSensor = new DigitalInput(IntakeConstants.beamBreakId);
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        inputs.intakeTemperature = intakeMotor.getMotorTemperature();
        inputs.intakeVelocityRadPerSec =
                intakeEncoder.getVelocity() * (2 * Math.PI / 60); // Convert RPM to rad/sec
        inputs.intakeAppliedVolts = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
        inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
        inputs.intakeTemperature = intakeMotor.getMotorTemperature();

        inputs.isBeamBroken = !beamBreakSensor.get(); // needs to be inverted
    }

    @Override
    public void stopMotors() {
        intakeMotor.set(0);
    }

    @Override
    public void setIntakePercent(double percent) {
        intakeMotor.set(percent);
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeMotor.setVoltage(volts);
    }
}
