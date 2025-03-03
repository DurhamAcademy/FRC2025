package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOSparkMax implements IntakeIO {
    private final SparkMax intakeMotor;
    private final SparkMax rotatorMotor;
    private final SparkClosedLoopController rotatorController;
    private final SparkMaxConfig intakeResetConfig = new SparkMaxConfig();
    private final SparkMaxConfig rotatorResetConfig = new SparkMaxConfig();
    private final RelativeEncoder intakeEncoder;
    private final RelativeEncoder rotatorRelativeEncoder;
    private final SparkAbsoluteEncoder rotatorAbsoluteEncoder;
    private final DigitalInput beamBreakSensor;

    public IntakeIOSparkMax() {
        intakeMotor =
                new SparkMax(IntakeConstants.intakeMotorId, SparkLowLevel.MotorType.kBrushless);
        rotatorMotor =
                new SparkMax(IntakeConstants.rotatorMotorId, SparkLowLevel.MotorType.kBrushless);

        intakeEncoder = intakeMotor.getEncoder();
        rotatorRelativeEncoder = rotatorMotor.getEncoder();
        rotatorAbsoluteEncoder = rotatorMotor.getAbsoluteEncoder();

        intakeResetConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        intakeResetConfig.smartCurrentLimit(40);
        intakeResetConfig.voltageCompensation(12.0);
        intakeResetConfig.inverted(true);

        rotatorResetConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        rotatorResetConfig.smartCurrentLimit(40);
        rotatorResetConfig.voltageCompensation(12.0);
        rotatorResetConfig
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .pid(IntakeConstants.rotatorKp, IntakeConstants.rotatorKi, IntakeConstants.rotatorKd);

        rotatorResetConfig.encoder.positionConversionFactor(
                IntakeConstants.rotatorGearRatio); // update gear ratio

        intakeMotor.configure(intakeResetConfig, SparkBase.ResetMode.kResetSafeParameters, null);
        rotatorMotor.configure(rotatorResetConfig, SparkBase.ResetMode.kResetSafeParameters, null);

        rotatorController = rotatorMotor.getClosedLoopController();

        beamBreakSensor = new DigitalInput(IntakeConstants.beamBreakId);
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        inputs.intakeVelocityRadPerSec =
                intakeEncoder.getVelocity() * (2 * Math.PI / 60); // Convert RPM to rad/sec
        inputs.intakeAppliedVolts = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
        inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
        inputs.intakeTemperature = intakeMotor.getMotorTemperature();

        // rotation in radians
        inputs.rotatorPosRad = rotatorAbsoluteEncoder.getPosition() * (2 * Math.PI);
        inputs.rotatorVelocityRadPerSec =
                rotatorRelativeEncoder.getVelocity() * (2 * Math.PI / 60); // Convert RPM to rad/sec
        inputs.rotatorAppliedVolts = rotatorMotor.getBusVoltage() * rotatorMotor.getAppliedOutput();
        inputs.rotatorCurrentAmps = rotatorMotor.getOutputCurrent();
        inputs.rotatorTemperature = rotatorMotor.getMotorTemperature();

        inputs.isBeamBroken = !beamBreakSensor.get(); // needs to be inverted
    }

    @Override
    public void setIntakePercent(double percent) {
        intakeMotor.set(percent);
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeMotor.setVoltage(volts);
    }

    @Override
    public void setRotatorVoltage(double volts) {
        rotatorMotor.setVoltage(volts);
    }

    @Override
    public void setRotatorReference(double position, double ffVolts) {
        rotatorController.setReference(
                position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVolts);
    }
}
