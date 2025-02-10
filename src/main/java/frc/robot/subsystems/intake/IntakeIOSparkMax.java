package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class IntakeIOSparkMax implements IntakeIO {
    private SparkMax intakeMotor;
    private SparkMaxConfig resetConfig = new SparkMaxConfig();
    private SimpleMotorFeedforward feedforward;
    private RelativeEncoder intakeEncoder;


    public IntakeIOSparkMax() {
        intakeMotor = new SparkMax(IntakeConstants.motorId, SparkLowLevel.MotorType.kBrushless);

        resetConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        resetConfig.smartCurrentLimit(40);
        resetConfig.voltageCompensation(12.0);
        resetConfig
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .pid(
                        IntakeConstants.intakeKp,
                        IntakeConstants.intakeKi,
                        IntakeConstants.intakeKd);

        intakeMotor.configure(resetConfig, SparkBase.ResetMode.kResetSafeParameters, null);

        feedforward =
                new SimpleMotorFeedforward(
                        IntakeConstants.intakeKs,
                        IntakeConstants.intakeKv,
                        IntakeConstants.intakeKa);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocityRadPerSec = intakeEncoder.getVelocity() * (2 * Math.PI / 60); // Convert RPM to rad/sec
        inputs.appliedVolts = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
        inputs.currentAmps = intakeMotor.getOutputCurrent();
        inputs.temperature = intakeMotor.getMotorTemperature();
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