package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.Logger;

public class WristIOSparkMax implements WristIO {
    private final SparkMax wristMotor;
    private final SparkClosedLoopController wristController;
    private final RelativeEncoder wristEncoder;
    private final SparkMaxConfig resetConfig = new SparkMaxConfig();

    private final TrapezoidProfile.Constraints constraints;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;
    private final ArmFeedforward feedForward;

    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);

    public WristIOSparkMax() {
        wristMotor = new SparkMax(ElevatorConstants.wristCanId, SparkLowLevel.MotorType.kBrushless);
        wristEncoder = wristMotor.getEncoder();
        wristController = wristMotor.getClosedLoopController();

        resetConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        resetConfig.smartCurrentLimit(30);
        resetConfig.voltageCompensation(12.0);
        resetConfig
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .pid(
                        ElevatorConstants.wristKp,
                        ElevatorConstants.wristKi,
                        ElevatorConstants.wristKd);
        wristMotor.configure(resetConfig, SparkBase.ResetMode.kResetSafeParameters, null);

        constraints =
                new TrapezoidProfile.Constraints(
                        ElevatorConstants.wristMaxVelocity, ElevatorConstants.wristMaxAcceleration);
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
        profile = new TrapezoidProfile(constraints);

        feedForward =
                new ArmFeedforward(
                        ElevatorConstants.wristKs,
                        ElevatorConstants.wristKg,
                        ElevatorConstants.wristKv,
                        ElevatorConstants.wristKa);
    }

    @Override
    public void setEncoder(double position) {
        wristEncoder.setPosition(position);
    }

    @Override
    public void setPower(double power) {
        wristMotor.set(power);
    }

    @Override
    public void setVoltage(double voltage) {
        wristMotor.setVoltage(voltage);
    }

    @Override
    public void stopMotors() {
        wristMotor.set(0);
    }

    @Override
    public void updateInputs(WristIO.WristIOInputs inputs) {
        inputs.angle = Rotation2d.fromRotations(wristEncoder.getPosition());
        inputs.targetAngle = targetAngle;
        inputs.velocity = wristEncoder.getVelocity();
        // TODO adjust how precise angle needs to be
        inputs.isAtTargetAngle =
                Math.abs(inputs.angle.getDegrees() - inputs.targetAngle.getDegrees()) < 2
                        && Math.abs(inputs.velocity) < 0.1;
        inputs.voltage = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
    }

    @Override
    public void setTargetAngle(double targetAngle) {
        this.targetAngle = Rotation2d.fromRadians(targetAngle);
        goalState = new TrapezoidProfile.State(targetAngle, 0);
    }

    @Override
    public void updateProfile() {
        // Calculate the next state (position and velocity)
        currentState = profile.calculate(0.02, currentState, goalState);
        // TODO double check these are supposed to be current state not goal state
        double ffVolts = feedForward.calculate(currentState.position, currentState.velocity);

        // Use the profiler's position as the target for the motor controller
        Logger.recordOutput("Wrist/ProfilerVelocity", currentState.velocity);
        Logger.recordOutput("Wrist/ProfilerPosition", currentState.position);

        wristController.setReference(
                currentState.position,
                SparkBase.ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                ffVolts);
    }
}
