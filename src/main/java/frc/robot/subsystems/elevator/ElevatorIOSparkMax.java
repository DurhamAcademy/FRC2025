package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorIOSparkMax implements ElevatorIO {
    // some credit to https://chiefdelphi.com/t/elevator-subsystem-example-code/482648
    private final SparkMax primaryMotor;
    private final SparkMax followerMotor;
    private final SparkClosedLoopController primaryController;
    private final RelativeEncoder primaryEncoder;
    private final RelativeEncoder followerEncoder;
    private final DigitalInput limitSwitch;
    private final SparkMaxConfig resetConfig = new SparkMaxConfig();
    private double targetHeightInches = 0.0;

    private final TrapezoidProfile.Constraints constraints;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;

    public ElevatorIOSparkMax() {
        // Primary motor = left motor
        primaryMotor = new SparkMax(ElevatorConstants.leftElevatorCanId, MotorType.kBrushless);
        followerMotor = new SparkMax(ElevatorConstants.rightElevatorCanId, MotorType.kBrushless);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(primaryMotor);

        followerMotor.configure(followerConfig, null, null);

        primaryEncoder = primaryMotor.getEncoder();
        followerEncoder = followerMotor.getEncoder();

        primaryController = primaryMotor.getClosedLoopController();

        limitSwitch = new DigitalInput(ElevatorConstants.limitSwitchPort);

        resetConfig.idleMode(IdleMode.kBrake);
        resetConfig.smartCurrentLimit(40);
        resetConfig.voltageCompensation(12.0);
        resetConfig
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .pidf(
                        ElevatorConstants.elevatorKp,
                        ElevatorConstants.elevatorKi,
                        ElevatorConstants.elevatorKd,
                        ElevatorConstants.elevatorFF);

        configureMotors();

        constraints = new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration);
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
        profile = new TrapezoidProfile(constraints);
    }

    private void configureMotors() {
        primaryMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);

        followerMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
    }

    @Override
    public void setEncoder(double position) {
        primaryEncoder.setPosition(position);
    }

    @Override
    public void setPower(double power) {
        primaryMotor.set(power);
    }

    @Override
    public void setVoltage(double voltage) {
        primaryMotor.setVoltage(voltage);
    }

    @Override
    public void stopMotors() {
        primaryMotor.set(0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.isLimitSwitchPressed = limitSwitch.get();
        inputs.leftHeightInches = primaryEncoder.getPosition() / ElevatorConstants.countsPerInch;
        inputs.rightHeightInches = followerEncoder.getPosition() / ElevatorConstants.countsPerInch;
        inputs.targetHeightInches = targetHeightInches;
        inputs.velocityInches = primaryEncoder.getVelocity() / ElevatorConstants.countsPerInch;
        inputs.isAtTargetLevel =
                Math.abs(inputs.leftHeightInches - targetHeightInches) < 0.5
                        && Math.abs(inputs.velocityInches) < 0.1;
        inputs.leftVoltage = primaryMotor.getBusVoltage();
        inputs.rightVoltage = followerMotor.getBusVoltage();
    }

    @Override
    public void setTargetHeightInches(double heightInches) {
        targetHeightInches =
                MathUtil.clamp(
                        heightInches, ElevatorConstants.minHeight, ElevatorConstants.maxHeight);

        goalState = new TrapezoidProfile.State(targetHeightInches * ElevatorConstants.countsPerInch, 0);
    }

    @Override
    public void updateProfile() {
        // Calculate the next state (position and velocity)
        currentState = profile.calculate(0.02, currentState, goalState);

        // Use the profiler's position as the target for the motor controller
        primaryController.setReference(
                currentState.position,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0
                // arbff?
        );
    }
}
