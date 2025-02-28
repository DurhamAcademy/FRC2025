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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSparkMax implements ElevatorIO {
    // some credit to https://chiefdelphi.com/t/elevator-subsystem-example-code/482648
    // spark max stuff
    private final SparkMax primaryMotor;
    private final SparkMax followerMotor;
    private final SparkClosedLoopController primaryController;
    private final RelativeEncoder primaryEncoder;
    private final RelativeEncoder followerEncoder;
    private final SparkMaxConfig resetConfig = new SparkMaxConfig();

    // limit switch
    private final DigitalInput limitSwitch;

    private double targetHeightInches = 0.0;

    // trapezoid profile stuff
    private final TrapezoidProfile.Constraints constraints;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;
    private final ElevatorFeedforward feedForward;

    /** Real IO Implementation for our elevator */
    public ElevatorIOSparkMax() {
        // Primary motor = left motor
        primaryMotor = new SparkMax(ElevatorConstants.leftElevatorCanId, MotorType.kBrushless);
        followerMotor = new SparkMax(ElevatorConstants.rightElevatorCanId, MotorType.kBrushless);

        primaryEncoder = primaryMotor.getEncoder();
        followerEncoder = followerMotor.getEncoder();

        primaryController = primaryMotor.getClosedLoopController();

        limitSwitch = new DigitalInput(ElevatorConstants.limitSwitchPort);

        resetConfig.idleMode(IdleMode.kBrake);
        resetConfig.smartCurrentLimit(40);
        resetConfig.voltageCompensation(12.0);
        resetConfig
                .encoder
                .positionConversionFactor(ElevatorConstants.elevatorEncoderPositionFactor)
                .velocityConversionFactor(ElevatorConstants.elevatorEncoderVelocityFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        resetConfig
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .pid(
                        ElevatorConstants.elevatorKp,
                        ElevatorConstants.elevatorKi,
                        ElevatorConstants.elevatorKd);

        configureMotors();

        // trapezoid movement profile best for elevator (trapezoid looking velocity graph)
        constraints =
                new TrapezoidProfile.Constraints(
                        ElevatorConstants.elevatorMaxVelocity, // in/s
                        ElevatorConstants.elevatorMaxAcceleration); // in/s
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
        profile = new TrapezoidProfile(constraints);

        // ff with values calculated by sysID
        feedForward =
                new ElevatorFeedforward(
                        ElevatorConstants.elevatorKs,
                        ElevatorConstants.elevatorKg,
                        ElevatorConstants.elevatorKv,
                        ElevatorConstants.elevatorKa);
    }

    /**
     * Configuring both motors with the same config but have the following motor follow the primary
     * motor
     */
    private void configureMotors() {
        primaryMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);

        // adding the follow config for the follower motor
        resetConfig.follow(primaryMotor, true);

        followerMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
    }

    @Override
    public void resetConfig() {
        resetConfig
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .pid(
                        ElevatorConstants.elevatorKp,
                        ElevatorConstants.elevatorKi,
                        ElevatorConstants.elevatorKd);

        configureMotors();
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
        Logger.recordOutput("Elevator/current", primaryMotor.getOutputCurrent());
        Logger.recordOutput("Elevator/rightcurrent", followerMotor.getOutputCurrent());
        inputs.isLimitSwitchPressed = !limitSwitch.get(); // limit switch is inverted
        inputs.leftHeightInches = primaryEncoder.getPosition();
        Logger.recordOutput("Elevator/primaryEncoder", primaryEncoder.getPosition());
        inputs.rightHeightInches = followerEncoder.getPosition();
        inputs.targetHeightInches = targetHeightInches;
        inputs.velocityInches = primaryEncoder.getVelocity();
        inputs.isAtTargetLevel =
                Math.abs(inputs.leftHeightInches - targetHeightInches) < 0.5
                        && Math.abs(inputs.velocityInches) < 0.1;
        inputs.leftVoltage = primaryMotor.getAppliedOutput() * primaryMotor.getBusVoltage();
        inputs.rightVoltage = followerMotor.getAppliedOutput() * followerMotor.getBusVoltage();
    }

    @Override
    public void setTargetHeightInches(double heightInches) {
        // ensuring that the target height is set between the min and max height
        targetHeightInches =
                MathUtil.clamp(
                        heightInches, ElevatorConstants.minHeight, ElevatorConstants.maxHeight);

        // setting the goal state of the trapezoid profile to the new target height
        goalState = new TrapezoidProfile.State(targetHeightInches, 0);
    }

    /**
     * Updating trapezoid profiler and reference height using the profiler during the elevator
     * subsystem's periodic function
     */
    @Override
    public void updateProfile() {
        // Calculate the next state (position and velocity)
        currentState = profile.calculate(0.02, currentState, goalState);
        double ffVolts = 0;
        // not sure if this works yet
        // double ffVolts = feedForward.calculate(currentState.velocity);;

        // Use the profiler's position as the target for the motor controller
        Logger.recordOutput("Elevator/ProfilerVelocity", currentState.velocity);
        Logger.recordOutput("Elevator/ProfilerPosition", currentState.position);

        // setting the motor controllers to the target positions and the controllers will do the PID
        // calculations
        primaryController.setReference(
                currentState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVolts);
    }
}
