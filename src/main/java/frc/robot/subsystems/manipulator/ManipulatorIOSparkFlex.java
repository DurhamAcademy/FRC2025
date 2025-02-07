package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class ManipulatorIOSparkFlex {
    private final SparkFlex primaryRollerR =
            new SparkFlex(
                    ManipulatorConstants.MANIPULATOR_ROLLERL_CanId,
                    SparkLowLevel.MotorType.kBrushless);
    private final SparkFlex followRollerL =
            new SparkFlex(
                    ManipulatorConstants.MANIPULATOR_ROLLERR_CanId,
                    SparkLowLevel.MotorType.kBrushless);

    private final SparkClosedLoopController primaryController;
    private final SparkMaxConfig resetConfig = new SparkMaxConfig();
    private final TrapezoidProfile.Constraints constraints;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;
    private final ElevatorFeedforward feedForward;

    public ManipulatorIOSparkFlex() {
        primaryRollerR.setCANTimeout(250);
        followRollerL.setCANTimeout(250);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(primaryRollerR, true);
        followRollerL.configure(followerConfig, null, null);

        primaryController = primaryRollerR.getClosedLoopController();

        resetConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        resetConfig.smartCurrentLimit(40);
        resetConfig.voltageCompensation(12.0);
        resetConfig
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .pid(
                        ManipulatorConstants.manipulatorKp,
                        ManipulatorConstants.manipulatorKi,
                        ManipulatorConstants.manipulatorKd);

        configureMotors();

        constraints =
                new TrapezoidProfile.Constraints(
                        ManipulatorConstants.maxVelocity, ManipulatorConstants.maxAcceleration);
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
        profile = new TrapezoidProfile(constraints);

        feedForward =
                new ElevatorFeedforward(
                        ManipulatorConstants.manipulatorKs,
                        ManipulatorConstants.manipulatorKg,
                        ManipulatorConstants.manipulatorKv,
                        ManipulatorConstants.manipulatorKa);
    }

    private void configureMotors() {
        primaryRollerR.configure(resetConfig, SparkBase.ResetMode.kResetSafeParameters, null);
        followRollerL.configure(resetConfig, SparkBase.ResetMode.kResetSafeParameters, null);
    }

    public void updateInputs(ManipulatorIO.ManipulatorIOInputs inputs) {
        inputs.rollerLTemperature = new double[] {followRollerL.getMotorTemperature()};
        inputs.rollerLAppliedVolts =
                followRollerL.getAppliedOutput() * followRollerL.getBusVoltage();
        inputs.rollerLCurrentAmps = new double[] {followRollerL.getOutputCurrent()};
        inputs.rollerLVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(
                        followRollerL.getExternalEncoder().getVelocity());

        inputs.rollerRTemperature = new double[] {primaryRollerR.getMotorTemperature()};
        inputs.rollerRAppliedVolts =
                primaryRollerR.getAppliedOutput() * primaryRollerR.getBusVoltage();
        inputs.rollerRCurrentAmps = new double[] {primaryRollerR.getOutputCurrent()};
        inputs.rollerRVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(
                        primaryRollerR.getExternalEncoder().getVelocity());
    }

    /** Set intake wheel percent -1 to 1 */
    public void setRollerPercent(double percent) {
        primaryRollerR.set(percent);
    }

    /** Set intake wheel voltage */
    public void setRollerVoltage(double volts) {
        primaryRollerR.setVoltage(volts);
    }

    public void stopMotors() {
        primaryRollerR.set(0);
    }

    public void updateProfile() {
        // Calculate the next state (position and velocity)
        currentState = profile.calculate(0.02, currentState, goalState);
        double ffVolts = feedForward.calculate(currentState.velocity);

        // Use the profiler's position as the target for the motor controller
        primaryController.setReference(
                currentState.position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVolts);
    }
}
