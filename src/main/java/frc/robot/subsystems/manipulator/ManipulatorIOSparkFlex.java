package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import org.littletonrobotics.junction.Logger;

public class ManipulatorIOSparkFlex implements ManipulatorIO {
    private final SparkFlex primaryRollerR =
            new SparkFlex(
                    ManipulatorConstants.leftManipulatorRollerCanId,
                    SparkLowLevel.MotorType.kBrushless);
    private final SparkFlex followRollerL =
            new SparkFlex(
                    ManipulatorConstants.rightManipulatorRollerCanId,
                    SparkLowLevel.MotorType.kBrushless);

    private final SparkFlexConfig resetConfig = new SparkFlexConfig();
    private final SparkFlexConfig followerConfig;

    private final SparkClosedLoopController primaryController;
    private final TrapezoidProfile.Constraints constraints;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;
    private final SimpleMotorFeedforward feedForward;

    // private final DigitalInput beam;
    private final AnalogInput distanceSensor;

    public ManipulatorIOSparkFlex() {
        primaryRollerR.setCANTimeout(250);
        followRollerL.setCANTimeout(250);

        followerConfig = new SparkFlexConfig();

        primaryController = primaryRollerR.getClosedLoopController();

        // beam = new DigitalInput(ManipulatorConstants.MANIPULATOR_BEAM_ID);
        // sets brake mode
        resetConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        // this is the amp limit, this is high at 80 because of net and how short eject
        // is, but we should be careful about running this too long
        resetConfig.smartCurrentLimit(80);
        resetConfig
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .pid(
                        ManipulatorConstants.manipulatorKp,
                        ManipulatorConstants.manipulatorKi,
                        ManipulatorConstants.manipulatorKd);
        resetConfig
                .encoder
                .positionConversionFactor(2 * Math.PI)
                .velocityConversionFactor(2 * Math.PI / 60);
        followerConfig
                .encoder
                .positionConversionFactor(2 * Math.PI)
                .velocityConversionFactor(2 * Math.PI / 60);
        followerConfig.follow(primaryRollerR, true);

        configureMotors();

        constraints =
                new TrapezoidProfile.Constraints(
                        ManipulatorConstants.maxVelocity, ManipulatorConstants.maxAcceleration);
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
        profile = new TrapezoidProfile(constraints);

        feedForward =
                new SimpleMotorFeedforward(
                        ManipulatorConstants.manipulatorKs,
                        ManipulatorConstants.manipulatorKv,
                        ManipulatorConstants.manipulatorKa);

        distanceSensor = new AnalogInput(ManipulatorConstants.manipulatorDistanceSensorPort);
    }

    private void configureMotors() {
        primaryRollerR.configure(resetConfig, SparkBase.ResetMode.kResetSafeParameters, null);
        followRollerL.configure(followerConfig, null, null);
    }

    public void updateInputs(ManipulatorIO.ManipulatorIOInputs inputs) {
        inputs.rollerLTemperature = followRollerL.getMotorTemperature();
        inputs.rollerLAppliedVolts =
                followRollerL.getAppliedOutput() * followRollerL.getBusVoltage();
        inputs.rollerLCurrentAmps = followRollerL.getOutputCurrent();
        inputs.rollerLVelocityRadPerSec = followRollerL.getEncoder().getVelocity();
        inputs.rollerLPosRad = followRollerL.getEncoder().getPosition();

        inputs.rollerRTemperature = primaryRollerR.getMotorTemperature();
        inputs.rollerRAppliedVolts =
                primaryRollerR.getAppliedOutput() * primaryRollerR.getBusVoltage();
        inputs.rollerRCurrentAmps = primaryRollerR.getOutputCurrent();
        inputs.rollerRVelocityRadPerSec = primaryRollerR.getEncoder().getVelocity();
        inputs.rollerRPosRad = followRollerL.getEncoder().getPosition();
        inputs.isAtSetpoint =
                Math.abs(inputs.rollerRPosRad - goalState.position) < 0.1
                        && Math.abs(inputs.rollerRVelocityRadPerSec) < 0.1;

        // inputs.beamObstructed = beam.get();
        inputs.sensorDistance = distanceSensor.getVoltage();
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

    public void setGoalStateToCurrentPosition() {
        goalState.position = primaryRollerR.getEncoder().getPosition();
        goalState.velocity = 0;

        currentState.position = primaryRollerR.getEncoder().getPosition();
        currentState.velocity = primaryRollerR.getEncoder().getVelocity();
    }

    public void setGoalState(double position, double velocity) {
        goalState.position = position;
        goalState.velocity = velocity;
    }

    public void updateProfile() {
        primaryController.setReference(goalState.position, SparkBase.ControlType.kPosition);
    }
    //        // Calculate the next state (position and velocity)
    //        currentState = profile.calculate(0.02, currentState, goalState);
    //        double ffVolts = feedForward.calculate(currentState.velocity);
    //        ffVolts = 0.0;
    //        Logger.recordOutput("Manipulator/currentStatePosition", currentState.position);
    //        Logger.recordOutput("Manipulator/goalStatePosition", goalState.position);
    //
    //        // Use the profiler's position as the target for the motor controller
    //        primaryController.setReference(
    //                currentState.position,
    //                SparkBase.ControlType.kPosition,
    //                ClosedLoopSlot.kSlot0,
    //                ffVolts);
    //    }
}
