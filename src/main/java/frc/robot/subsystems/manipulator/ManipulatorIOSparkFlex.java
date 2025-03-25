package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;

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
        followRollerL.configure(resetConfig, SparkBase.ResetMode.kResetSafeParameters, null);

        followerConfig.follow(primaryRollerR, true);
        followRollerL.configure(followerConfig, null, null);
    }

    public void updateInputs(ManipulatorIO.ManipulatorIOInputs inputs) {
        inputs.rollerLTemperature = followRollerL.getMotorTemperature();
        inputs.rollerLAppliedVolts =
                followRollerL.getAppliedOutput() * followRollerL.getBusVoltage();
        inputs.rollerLCurrentAmps = followRollerL.getOutputCurrent();
        inputs.rollerLVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(
                        followRollerL.getExternalEncoder().getVelocity());
        inputs.rollerLPosRad = followRollerL.getAbsoluteEncoder().getPosition();

        inputs.rollerRTemperature = primaryRollerR.getMotorTemperature();
        inputs.rollerRAppliedVolts =
                primaryRollerR.getAppliedOutput() * primaryRollerR.getBusVoltage();
        inputs.rollerRCurrentAmps = primaryRollerR.getOutputCurrent();
        inputs.rollerRVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(
                        primaryRollerR.getExternalEncoder().getVelocity());
        inputs.rollerRPosRad = followRollerL.getAbsoluteEncoder().getPosition();

        currentState.position = inputs.rollerRPosRad;
        currentState.velocity = inputs.rollerRVelocityRadPerSec;

        // inputs.beamObstructed = beam.get();
        inputs.sensorDistance =
                distanceSensor.getVoltage(); // todo: distance formula might not work
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

    public void setGoalState(double position, double velocity) {
        goalState.position = position;
        goalState.velocity = velocity;
    }

    // only to be used if we at some point want to have speed such as something ike barge or
    // something similar, thus it is commented out, but should be easy to set up since pid and
    // FF are already initialized
    public void updateProfile() {
        // Calculate the next state (position and velocity)
        currentState = profile.calculate(0.02, currentState, goalState);
        double ffVolts = feedForward.calculate(currentState.velocity);

        // Use the profiler's position as the target for the motor controller
        primaryController.setReference( currentState.position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVolts);
    }
}
