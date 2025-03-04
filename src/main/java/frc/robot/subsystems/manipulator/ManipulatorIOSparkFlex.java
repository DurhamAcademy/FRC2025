package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;

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

    // private final DigitalInput beam;
    private final AnalogInput distanceSensor;

    public ManipulatorIOSparkFlex() {
        primaryRollerR.setCANTimeout(250);
        followRollerL.setCANTimeout(250);

        followerConfig = new SparkFlexConfig();

        // beam = new DigitalInput(ManipulatorConstants.MANIPULATOR_BEAM_ID);

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

        inputs.rollerRTemperature = primaryRollerR.getMotorTemperature();
        inputs.rollerRAppliedVolts =
                primaryRollerR.getAppliedOutput() * primaryRollerR.getBusVoltage();
        inputs.rollerRCurrentAmps = primaryRollerR.getOutputCurrent();
        inputs.rollerRVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(
                        primaryRollerR.getExternalEncoder().getVelocity());
        // inputs.beamObstructed = beam.get();
        inputs.sensorDistance =
                27.86
                        / (distanceSensor.getVoltage()
                                - 0.42); // todo: distance formula might not work
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

    // only to be used if we at some point want to have speed such as something ike barge or
    // something similar, thus it is commented out, but should be easy to set up since pid and
    // FF are already initialized
    /**
     * public void updateProfile() { // Calculate the next state (position and velocity)
     * currentState = profile.calculate(0.02, currentState, goalState); double ffVolts =
     * feedForward.calculate(currentState.velocity);
     *
     * <p>// Use the profiler's position as the target for the motor controller
     * primaryController.setReference( currentState.position, SparkBase.ControlType.kPosition,
     * ClosedLoopSlot.kSlot0, ffVolts); <<<<<<< HEAD }
     */
}
