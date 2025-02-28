package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class WristIOSparkMax implements WristIO {
    private final SparkMax wristMotor;
    private final SparkClosedLoopController wristController;
    private final SparkAbsoluteEncoder wristEncoder;
    private final RelativeEncoder relativeEncoder;
    SparkMaxConfig resetConfig = new SparkMaxConfig();
    private final TrapezoidProfile.Constraints constraints;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;
    private final double zeroOffset = .0714 + .3;
    private final double horizontalFromZero = 1.4490557670593262;
    // 1.3490557670593262

    private ArmFeedforward feedForward;

    // target angle in radians. default angle is horizontal with the ground
    private double targetAngle = 0.0;

    public WristIOSparkMax() {
        // using a neo and an absolute encoder
        wristMotor = new SparkMax(WristConstants.wristCanId, SparkLowLevel.MotorType.kBrushless);
        // zeroed in REV to be horizontal with floor
        wristEncoder = wristMotor.getAbsoluteEncoder();
        wristController = wristMotor.getClosedLoopController();
        relativeEncoder = wristMotor.getEncoder();

        resetConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        resetConfig.closedLoop.pid(
                WristConstants.wristKp, WristConstants.wristKi, WristConstants.wristKd);

        // sets SparkMax to encode the position of wrist, accounting for gear ratios
        resetConfig
                .absoluteEncoder
                .positionConversionFactor(WristConstants.wristAbsoluteEncoderReduction)
                .velocityConversionFactor(WristConstants.wristAbsoluteEncoderVelocityFactor)
                .zeroOffset(zeroOffset);

        resetConfig
                .encoder
                .positionConversionFactor(WristConstants.wristRelativeEncoderReduction)
                .velocityConversionFactor(WristConstants.wristRelativeEncoderVelocityFactor);

        wristMotor.configure(resetConfig, SparkBase.ResetMode.kResetSafeParameters, null);

        // sets velocity and acceleration constraints on the wrist
        constraints =
                new TrapezoidProfile.Constraints(
                        WristConstants.wristMaxVelocity, WristConstants.wristMaxAcceleration);
        profile = new TrapezoidProfile(constraints);

        // default goal and current state
        currentState = new TrapezoidProfile.State(getWristOffsetAngle(), 0);
        targetAngle = getWristOffsetAngle();
        goalState = new TrapezoidProfile.State(targetAngle, 0);

        // feedforward to deal with gravity
        feedForward =
                new ArmFeedforward(
                        WristConstants.wristKs,
                        WristConstants.wristKg,
                        WristConstants.wristKv,
                        WristConstants.wristKa);
    }

    public double getWristOffsetAngle() {
        return wristEncoder.getPosition() - horizontalFromZero;
    }

    @Override
    public void recreateFeedforward() {
        feedForward =
                new ArmFeedforward(
                        WristConstants.wristKs,
                        WristConstants.wristKg,
                        WristConstants.wristKv,
                        WristConstants.wristKa);
    }

    @Override
    public void resetConfig() {
        resetConfig.closedLoop.pid(
                WristConstants.wristKp, WristConstants.wristKi, WristConstants.wristKd);
        wristMotor.configure(resetConfig, SparkBase.ResetMode.kResetSafeParameters, null);
    }

    /**
     * Resets the position of the wrist's relative encoder to a specified value.
     *
     * @param position The position value to set the encoder to (radians).
     */
    @Override
    public void setEncoder(double position) {}

    /**
     * Sets the speed of the wrist motor.
     *
     * @param speed The desired speed level for the motor: -1.0 => full reverse, 1.0 => full
     *     forward, 0.0 => no speed.
     */
    @Override
    public void setSpeed(double speed) {
        wristMotor.set(speed);
    }

    /**
     * Sets the voltage for the wrist motor.
     *
     * @param voltage The desired voltage to apply to the wrist motor (volts).
     */
    @Override
    public void setVoltage(double voltage) {
        wristMotor.setVoltage(voltage);
    }

    /** Stops wrist motor. */
    @Override
    public void stopMotors() {
        wristMotor.set(0);
    }

    /**
     * Updates angle (rad), target angle (rad), velocity (rad/s), voltage (volts), and 'at target
     * angle' boolean
     *
     * @param inputs WristIOInputs
     */
    @Override
    public void updateInputs(WristIO.WristIOInputs inputs) {
        // sets inputs from raw values
        inputs.angle = getWristOffsetAngle();
        inputs.velocity = wristEncoder.getVelocity();
        Logger.recordOutput("Wrist/current", wristMotor.getOutputCurrent());
        Logger.recordOutput("Wrist/angle", wristEncoder.getPosition());
        inputs.voltage = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();

        inputs.targetAngle = targetAngle;

        // determines if wrist is at target angle from angle and velocity
        // TODO adjust how precise angle needs to be
        inputs.isAtTargetAngle =
                Math.abs(
                                        Units.radiansToDegrees(inputs.angle)
                                                - Units.radiansToDegrees(
                                                        inputs.targetAngle - horizontalFromZero))
                                < WristConstants.wristAngularTolerance
                        && Math.abs(inputs.velocity) < WristConstants.wristVelocityTolerance;
    }

    /**
     * Sets the target angle of the wrist
     *
     * @param targetAngle (radians)
     */
    @Override
    public void setTargetAngle(double targetAngle) {
        Logger.recordOutput("Wrist/targetAngle", targetAngle);
        this.targetAngle =
                MathUtil.clamp(
                        targetAngle,
                        WristConstants.minWristPosition,
                        WristConstants.maxWristPosition);
        currentState =
                new TrapezoidProfile.State(getWristOffsetAngle(), wristEncoder.getVelocity());
        goalState = new TrapezoidProfile.State(targetAngle, 0);
    }

    /** Updates wrist trapezoid profile with feed forward calculations */
    @Override
    public void updateStates() {
        relativeEncoder.setPosition(getWristOffsetAngle());
        currentState = profile.calculate(0.02, currentState, goalState);
        double ffVolts = feedForward.calculate(getWristOffsetAngle(), wristEncoder.getVelocity());
        Logger.recordOutput("Wrist/kd", WristConstants.wristKd);

        // Use the profiler's position as the target for the motor controller
        Logger.recordOutput("Wrist/feedForwardVolts", ffVolts);
        Logger.recordOutput("Wrist/ProfilerVelocity", currentState.velocity);
        Logger.recordOutput("Wrist/ProfilerPosition", currentState.position);

        wristController.setReference(
                currentState.position,
                SparkBase.ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                ffVolts);
    }
}
