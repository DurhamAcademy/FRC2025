package frc.robot.subsystems.elevator;

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
    SparkMaxConfig resetConfig = new SparkMaxConfig();
    private final TrapezoidProfile.Constraints constraints;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;
    private final double zeroOffset = .0714 + .3;
    private final double horizontalFromZero =
            2.8107917308807373 - 1.3093030452728271; // 2.462753652191162 - 0.9380345331192017;

    private final ArmFeedforward feedForward;

    // target angle in radians. default angle is horizontal with the ground
    private double targetAngle = 0.0;

    public WristIOSparkMax() {
        // using a neo and an absolute encoder
        wristMotor = new SparkMax(WristConstants.wristCanId, SparkLowLevel.MotorType.kBrushless);
        // zeroed in REV to be horizontal with floor
        wristEncoder = wristMotor.getAbsoluteEncoder();
        wristController = wristMotor.getClosedLoopController();

        resetConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        resetConfig.closedLoop.pid(
                WristConstants.wristKp, WristConstants.wristKi, WristConstants.wristKd);

        // sets SparkMax to encode the position of wrist, accounting for gear ratios
        resetConfig
                .absoluteEncoder
                .positionConversionFactor(WristConstants.wristEncoderPositionFactor)
                .velocityConversionFactor(WristConstants.wristEncoderVelocityFactor)
                .zeroOffset(zeroOffset);

        wristMotor.configure(resetConfig, SparkBase.ResetMode.kResetSafeParameters, null);

        // sets velocity and acceleration constraints on the wrist
        constraints =
                new TrapezoidProfile.Constraints(
                        WristConstants.wristMaxVelocity, WristConstants.wristMaxAcceleration);
        profile = new TrapezoidProfile(constraints);
        // default goal and current state
        currentState =
                new TrapezoidProfile.State(wristEncoder.getPosition() - horizontalFromZero, 0);
        goalState = new TrapezoidProfile.State(wristEncoder.getPosition() - horizontalFromZero, 0);
        targetAngle = wristEncoder.getPosition() - horizontalFromZero;

        // feedforward to deal with gravity
        feedForward =
                new ArmFeedforward(
                        WristConstants.wristKs,
                        WristConstants.wristKg,
                        WristConstants.wristKv,
                        WristConstants.wristKa);
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
        inputs.angle = wristEncoder.getPosition() - horizontalFromZero;
        inputs.velocity = wristEncoder.getVelocity();
        Logger.recordOutput("Wrist/current", wristMotor.getOutputCurrent());
        Logger.recordOutput("Wrist/angle", wristMotor.getAppliedOutput());
        inputs.voltage = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();

        inputs.targetAngle = targetAngle;

        // determines if wrist is at target angle from angle and velocity
        // TODO adjust how precise angle needs to be
        inputs.isAtTargetAngle =
                Math.abs(
                                        Units.radiansToDegrees(inputs.angle)
                                                - Units.radiansToDegrees(inputs.targetAngle))
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
        goalState = new TrapezoidProfile.State(targetAngle, 0);
    }

    /** Updates wrist trapezoid profile with feed forward calculations */
    @Override
    public void updateStates() {
        currentState = profile.calculate(0.02, currentState, goalState);
        double ffVolts = feedForward.calculate(currentState.position, currentState.velocity);

        // Use the profiler's position as the target for the motor controller
        Logger.recordOutput("Wrist/ProfilerVelocity", currentState.velocity);
        Logger.recordOutput("Wrist/ProfilerPosition", currentState.position);

        wristController.setReference(
                currentState.position + horizontalFromZero,
                SparkBase.ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                ffVolts);
    }
}
