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

    // Zero offset is the distance from the absolute encoder's default offset to where we want it to
    // set as zero
    // This value is intentionally a point where the wrist can never reach, avoiding issues with
    // chain reduction
    // and angle wraparound
    private final double zeroOffset = .0714 + .3;
    // Horizontal from zero is the distance from the newly set zero point to a point horizontal with
    // the ground
    // This is calculated by setting the wrist level, then getting the value of the absolute encoder
    // in advantage scope
    // Note: you can only accurately get this value from the WristIO on advantage scope if the
    // deployed code has this
    // temporarily set to 0
    private final double horizontalFromZero = 1.4490557670593262;

    private ArmFeedforward feedForward;

    // target angle in radians. default angle is horizontal with the ground
    private double targetAngle = 0.0;

    public WristIOSparkMax() {
        // using a neo and an absolute encoder
        wristMotor = new SparkMax(WristConstants.wristCanId, SparkLowLevel.MotorType.kBrushless);
        wristEncoder = wristMotor.getAbsoluteEncoder();
        // this wrist controller uses a relative encoder
        wristController = wristMotor.getClosedLoopController();
        relativeEncoder = wristMotor.getEncoder();

        resetConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

        resetConfig.closedLoop.pid(
                WristConstants.wristKp, WristConstants.wristKi, WristConstants.wristKd);

        // sets SparkMax to encode the position of wrist, accounting for gear ratios and the zero
        // offset
        resetConfig
                .absoluteEncoder
                .positionConversionFactor(WristConstants.wristAbsoluteEncoderReduction)
                .velocityConversionFactor(WristConstants.wristAbsoluteEncoderVelocityFactor)
                .zeroOffset(zeroOffset);

        resetConfig.smartCurrentLimit(30);

        // the relative encoder is used for setpoint calculation, so gear ratios must be set
        // these values are different from the absolute encoder as this encoder is from the motor
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

    /**
     * Gets the angle of the wrist with the zero set as horizontal with the ground
     *
     * @return double in radians
     */
    private double getWristOffsetAngle() {
        return wristEncoder.getPosition() - horizontalFromZero;
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
        inputs.voltage = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();

        inputs.targetAngle = targetAngle;

        // determines if wrist is on target
        // uses angle and velocity tolerance

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
        this.targetAngle =
                MathUtil.clamp(
                        targetAngle,
                        WristConstants.minWristPosition,
                        WristConstants.maxWristPosition);

        // resets current state to encoder values in case anything has changed since target angle
        // set
        currentState =
                new TrapezoidProfile.State(getWristOffsetAngle(), wristEncoder.getVelocity());
        goalState = new TrapezoidProfile.State(targetAngle, 0);
    }

    /** Updates wrist trapezoid profile with feed forward calculations */
    @Override
    public void updateStates() {
        double wristOffsetAngle = getWristOffsetAngle();
        // sets the relative encoder to horizontally zeroed value from absolute encoder
        // this is used in wristController.setReference() below
        relativeEncoder.setPosition(wristOffsetAngle);

        // moves the profile forward. calculates feedforward volts using horizontally-zeroed value
        currentState = profile.calculate(0.02, currentState, goalState);
        double ffVolts = feedForward.calculate(wristOffsetAngle, wristEncoder.getVelocity());

        // actually moves the motor to the setpoint
        wristController.setReference(
                currentState.position,
                SparkBase.ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                ffVolts);

        // Log profile and feedforward values
        Logger.recordOutput("Wrist/feedForwardVolts", ffVolts);
        Logger.recordOutput("Wrist/ProfilerVelocity", currentState.velocity);
        Logger.recordOutput("Wrist/ProfilerPosition", currentState.position);
    }
}
