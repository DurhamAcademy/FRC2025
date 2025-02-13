package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.littletonrobotics.junction.Logger;

public class WristIOSparkMax implements WristIO {
    private final SparkMax wristMotor;
    private final SparkClosedLoopController wristController;
    private final RelativeEncoder wristRelativeEncoder;
    private final DutyCycleEncoder wristAbsoluteEncoder;

    private final TrapezoidProfile.Constraints constraints;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;
    private final ArmFeedforward feedForward;

    // target angle in radians. default angle is horizontal with the ground
    private double targetAngle = 0.0;

    public WristIOSparkMax() {
        // using a neo and an absolute encoder
        wristMotor = new SparkMax(WristConstants.wristCanId, SparkLowLevel.MotorType.kBrushless);
        wristRelativeEncoder = wristMotor.getEncoder();
        wristAbsoluteEncoder = new DutyCycleEncoder(0);
        wristController = wristMotor.getClosedLoopController();

        SparkMaxConfig resetConfig = new SparkMaxConfig();
        resetConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        resetConfig.smartCurrentLimit(30);
        resetConfig.voltageCompensation(12.0);
        resetConfig
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .pid(WristConstants.wristKp, WristConstants.wristKi, WristConstants.wristKd);
        // sets SparkMax to encode the position of wrist, accounting for gear ratios
        resetConfig
                .encoder
                .positionConversionFactor(WristConstants.wristEncoderPositionFactor)
                .velocityConversionFactor(WristConstants.wristEncoderVelocityFactor);

        wristMotor.configure(resetConfig, SparkBase.ResetMode.kResetSafeParameters, null);

        // sets velocity and acceleration constraints on the wrist
        constraints =
                new TrapezoidProfile.Constraints(
                        WristConstants.wristMaxVelocity, WristConstants.wristMaxAcceleration);
        profile = new TrapezoidProfile(constraints);

        // default goal and current state
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);

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
    public void setEncoder(double position) {
        wristRelativeEncoder.setPosition(Units.radiansToRotations(position));
    }

    /**
     * Sets the power of the wrist motor.
     *
     * @param power The desired power level for the motor: -1.0 => full reverse,
     *              1.0 => full forward, 0.0 => no power.
     */
    @Override
    public void setPower(double power) {
        wristMotor.set(power);
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

    /**
     * Stops wrist motor.
     */
    @Override
    public void stopMotors() {
        wristMotor.set(0);
    }

    /**
     * Updates angle (rad), target angle (rad), velocity (rad/s), voltage (volts), and 'at target angle' boolean
     * @param inputs WristIOInputs
     */
    @Override
    public void updateInputs(WristIO.WristIOInputs inputs) {
        // sets inputs from raw values
        inputs.angle = Units.rotationsToRadians(wristAbsoluteEncoder.get());
        inputs.velocity = wristRelativeEncoder.getVelocity();
        inputs.voltage = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();

        inputs.targetAngle = targetAngle;

        // determines if wrist is at target angle from angle and velocity
        // TODO adjust how precise angle needs to be
        inputs.isAtTargetAngle =
                Math.abs(Units.radiansToDegrees(inputs.angle) - Units.radiansToDegrees(inputs.targetAngle)) < WristConstants.wristAngularTolerance
                        && Math.abs(inputs.velocity) < WristConstants.wristVelocityTolerance;
    }

    /**
     * Sets the target angle of the wrist
     * @param targetAngle (radians)
     */
    @Override
    public void setTargetAngle(double targetAngle) {

        this.targetAngle = MathUtil.clamp(targetAngle, WristConstants.minWristPosition, WristConstants.maxWristPosition);
        goalState = new TrapezoidProfile.State(targetAngle, 0);
    }

    /**
     * Updates wrist trapezoid profile with feed forward calculations
     */
    @Override
    public void updateProfile() {
        currentState = profile.calculate(0.02, currentState, goalState);
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
