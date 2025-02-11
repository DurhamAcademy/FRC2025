package frc.robot.subsystems.elevator;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSim elevatorSim;
    private final SparkMaxSim motorSim;
    private final SparkRelativeEncoderSim encoderSim;
    private final SparkClosedLoopController controllerSim;
    private final DIOSim limitSwitchSim;

    private final TrapezoidProfile.Constraints constraints;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;
    private final ElevatorFeedforward feedForward;

    private double targetHeightInches = 0.0;



    public ElevatorIOSim() {
        SparkMax sparkMax = new SparkMax(ElevatorConstants.leftElevatorCanId, MotorType.kBrushless);

        controllerSim = sparkMax.getClosedLoopController();

        limitSwitchSim = new DIOSim(ElevatorConstants.limitSwitchPort);

        // Create the SparkMax configuration
        SparkMaxConfig motorConfig = new SparkMaxConfig();

        // Set basic motor configuration
        motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);

        // Configure the encoder
        motorConfig.encoder
                .positionConversionFactor(ElevatorConstants.elevatorEncoderPositionFactor)
                .velocityConversionFactor(ElevatorConstants.elevatorEncoderVelocityFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        // Configure closed loop control
        motorConfig.closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .pid(
                        ElevatorConstants.elevatorKp,
                        ElevatorConstants.elevatorKi,
                        ElevatorConstants.elevatorKd
                );

        sparkMax.configure(motorConfig, ResetMode.kResetSafeParameters, null);

        motorSim = new SparkMaxSim(new SparkMax(ElevatorConstants.leftElevatorCanId, MotorType.kBrushless), DCMotor.getNEO(2));
        encoderSim = motorSim.getRelativeEncoderSim();

        constraints =
                new TrapezoidProfile.Constraints(
                        ElevatorConstants.elevatorMaxVelocity, // in/s
                        ElevatorConstants.elevatorMaxAcceleration); // in/s
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
        profile = new TrapezoidProfile(constraints);

        feedForward =
                new ElevatorFeedforward(
                        ElevatorConstants.elevatorKs,
                        ElevatorConstants.elevatorKg,
                        ElevatorConstants.elevatorKv,
                        ElevatorConstants.elevatorKa);

        // All sim stuff should be done in meters because it's designed to work with meters
        elevatorSim =
                new ElevatorSim(
                        LinearSystemId.createElevatorSystem(
                                DCMotor.getNEO(2),
                                ElevatorConstants.elevatorMotorReduction,
                                ElevatorConstants.carriageWeightKg,
                                Units.inchesToMeters(ElevatorConstants.elevatorEffectiveDrumRadius)),
                        DCMotor.getNEO(2),
                        Units.inchesToMeters(ElevatorConstants.ZERO), // min height
                        Units.inchesToMeters(ElevatorConstants.L4), // max height
                        true, // simulate gravity
                        0,
                        0.003, // Position stddev - NEO encoder precision
                        0.03);   // Velocity stddev
    }

    // mechanism2d stuff
    // discard later
    private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
    private final MechanismRoot2d m_mech2d_root = m_mech2d.getRoot("Elevator Root", 10, 0);
    private final MechanismLigament2d m_elevatorMech2d =
            m_mech2d_root.append(
                    new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90));

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
        motorSim.set(voltage);
    }

    @Override
    public void stopMotors() {
        motorSim.setMotorCurrent(0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.isLimitSwitchPressed = limitSwitchSim.getValue();
        inputs.leftHeightInches = encoderSim.getPosition();
        inputs.rightHeightInches = encoderSim.getPosition();
        inputs.targetHeightInches = targetHeightInches;
        inputs.velocityInches = encoderSim.getVelocity();
        inputs.isAtTargetLevel =
                Math.abs(inputs.leftHeightInches - targetHeightInches) < 0.5
                        && Math.abs(inputs.velocityInches) < 0.1;
        inputs.leftVoltage = motorSim.getAppliedOutput() * motorSim.getBusVoltage();
        inputs.rightVoltage = motorSim.getAppliedOutput() * motorSim.getBusVoltage();
    }

    @Override
    public void setTargetHeightInches(double heightInches) {
        targetHeightInches =
                MathUtil.clamp(
                        heightInches, ElevatorConstants.minHeight, ElevatorConstants.maxHeight);

        goalState = new TrapezoidProfile.State(targetHeightInches, 0);
    }

    @Override
    public void periodic() {
        // Calculate the next state (position and velocity)
        currentState = profile.calculate(0.02, currentState, goalState);
        double ffVolts = feedForward.calculate(currentState.velocity);

        // Use the profiler's position as the target for the motor controller
        Logger.recordOutput("Elevator/ProfilerVelocity", currentState.velocity);
        Logger.recordOutput("Elevator/ProfilerPosition", currentState.position);

        .setReference(
                currentState.position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVolts);
        elevatorSim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());
        elevatorSim.update(0.02);
        m_encoderSim.setDistance(elevatorSim.getPositionMeters());
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    }
}
