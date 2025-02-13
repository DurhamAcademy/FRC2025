package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class WristIOSim implements WristIO {
    private double currentPositionRadians = 0.0; // Simulated position of the wrist (in radians)
    private double currentVelocityRadiansPerSec = 0.0; // Simulated velocity of the wrist
    private double appliedVoltage = 0.0; // Simulated applied voltage
    private Rotation2d targetAngle = Rotation2d.fromRadians(0); // Target angle

    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;
    private final ArmFeedforward feedForward;

    // TODO have base mech2d instead of separate one
    public LoggedMechanism2d wristMechanism2d = new LoggedMechanism2d(3, 3);
    public LoggedMechanismRoot2d root;
    public LoggedMechanismLigament2d wristLigament;

    public WristIOSim() {
        // Initialize feedforward controller and constraints
        constraints =
                new TrapezoidProfile.Constraints(
                        WristConstants.wristMaxVelocity, WristConstants.wristMaxAcceleration);
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);

        feedForward =
                new ArmFeedforward(
                        WristConstants.wristKs,
                        WristConstants.wristKg,
                        WristConstants.wristKv,
                        WristConstants.wristKa);

        root = wristMechanism2d.getRoot("wrist", 1.5, 1);
        wristLigament =
                root.append(
                        new LoggedMechanismLigament2d(
                                "wrist", 15, 90, 6, new Color8Bit(Color.kPurple)));
    }

    @Override
    public void setEncoder(double position) {
        currentPositionRadians = position; // Set simulated encoder position for testing
    }

    @Override
    public void setPower(double power) {
        // Simulate power control by scaling to a voltage range
        appliedVoltage = power * 12.0;
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVoltage = voltage; // Directly set simulated applied voltage
    }

    @Override
    public void stopMotors() {
        appliedVoltage = 0.0; // Simulate motor stop
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        // Update simulated inputs for AdvantageScope
        inputs.angle = Rotation2d.fromRadians(currentPositionRadians); // Simulated angle
        inputs.targetAngle = targetAngle;
        inputs.velocity = currentVelocityRadiansPerSec; // Simulated velocity

        inputs.isAtTargetAngle =
                Math.abs(inputs.angle.getRadians() - inputs.targetAngle.getRadians()) < 0.05
                        && Math.abs(inputs.velocity) < 0.1;

        inputs.voltage = appliedVoltage;

        // Log the inputs for visualization in AdvantageScope
        Logger.recordOutput("WristSim/Angle", currentPositionRadians);
        Logger.recordOutput("WristSim/Velocity", currentVelocityRadiansPerSec);
        Logger.recordOutput("WristSim/TargetAngle", targetAngle.getRadians());
        Logger.recordOutput("WristSim/Mechanism2d", wristMechanism2d);
        wristLigament.setAngle(Units.radiansToDegrees(currentPositionRadians));
    }

    @Override
    public void setTargetAngle(double targetAngleRadians) {
        // Set a new target angle for the wrist
        this.targetAngle = Rotation2d.fromRadians(targetAngleRadians);
        goalState = new TrapezoidProfile.State(targetAngleRadians, 0);
    }

    @Override
    public void updateProfile() {
        // Simulate the motion profile for position and velocity updates
        currentState =
                new TrapezoidProfile(constraints)
                        .calculate(0.02, currentState, goalState); // Simulate a 20 ms control loop

        // Use feedforward to predict the voltage required at the current state
        double ffVoltage = feedForward.calculate(currentState.position, currentState.velocity);

        // Update simulated state variables
        currentPositionRadians = currentState.position;
        currentVelocityRadiansPerSec = currentState.velocity;
        appliedVoltage = ffVoltage;

        // Log profiler values
        Logger.recordOutput("WristSim/ProfilerPosition", currentState.position);
        Logger.recordOutput("WristSim/ProfilerVelocity", currentState.velocity);
        Logger.recordOutput("WristSim/FeedforwardVoltage", ffVoltage);
    }
}
