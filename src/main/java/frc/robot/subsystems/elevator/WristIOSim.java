package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.Logger;

public class WristIOSim implements WristIO {
    private double currentPositionRadians = 0.0; // Simulated position of the wrist (in radians)
    private double currentVelocityRadiansPerSec = 0.0; // Simulated velocity of the wrist
    private double appliedVoltage = 0.0; // Simulated applied voltage
    private double targetAngle = 0.0; // Target angle

    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;
    private final ArmFeedforward feedForward;

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
        inputs.angle = currentPositionRadians; // Simulated angle
        inputs.targetAngle = targetAngle;
        inputs.velocity = currentVelocityRadiansPerSec; // Simulated velocity

        inputs.isAtTargetAngle =
                Math.abs(inputs.angle - inputs.targetAngle) < 0.05
                        && Math.abs(inputs.velocity) < 0.1;

        inputs.voltage = appliedVoltage;

        // Log the inputs for visualization in AdvantageScope
        Logger.recordOutput("WristSim/Angle", currentPositionRadians);
        Logger.recordOutput("WristSim/Velocity", currentVelocityRadiansPerSec);
        Logger.recordOutput("WristSim/TargetAngle", targetAngle);
    }

    @Override
    public void setTargetAngle(double targetAngleRadians) {
        // Set a new target angle for the wrist
        this.targetAngle = targetAngleRadians;
        goalState = new TrapezoidProfile.State(targetAngleRadians, 0);
    }

    @Override
    public void updateStates() {
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
