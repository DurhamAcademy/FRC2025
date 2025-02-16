package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {
    private double currentHeightInches = 0.0; // Simulated height of the elevator
    private double currentVelocityInchesPerSec = 0.0; // Simulated velocity of the elevator
    private double appliedVoltage = 0.0; // Simulated applied voltage
    private double targetHeightInches = 0.0; // Target height of the elevator

    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;
    private final ElevatorFeedforward feedForward;

    /** Simulated limit switch */
    private boolean limitSwitchPressed = false;

    public ElevatorIOSim() {
        // Initialize constraints, trapezoid profile states, and feedforward controller
        constraints =
                new TrapezoidProfile.Constraints(
                        ElevatorConstants.elevatorMaxVelocity, // max velocity in inches/sec
                        ElevatorConstants
                                .elevatorMaxAcceleration); // max acceleration in inches/sec^2

        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);

        feedForward =
                new ElevatorFeedforward(
                        ElevatorConstants.elevatorKs,
                        ElevatorConstants.elevatorKg,
                        ElevatorConstants.elevatorKv,
                        ElevatorConstants.elevatorKa);
    }

    @Override
    public void setEncoder(double position) {
        // Force the simulated encoder position for testing
        currentHeightInches = position;
    }

    @Override
    public void setPower(double power) {
        // Set simulated power control by scaling it to a voltage range
        appliedVoltage = power * 12.0;
    }

    @Override
    public void setVoltage(double voltage) {
        // Directly set the simulated applied voltage
        appliedVoltage = voltage;
    }

    @Override
    public void stopMotors() {
        // Simulate motor stop by setting applied voltage to zero
        appliedVoltage = 0.0;
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Simulate sensor data and state variables
        inputs.isLimitSwitchPressed = limitSwitchPressed;
        inputs.leftHeightInches = currentHeightInches;
        inputs.rightHeightInches =
                currentHeightInches; // Assume symmetrical behavior for simplicity
        inputs.targetHeightInches = targetHeightInches;
        inputs.velocityInches = currentVelocityInchesPerSec;

        // Check if the elevator has reached its target position
        inputs.isAtTargetLevel =
                Math.abs(inputs.leftHeightInches - inputs.targetHeightInches) < 0.5
                        && Math.abs(inputs.velocityInches) < 0.1;

        inputs.leftVoltage = appliedVoltage;
        inputs.rightVoltage = appliedVoltage;

        // Log values for visualization in AdvantageScope or SmartDashboard
        Logger.recordOutput("ElevatorSim/Height", currentHeightInches);
        Logger.recordOutput("ElevatorSim/Velocity", currentVelocityInchesPerSec);
        Logger.recordOutput("ElevatorSim/TargetHeight", targetHeightInches);
        Logger.recordOutput("ElevatorSim/AppliedVoltage", appliedVoltage);
    }

    @Override
    public void setTargetHeightInches(double heightInches) {
        // Clamp the target height to valid limits and update the trapezoid profile goal state
        targetHeightInches =
                MathUtil.clamp(
                        heightInches, ElevatorConstants.minHeight, ElevatorConstants.maxHeight);
        goalState = new TrapezoidProfile.State(targetHeightInches, 0);
    }

    @Override
    public void updateProfile() {
        // Calculate motion profile for position and velocity over a 20ms control loop
        currentState = new TrapezoidProfile(constraints).calculate(0.02, currentState, goalState);

        // Calculate feedforward voltage based on current velocity and acceleration
        double ffVoltage = feedForward.calculate(currentState.velocity);

        // Update simulated state variables
        currentHeightInches = currentState.position;
        currentVelocityInchesPerSec = currentState.velocity;
        appliedVoltage = ffVoltage;

        // Simulate limit switch behavior (elevator "bottoming out")
        if (currentHeightInches <= ElevatorConstants.minHeight) {
            currentHeightInches = ElevatorConstants.minHeight;
            limitSwitchPressed = true;
            currentVelocityInchesPerSec = 0.0; // Stop motion
        } else {
            limitSwitchPressed = false;
        }

        // Log profiler values for visualization
        Logger.recordOutput("ElevatorSim/ProfilerPosition", currentState.position);
        Logger.recordOutput("ElevatorSim/ProfilerVelocity", currentState.velocity);
        Logger.recordOutput("ElevatorSim/FeedforwardVoltage", ffVoltage);
    }
}
