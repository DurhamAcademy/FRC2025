package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.elevator.Elevator.ElevatorLevel;

public class ElevatorIOSparkMax implements ElevatorIO {
    // some credit to https://chiefdelphi.com/t/elevator-subsystem-example-code/482648
    private final SparkMax primaryMotor;
    private final SparkMax followerMotor;
    private final RelativeEncoder primaryEncoder;
    private final DigitalInput limitSwitch;
    private final SparkMaxConfig resetConfig = new SparkMaxConfig();
    private double targetHeightInches = 0.0;

    private final boolean isZeroed = false;

    public ElevatorIOSparkMax() {
        // Primary motor = left motor

        primaryMotor = new SparkMax(ElevatorConstants.leftElevatorCanId, MotorType.kBrushless);
        followerMotor = new SparkMax(ElevatorConstants.rightElevatorCanId, MotorType.kBrushless);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(primaryMotor);

        followerMotor.configure(followerConfig, null, null); // TODO: WHAT IS THIS NULL NULL THING

        primaryEncoder = primaryMotor.getEncoder();
        limitSwitch = new DigitalInput(ElevatorConstants.limitSwitchPort);

        resetConfig.idleMode(IdleMode.kBrake);
        resetConfig.smartCurrentLimit(40);
        resetConfig.voltageCompensation(12.0);
        resetConfig
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .pidf(ElevatorConstants.elevatorKp, ElevatorConstants.elevatorKi, ElevatorConstants.elevatorKd, ElevatorConstants.elevatorFF)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(-Math.PI, Math.PI);

        configureMotors();
    }

    private void configureMotors() {
        // Primary motor configuration
        primaryMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);

        // Follower motor configuration
        primaryMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
    }

    @Override
    public void resetEncoder() {
        primaryEncoder.setPosition(0);
    }

    @Override
    public void setVoltage(double voltage) {
        primaryMotor.setVoltage(voltage);
    }

    @Override
    public void stopMotors() {
        primaryMotor.set(0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.isLimitSwitchPressed = limitSwitch.get();
        inputs.heightInches = primaryEncoder.getPosition() / ElevatorConstants.countsPerInch;
        inputs.targetHeightInches = targetHeightInches;
        inputs.velocityInches = primaryEncoder.getVelocity() / ElevatorConstants.countsPerInch;
        inputs.isAtTargetLevel = Math.abs(inputs.heightInches - targetHeightInches) < 0.5 && Math.abs(inputs.velocityInches) < 0.1;
    }

    @Override
    public void setTargetHeightInches(double heightInches) {
        targetHeightInches =
                MathUtil.clamp(
                        heightInches,
                        ElevatorLevel.ZERO.heightInches,
                        ElevatorLevel.L4.heightInches
        );
    }


}
