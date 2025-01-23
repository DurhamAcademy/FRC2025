package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final SparkMax primaryMotor;
    private final SparkMax secondaryMotor;


    private final DigitalInput limitSwitch;

    private final boolean isZeroed = false;

    public ElevatorIOSparkMax() {
        primaryMotor = new SparkMax(ElevatorConstants.leftElevatorCanId, MotorType.kBrushless);
        secondaryMotor = new SparkMax(ElevatorConstants.rightElevatorCanId, MotorType.kBrushless);

        limitSwitch = new DigitalInput(ElevatorConstants.limitSwitchDIO);
    }
}
