package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.drive.DriveConstants.levelFourSpeedLimit;
import static frc.robot.subsystems.drive.DriveConstants.maxSpeedLimitMetersPerSec;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    private final WristIO wristIO;
    private final Drive drive;

    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

    private static final LoggedMechanism2d loggedMechanism = new LoggedMechanism2d(3, 3);
    public LoggedMechanismRoot2d loggedMechanismRoot;
    public LoggedMechanismLigament2d wristLigament;
    public LoggedMechanismLigament2d elevatorLigament;

    //    SysIdRoutine elevatorSysIDRoutine;
    //    SysIdRoutine wristSysIDRoutine;

    private boolean wristRestricted = false;
    private double savedWristTargetAngle = 0.0;

    public enum ElevatorLevel {
        ZERO(ElevatorConstants.ZERO, WristConstants.ALGAE_IDLE),
        INTAKE(ElevatorConstants.ZERO, WristConstants.INTAKE),
        L1(ElevatorConstants.L1, WristConstants.L1),
        L2(ElevatorConstants.L2, WristConstants.L2),
        L3(ElevatorConstants.L3, WristConstants.L3),
        L4(ElevatorConstants.L4, WristConstants.L4),
        NET(ElevatorConstants.L4, WristConstants.NET),
        PROCESSOR(ElevatorConstants.ZERO, WristConstants.PROCESSOR),
        LOWER_ALGAE_REMOVAL(
                ElevatorConstants.LOWER_ALGAE_REMOVAL, WristConstants.LOWER_ALGAE_REMOVAL),
        UPPER_ALGAE_REMOVAL(
                ElevatorConstants.UPPER_ALGAE_REMOVAL, WristConstants.UPPER_ALGAE_REMOVAL),
        LOLLIPOP_REMOVAL(ElevatorConstants.LOLLIPOP_REMOVAL, WristConstants.LOLLIPOP_REMOVAL),
        ;

        public final double heightInches;
        public final double angleRadians;

        ElevatorLevel(double heightInches, double angleRadians) {
            this.heightInches = heightInches;
            this.angleRadians = angleRadians;
        }
    }

    public Elevator(ElevatorIO elevatorIO, WristIO wristIO, Drive drive) {
        this.elevatorIO = elevatorIO;
        this.wristIO = wristIO;
        this.drive = drive;

        //        elevatorSysIDRoutine =
        //                new SysIdRoutine(
        //                        new SysIdRoutine.Config(
        //                                null,
        //                                null,
        //                                null,
        //                                (state) -> Logger.recordOutput("SysIdTestState",
        // state.toString())),
        //                        new SysIdRoutine.Mechanism(
        //                                (voltage) -> elevatorIO.setVoltage(voltage.in(Volts)),
        // null, this));
        //
        //        wristSysIDRoutine =
        //                new SysIdRoutine(
        //                        new SysIdRoutine.Config(
        //                                Volts.per(Second).of(.5),
        //                                Volts.of(2),
        //                                Seconds.of(5),
        //                                (state) -> Logger.recordOutput("SysIdTestState",
        // state.toString())),
        //                        new SysIdRoutine.Mechanism(
        //                                (voltage) -> wristIO.setVoltage(voltage.in(Volts)), null,
        // this));

        loggedMechanismRoot =
                loggedMechanism.getRoot(
                        "elevator",
                        1.5,
                        Units.inchesToMeters(ElevatorConstants.elevatorBaseHeight));
        elevatorLigament =
                loggedMechanismRoot.append(
                        new LoggedMechanismLigament2d(
                                "elevator",
                                Units.inchesToMeters(
                                        elevatorInputs.leftHeightInches
                                                + WristConstants.WRIST_AXLE_HEIGHT),
                                90));
        wristLigament =
                elevatorLigament.append(
                        new LoggedMechanismLigament2d(
                                "wrist",
                                Units.inchesToMeters(WristConstants.WRIST_LENGTH),
                                90,
                                6,
                                new Color8Bit(Color.kPurple)));
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorInputs);
        wristIO.updateInputs(wristInputs);

        Logger.processInputs("Elevator", elevatorInputs);
        Logger.processInputs("Wrist", wristInputs);

        updateDriveMaxVelocity();

        // if wrist was previously restricted, but no longer needs to be
        if (wristRestricted && !isWristHeightRestricted()) {
            // set the wrist target angle to the saved value
            wristRestricted = false;
            wristIO.setTargetAngle(savedWristTargetAngle);
        }

        if (isZeroed()) {
            elevatorIO.setEncoder(ElevatorConstants.minHeight);
        }

        updateLigamentSimulation();

        elevatorIO.updateStates();
        wristIO.updateStates();
    }

    /**
     * Sets the target height of the elevator in inches
     *
     * @param heightInches
     */
    public void setElevatorTargetHeight(double heightInches) {
        elevatorIO.setTargetHeightInches(heightInches);
    }

    /**
     * Sets the target angle of the wrist, avoiding the possibility of the wrist hitting the reef
     *
     * @param targetAngle in radians, 0 being horizontal with the ground
     */
    public void setWristTargetAngle(double targetAngle) {
        // if wrist could possibly hit the reef
        if (isWristHeightRestricted() && targetAngle > WristConstants.NET) {
            // set vars to hold targetAngle until safe to move the wrist
            wristRestricted = true;
            savedWristTargetAngle = targetAngle;

            targetAngle = WristConstants.NET;
        }

        wristIO.setTargetAngle(targetAngle);
    }

    /**
     * If the wrist is safe to move
     *
     * @return boolean
     */
    public boolean isWristHeightRestricted() {
        return elevatorInputs.leftHeightInches > 50;
    }

    public double getElevatorHeight() {
        return elevatorInputs.leftHeightInches;
    }

    public double getWristAngle() {
        return wristInputs.angle;
    }

    /**
     * If the elevator & wrist are at their target values
     *
     * @return boolean
     */
    public boolean isAtSetpoint() {
        return elevatorInputs.isAtTargetLevel && wristInputs.isAtTargetAngle;
    }

    /**
     * If the elevator is at it's target height
     *
     * @return boolean
     */
    public boolean elevatorIsAtSetpoint() {
        return elevatorInputs.isAtTargetLevel;
    }

    /**
     * Whether the elevator is currently zeroed
     *
     * @return boolean
     */
    public boolean isZeroed() {
        return elevatorInputs.isLimitSwitchPressed;
    }

    /**
     * Sets the voltage of the wrist
     *
     * @param voltage in volts
     */
    public void setWristVoltage(double voltage) {
        wristIO.setVoltage(voltage);
    }

    /**
     * Sets the voltage of the elevator
     *
     * @param voltage in volts
     */
    public void setVoltage(double voltage) {
        elevatorIO.setVoltage(voltage);
    }

    public void stopElevator() {
        elevatorIO.stopMotors();
    }

    public void stopWrist() {
        wristIO.stopMotors();
    }

    public double getElevatorSetpoint() {
        return elevatorInputs.targetHeightInches;
    }

    public double getWristSetpoint() {
        return wristInputs.targetAngle;
    }

    public boolean getWristIsAtSetpoint() {
        return elevatorInputs.isAtTargetLevel;
    }

    /**
     * Updates the max velocity of the drivetrain based on the height of the elevator in a linear
     * relationship.
     */
    public void updateDriveMaxVelocity() {
        double slope =
                (levelFourSpeedLimit - maxSpeedLimitMetersPerSec) / ElevatorConstants.maxHeight;
        drive.setMaxVelocity(slope * elevatorInputs.leftHeightInches + maxSpeedLimitMetersPerSec);
    }

    private void updateLigamentSimulation() {
        elevatorLigament.setLength(
                Units.inchesToMeters(
                        elevatorInputs.leftHeightInches + WristConstants.WRIST_AXLE_HEIGHT));
        // -90 because the '0' for the wrist is horizontal with the ground
        wristLigament.setAngle(Units.radiansToDegrees(wristInputs.angle) - 90);
        Logger.recordOutput("FieldSimulation/ElevatorMech2d", loggedMechanism);
    }

    //    public Command wristSysIDQuasistatic(SysIdRoutine.Direction direction) {
    //        return wristSysIDRoutine.quasistatic(direction);
    //    }
    //
    //    public Command wristSysIDDynamic(SysIdRoutine.Direction direction) {
    //        return wristSysIDRoutine.dynamic(direction);
    //    }
    //
    //    public Command elevatorSysIDQuasistatic(SysIdRoutine.Direction direction) {
    //        return elevatorSysIDRoutine.quasistatic(direction);
    //    }
    //
    //    public Command elevatorSysIDDynamic(SysIdRoutine.Direction direction) {
    //        return elevatorSysIDRoutine.dynamic(direction);
    //    }
}
