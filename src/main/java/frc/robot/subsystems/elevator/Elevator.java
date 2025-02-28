package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    private final WristIO wristIO;
    private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();
    private final Drive drive;

    private boolean wristRestricted = false;
    private double savedWristTargetAngle = 0.0;

    private static final LoggedMechanism2d loggedMechanism = new LoggedMechanism2d(3, 3);
    public LoggedMechanismRoot2d loggedMechanismRoot;
    public LoggedMechanismLigament2d wristLigament;
    public LoggedMechanismLigament2d elevatorLigament;

    SysIdRoutine elevatorSysIDRoutine;
    SysIdRoutine wristSysIDRoutine;

    public enum ElevatorLevel {
        ZERO(ElevatorConstants.ZERO, WristConstants.STARTING),
        INTAKE(ElevatorConstants.ZERO, WristConstants.INTAKE),
        L1(ElevatorConstants.ZERO, WristConstants.LOWER_ALGAE_REMOVAL),
        L2(ElevatorConstants.ZERO, WristConstants.L2),
        L3(ElevatorConstants.ZERO, WristConstants.L3),
        L4(ElevatorConstants.ZERO, WristConstants.NET),
        NET(ElevatorConstants.L4, WristConstants.NET),
        PROCESSOR(ElevatorConstants.L1, WristConstants.PROCESSOR),
        LOWER_ALGAE_REMOVAL(
                ElevatorConstants.LOWER_ALGAE_REMOVAL, WristConstants.LOWER_ALGAE_REMOVAL),
        UPPER_ALGAE_REMOVAL(
                ElevatorConstants.UPPER_ALGAE_REMOVAL, WristConstants.UPPER_ALGAE_REMOVAL);

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

        elevatorSysIDRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> elevatorIO.setVoltage(voltage.in(Volts)), null, this));

        wristSysIDRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                Volts.per(Second).of(.5),
                                Volts.of(2),
                                Seconds.of(5),
                                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> wristIO.setVoltage(voltage.in(Volts)), null, this));

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
        // if wrist was previously restricted, but no longer needs to be
        if (wristRestricted && !isWristRestricted()) {
            // set the wrist target angle to the saved value
            wristRestricted = false;
            wristIO.setTargetAngle(savedWristTargetAngle);
        }

        elevatorIO.updateInputs(elevatorInputs);
        wristIO.updateInputs(wristInputs);
        Logger.processInputs("Elevator", elevatorInputs);
        Logger.processInputs("Wrist", wristInputs);

        if (elevatorInputs.isLimitSwitchPressed) {
            elevatorIO.setEncoder(ElevatorConstants.minHeight);
        }

        elevatorIO.updateProfile();
        wristIO.updateStates();

        elevatorLigament.setLength(
                Units.inchesToMeters(
                        elevatorInputs.leftHeightInches + WristConstants.WRIST_AXLE_HEIGHT));
        // -90 because the '0' for the wrist is horizontal with the ground
        wristLigament.setAngle(Units.radiansToDegrees(wristInputs.angle) - 90);
        Logger.recordOutput("FieldSimulation/ElevatorMech2d", loggedMechanism);
    }

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
        if (false) {
            // set vars to hold targetAngle until safe to move the wrist
            wristRestricted = true;
            savedWristTargetAngle = targetAngle;

            // get the safe angles for the wrist when right next to the reef + extra room
            double restrictedAngle =
                    Math.acos(WristConstants.REEF_MIN_DISTANCE / WristConstants.WRIST_LENGTH) + .1;

            // if the target angle is in the restricted area, set it to the closest angle it can get
            // to safely
            if (Math.abs(targetAngle) < restrictedAngle) {
                targetAngle = wristInputs.angle > 0 ? restrictedAngle : -restrictedAngle;
            }
            // if the target angle is on the opposite side of the restricted area from the wrist's
            // current angle,
            // set it to the closest angle it can get to safely
            else if (targetAngle > 0 && wristInputs.angle < 0) {
                targetAngle = -restrictedAngle;
            } else if (targetAngle < 0 && wristInputs.angle > 0) {
                targetAngle = restrictedAngle;
            }
        }
        wristIO.setTargetAngle(targetAngle);
    }

    /**
     * Determines if wrist should be restricted
     *
     * @return boolean
     */
    public boolean isWristRestricted() {
        // if the robot isn't near the closest reef, allow normal wrist movement
        if (drive.getPose()
                        .getTranslation()
                        .getDistance(
                                drive.getReefPose(drive.getClosestTargetReef()).getTranslation())
                > 1) {
            return false;
        }

        // if elevator height (off the ground) > than the reef height +
        // the height of a triangle formed by the wrist and the robot's minimum distance to the reef
        return elevatorInputs.leftHeightInches
                        + ElevatorConstants.elevatorBaseHeight
                        + WristConstants.WRIST_AXLE_HEIGHT
                < WristConstants.REEF_PANEL_HEIGHT
                        + Math.sqrt(
                                Math.pow(WristConstants.WRIST_LENGTH, 2)
                                        - Math.pow(WristConstants.REEF_MIN_DISTANCE, 2));
    }

    /**
     * Sets the speed of the wrist
     *
     * @param speed The desired speed level for the motor: -1.0 => full reverse, 1.0 => full
     *     forward, 0.0 => no speed.
     */
    public void setWristSpeed(double speed) {
        wristIO.setSpeed(speed);
    }

    public void recreateWristFeedforward() {
        wristIO.recreateFeedforward();
    }

    public void resetWristConfig() {
        wristIO.resetConfig();
    }

    public double getElevatorHeight() {
        return elevatorInputs.leftHeightInches;
    }

    /**
     * Sets the voltage of the wrist
     *
     * @param voltage in volts
     */
    public void setWristVoltage(double voltage) {
        wristIO.setVoltage(voltage);
    }

    public void setElevatorPower(double power) {
        elevatorIO.setPower(power);
    }

    public void setVoltage(double voltage) {
        elevatorIO.setVoltage(voltage);
    }

    public boolean isZeroed() {
        return elevatorInputs.isLimitSwitchPressed;
    }

    public Command wristSysIDQuasistatic(SysIdRoutine.Direction direction) {
        return wristSysIDRoutine.quasistatic(direction);
    }

    public Command wristSysIDDynamic(SysIdRoutine.Direction direction) {
        return wristSysIDRoutine.dynamic(direction);
    }

    public Command elevatorSysIDQuasistatic(SysIdRoutine.Direction direction) {
        return elevatorSysIDRoutine.quasistatic(direction);
    }

    public Command elevatorSysIDDynamic(SysIdRoutine.Direction direction) {
        return elevatorSysIDRoutine.dynamic(direction);
    }

    /*
    TODO this is sys ID stuff to do later
    operatorController
                .povUp()
                .whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        operatorController
                .povDown()
                .whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        operatorController
                .povLeft()
                .whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
        operatorController
                .povRight()
                .whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
     */
}
