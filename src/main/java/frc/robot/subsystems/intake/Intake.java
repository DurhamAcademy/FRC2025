package frc.robot.subsystems.intake;

import static edu.wpi.first.math.filter.Debouncer.DebounceType.kBoth;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private double intakeVoltageSetpoint = 0.0;
    private Rotation2d targetRotation = new Rotation2d();

    IntakeIO io;
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    Debouncer debouncer = new Debouncer(.05, kBoth);
    ArmFeedforward rotatorFF;
    ProfiledPIDController rotatorFB;
    private final TrapezoidProfile.Constraints constraints;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;

    public Intake(IntakeIO io) {
        this.io = io;
        rotatorFF =
                new ArmFeedforward(
                        IntakeConstants.rotatorKs,
                        IntakeConstants.rotatorKg,
                        IntakeConstants.rotatorKv,
                        IntakeConstants.rotatarKa);
        rotatorFB =
                new ProfiledPIDController(
                        IntakeConstants.rotatorKp,
                        IntakeConstants.rotatorKi,
                        IntakeConstants.rotatorKd,
                        new TrapezoidProfile.Constraints(
                                IntakeConstants.rotatorMaxVelocity,
                                IntakeConstants.rotatorMaxAcceleration));

        constraints =
                new TrapezoidProfile.Constraints(
                        IntakeConstants.rotatorMaxVelocity, // in/s
                        IntakeConstants.rotatorMaxAcceleration); // in/s
        currentState = new TrapezoidProfile.State(inputs.rotatorPosRad, 0);
        goalState = new TrapezoidProfile.State(0, 0);
        profile = new TrapezoidProfile(constraints);
    }

    public boolean getBeamBroken() {
        return !debouncer.calculate(
                inputs.isBeamBroken); // TODO: might need to invert this? not invert it?
    }

    public void setVoltage(double voltage) {
        intakeVoltageSetpoint = voltage;
    }

    private static boolean simInsideIntakeRange(
            Pose2d simulatedDriveTrainPose, Pose3d coralPositionInAir) {
        Translation3d robotPositionOnField =
                new Translation3d(simulatedDriveTrainPose.getTranslation());
        Rotation3d robotOrientation = new Rotation3d(simulatedDriveTrainPose.getRotation());
        Translation3d intakePositionOnField =
                robotPositionOnField.plus(
                        IntakeConstants.intakePositionOnRobot.rotateBy(robotOrientation));

        Translation3d difference = coralPositionInAir.getTranslation().minus(intakePositionOnField);
        return Math.abs(difference.getX()) < IntakeConstants.intakeRange.getX()
                && Math.abs(difference.getY()) < IntakeConstants.intakeRange.getY()
                && Math.abs(difference.getZ()) < IntakeConstants.intakeRange.getZ();
    }

    public void simCheckForCoral(Pose2d robotPose) {
        for (GamePieceProjectile gp : SimulatedArena.getInstance().gamePieceLaunched()) {
            if (gp instanceof ReefscapeCoralOnFly) {
                if (simInsideIntakeRange(robotPose, gp.getPose3d())) {
                    SimulatedArena.getInstance().removeProjectile(gp);
                }
            }
        }
    }

    public void setTargetRotation(Rotation2d targetRotation) {
        this.targetRotation = targetRotation;
        goalState = new TrapezoidProfile.State(targetRotation.getRadians(), 0);
    }

    private void rotateIntake() {
        currentState = profile.calculate(0.02, currentState, goalState);
        double ffVolts =
                rotatorFF.calculate(
                        targetRotation.getRadians(), 0); // Feedforward (for holding position)
        io.setRotatorReference(currentState.position, ffVolts);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.setIntakeVoltage(intakeVoltageSetpoint);
        rotateIntake();
        Logger.recordOutput("Intake/isRunning", intakeVoltageSetpoint != 0);
    }
}
