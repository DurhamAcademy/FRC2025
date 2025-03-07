package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    IntakeIO io;
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public boolean getBeamBroken() {
        return inputs.isBeamBroken; // if this becomes noisy we can add a debouncer
    }

    public void setVoltage(double voltage) {
        io.setIntakeVoltage(voltage);
    }

    public void stopMotors() {
        io.stopMotors();
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

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.recordOutput("Intake/beamBreakBroken", getBeamBroken());
        Logger.processInputs("Intake", inputs);
    }
}
