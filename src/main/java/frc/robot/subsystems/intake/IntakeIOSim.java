package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.*;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class IntakeIOSim implements IntakeIO {
    private final SwerveDriveSimulation driveSimulation;
    private final IntakeSimulation intakeSimulation;

    public IntakeIOSim(SwerveDriveSimulation driveTrain) {
        this.driveSimulation = driveTrain;
        this.intakeSimulation =
                IntakeSimulation.OverTheBumperIntake(
                        // Specify the type of game pieces that the intake can collect
                        "Coral",
                        // Specify the drivetrain to which this intake is attached
                        driveTrain,
                        // Width of the intake
                        IntakeConstants.intakeWidth,
                        // The extension length of the intake beyond the robot's frame (when
                        // activated)
                        IntakeConstants.intakeExtensionLength,
                        // The intake is mounted on the back side of the chassis
                        IntakeSimulation.IntakeSide.BACK,
                        // The intake can hold up to 1 coral
                    1);
    }

    private boolean hasNewCoralFromCollector() {
        // find all corals
        List<ReefscapeCoralOnFly> corals = new ArrayList<>();
        for (GamePieceProjectile gamePieceProjectile :
                SimulatedArena.getInstance().gamePieceLaunched())
            if (gamePieceProjectile instanceof ReefscapeCoralOnFly coral) corals.add(coral);

        // choose those close enough to intake
        for (ReefscapeCoralOnFly coral : corals)
            if (insideIntakeRange(driveSimulation.getSimulatedDriveTrainPose(), coral.getPose3d()))
                return SimulatedArena.getInstance().removeProjectile(coral);

        return false;
    }

    private boolean insideIntakeRange(Pose2d simulatedDriveTrainPose, Pose3d coralPositionInAir) {
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

    /*@Override
    public void simAddCoral(Pose2d robotPose) {
        Logger.recordOutput("Intake/numPieces", intakeSimulation.getGamePiecesAmount());
        if (intakeSimulation.getGamePiecesAmount() > 0) return; // quit if already have one
        intakeSimulation.startIntake(); // allow for coral to be added
        // spawn new game piece in intake so it picks it up
        SimulatedArena.getInstance()
                .addGamePiece(
                        new ReefscapeCoralOnField(
                                new Pose2d(
                                        robotPose.getX(), robotPose.getY(), new Rotation2d(90))));
        intakeSimulation.stopIntake(); // close intake
    }*/

    /*@Override
    public void setIntakePercent(double percent) {
        if (percent == 0.0) intakeSimulation.stopIntake(); // not running if percent is 0
        else intakeSimulation.startIntake(); // if setting speed above 0, run intake
    }

    @Override
    public void setIntakeVoltage(double volts) {
        if (volts == 0.0) intakeSimulation.stopIntake(); // not running if speed is 0
        else intakeSimulation.startIntake(); // if setting speed above 0, run intake
    }*/

    /*@Override // Defined by IntakeIO
    public boolean isNoteInsideIntake() {
        return intakeSimulation.getGamePiecesAmount() != 0; // True if there is a game piece in the intake
    }

    @Override // Defined by IntakeIO
    public void launchNote() {
        // if there is a note in the intake, it will be removed and return true; otherwise, returns false
        if (intakeSimulation.obtainGamePieceFromIntake()) return; // TODO: make this shoot once end effector is done
    }*/
}
