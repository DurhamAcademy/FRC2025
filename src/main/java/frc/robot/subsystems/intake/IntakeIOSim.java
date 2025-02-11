package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSim implements IntakeIO {
    private final IntakeSimulation intakeSimulation;

    public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
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

    @Override
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
    }

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
