package frc.robot.subsystems.drive;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

/**
 * Class to handle Limelight vision find docs at: <a
 * href="https://limelightlib-wpijava-reference.limelightvision.io/frc/robot/package-summary.html">link</a>
 */
public class Vision extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    GyroIO.GyroIOInputs gyro;
    Drive drive;

    Pose2d currentPosition = new Pose2d(0, 0, new Rotation2d(0));

    ArrayList<CameraConfig> cameraConfigs =
            new ArrayList<>(); // holds all the cameras (initialized in constructor)

    public Vision(GyroIO.GyroIOInputs gyro, Drive drive) {
        this.gyro = gyro;
        this.drive = drive;

        // create two new cameras with different positions and offsets and store them to be used for
        // position later
        cameraConfigs.add(
                new CameraConfig(
                        "limelight", // camera name
                        new Transform3d(
                                new Translation3d(0.06220, 0, 0.4683),
                                new Rotation3d(0.0, Math.toRadians(-15), 0.0)), // Camera pose
                        new Translation3d(0.0, 0.0, 0.0) // Fiducial offset
                        ));

        // Initialize pose estimator
        poseEstimator = drive.poseEstimator;

        // TODO do we want to do this in code or via limelight local
        // initializeLimelightHelpers(); // sets up all the cameras in the cameraConfigs list
    }

    /** sets up limelight cameras from cameraConfig list */
    private void initializeLimelightHelpers() {
        // run for each camera in list
        for (CameraConfig camera : cameraConfigs) {
            // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
            LimelightHelpers.setCameraPose_RobotSpace(
                    camera.name,
                    camera.cameraPose.getTranslation().getX(),
                    camera.cameraPose.getTranslation().getY(),
                    camera.cameraPose.getTranslation().getZ(),
                    Math.toDegrees(camera.cameraPose.getRotation().getX()),
                    Math.toDegrees(camera.cameraPose.getRotation().getY()),
                    Math.toDegrees(camera.cameraPose.getRotation().getZ()));

            // Set AprilTag offset tracking point (meters)
            // accounts for the physical position of the Limelight (or other camera) relative to
            // your
            // robot’s coordinate system or desired alignment point (e.g., the center of the robot
            // or the
            // shooter mechanism)
            LimelightHelpers.setFiducial3DOffset(
                    camera.name,
                    camera.fiducialOffset.getX(),
                    camera.fiducialOffset.getY(),
                    camera.fiducialOffset.getZ());
        }
    }

    /**
     * Given any limelight camera on the robot, returns the estimated pose from that camera
     *
     * @param cameraName name of limelight camera
     * @return the PoseEstimate obj from that camera, including estimated pose, timestamp, num tags,
     *     etc
     */
    private LimelightHelpers.PoseEstimate getEstimatedPoseFromCamera(String cameraName) {
        // Update Limelight robot orientation from pose estimator
        LimelightHelpers.SetRobotOrientation(
                cameraName,
                poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
                0,
                0,
                0,
                0,
                0);
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
    }

    /**
     * With all the cameras in the cameraConfigs array, updates the poseEstimator's estimated pose
     * using AprilTag vision. read more: <a
     * href="https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation">here</a>
     */
    private void updateEstimatedPose() {
        for (CameraConfig camera : cameraConfigs) {
            LimelightHelpers.PoseEstimate poseEstimate = getEstimatedPoseFromCamera(camera.name);

            // Reject update if no tags are detected
            if (poseEstimate == null || poseEstimate.tagCount == 0) {
                continue; // skip to next camera if conditions are not met
            }

            // reject update if rotating too fast
            if (Math.abs(gyro.yawVelocityRadPerSec) > 4 * Math.PI) {
                continue;
            }

            // adds the found position to our position estimator
            poseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
        }
    }

    /** Logs current robot position */
    private void logRobotPosition() {
        Logger.recordOutput("RobotPosition", currentPosition);
    }

    /**
     * @return the estimated position based on limelight cameras
     */
    public Pose2d getPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        //        poseEstimator.update(gyro.yawPosition, modulePositions); // update rotation
        updateEstimatedPose(); // use camera data to estimate position
        //        currentPosition = getPosition();
        //        logRobotPosition(); // show field visualization in shuffleboard
    }
}
