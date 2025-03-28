package frc.robot.subsystems.drive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.*;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Class to handle Limelight vision find docs at: <a
 * href="https://limelightlib-wpijava-reference.limelightvision.io/frc/robot/package-summary.html">link</a>
 */
public class Vision extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;

    Map<String, Transform3d> cameraTransforms =
            new HashMap<>(); // key: camera name, value: camera to robot transform
    Map<PhotonCamera, PhotonPoseEstimator> cameraPoseEstimators =
            new HashMap<>(); // key: camera, value: pose estimator

    boolean useOnlyFrontCameras = false;

    private static final Matrix<N3, N1> kSingleTagStdDevs =
            VecBuilder.fill(
                    0.5,
                    0.5,
                    Math.toRadians(
                            10)); // single tag uncertainty: x and y can be off 50cm, rotation can
    // be off 10 deg
    private static final Matrix<N3, N1> kMultiTagStdDevs =
            VecBuilder.fill(
                    0.2,
                    0.2,
                    Math.toRadians(3)); // multi tag uncertainty: more confident for multiple tags

    public Vision(SwerveDrivePoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;

        // cameras on the front should start with "front"
        cameraTransforms.put(
                "front-left-camera", new Transform3d(0, 0, 0, new Rotation3d(0, 0.0, 0)));
        //                new Transform3d(
        //                        0.212725, 0.29845, 0.2413, new Rotation3d(0, 102.5 - 90, 90 -
        // 26.247)));
        /*cameraTransforms.put(
        "front-right-camera",
        new Transform3d(-0.29845, 0.212725, 0.2413, new Rotation3d(0, 102.5, 26.247)));*/

        AprilTagFieldLayout aprilTagFieldLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        for (String cameraName : cameraTransforms.keySet()) {
            Transform3d robotToCam = cameraTransforms.get(cameraName);
            try {
                PhotonCamera camera = new PhotonCamera(cameraName);

                // Construct PhotonPoseEstimator
                PhotonPoseEstimator photonPoseEstimator =
                        new PhotonPoseEstimator(
                                aprilTagFieldLayout,
                                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                robotToCam);
                photonPoseEstimator.setMultiTagFallbackStrategy(
                        PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

                cameraPoseEstimators.put(camera, photonPoseEstimator);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(
            PhotonCamera camera, PhotonPoseEstimator photonEstimator) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(change);
        }
        return visionEst;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic
     * standard deviations based on number of tags, estimation strategy, and distance from the tags.
     * from <a
     * href="https://github.com/PhotonVision/photonvision/blob/main/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java">this</a>
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private Matrix<N3, N1> calculateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose,
            List<PhotonTrackedTarget> targets,
            PhotonPoseEstimator photonEstimator) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            return kSingleTagStdDevs;
        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose.get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(
                                        estimatedPose
                                                .get()
                                                .estimatedPose
                                                .toPose2d()
                                                .getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                return kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs =
                            VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                return estStdDevs;
            }
        }
    }

    private void updateEstimatedPose() {
        Logger.recordOutput("Vision/running", true);
        for (PhotonCamera camera : cameraPoseEstimators.keySet()) {
            /*if (useOnlyFrontCameras && !camera.getName().contains("front")) {
                // skip camera because it's not on the front
                continue;
            }*/
            PhotonPoseEstimator photonPoseEstimator = cameraPoseEstimators.get(camera);
            Optional<EstimatedRobotPose> estimatedPose =
                    getEstimatedGlobalPose(camera, photonPoseEstimator);
            if (estimatedPose.isPresent()) {
                //                Logger.recordOutput("Vision/cameraOutput", true);
                Pose2d pose = estimatedPose.get().estimatedPose.toPose2d();
                Logger.recordOutput("Vision/pose3d", estimatedPose.get().estimatedPose);
                Logger.recordOutput("Vision/cameraOutput", pose);
                double timestamp = estimatedPose.get().timestampSeconds;
                Matrix<N3, N1> stdDevs =
                        calculateEstimationStdDevs(
                                estimatedPose,
                                estimatedPose.get().targetsUsed,
                                photonPoseEstimator);
                poseEstimator.addVisionMeasurement(
                        pose, timestamp, stdDevs); // taking in curStdDevs to apply confidence
            } else {
                //                Logger.recordOutput("Vision/cameraOutput", false);
            }
        }
    }

    /**
     * @return the estimated position based on limelight cameras
     */
    public Pose2d getPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        updateEstimatedPose(); // use camera data to estimate position
    }
}
