package frc.robot.subsystems.drive;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Added for AprilTagFieldLayout loading
import java.util.*;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Class to handle vision for robot pose estimation. It provides a global field pose and a pose
 * relative to a shifting game element.
 */
public class Vision extends SubsystemBase {
    private final SwerveDrivePoseEstimator mainPoseEstimator; // Renamed for clarity

    // Camera names and their transformations from robot origin to camera origin
    Map<String, Transform3d> cameraTransforms = new HashMap<>();

    // Estimators for global field pose (using all field tags)
    Map<PhotonCamera, PhotonPoseEstimator> globalCameraPoseEstimators = new HashMap<>();

    // Estimators for shifting element pose (using only specified tags)
    Map<PhotonCamera, PhotonPoseEstimator> shiftingElementCameraPoseEstimators = new HashMap<>();

    // Define the Tag IDs that are on your shifting game element
    // IMPORTANT: Initialize this with the actual tag IDs from the field layout
    // that are physically mounted on your shifting element.
    // Example: if tags 5 and 8 are on the shifting element:
    public static final Set<Integer> SHIFTING_ELEMENT_TAG_IDS =
            Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22); // <<--- CONFIGURE THIS
    // If you want to initialize it empty and populate it later (e.g., from a config file),
    // you can do that, but ensure it's populated before the Vision subsystem is fully constructed.
    // public static Set<Integer> SHIFTING_ELEMENT_TAG_IDS = new HashSet<>();

    private AprilTagFieldLayout fullFieldLayout; // The complete layout from JSON
    private AprilTagFieldLayout mainFieldLayout; // Layout for global, EXCLUDING shifting tags
    private AprilTagFieldLayout shiftingElementFieldLayout; // Layout containing


    boolean useOnlyFrontCameras = false; // You can keep this if needed

    private static final Matrix<N3, N1> kSingleTagStdDevs =
            VecBuilder.fill(0.5, 0.5, Math.toRadians(10));
    private static final Matrix<N3, N1> kMultiTagStdDevs =
            VecBuilder.fill(0.2, 0.2, Math.toRadians(3));

    public Vision(SwerveDrivePoseEstimator poseEstimator) {
        this.mainPoseEstimator = poseEstimator;

        cameraTransforms.put(
                "front-left-camera",
                new Transform3d(
                        0.212725,
                        0.29845,
                        0.2413,
                        new Rotation3d(
                                0,
                                Units.degreesToRadians(-(102.5 - 90)),
                                Units.degreesToRadians(-26.247))));
        cameraTransforms.put(
                "front-right-camera",
                new Transform3d(
                        0.212725,
                        -0.29845,
                        0.2413,
                        new Rotation3d(
                                0,
                                Units.degreesToRadians(-(102.5 - 90)),
                                Units.degreesToRadians(26.247))));

        try {
            // 1. Load the FULL field layout
            fullFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded); // Or your current year

            // 2. Create the layout for the SHIFTING ELEMENT tags
            List<AprilTag> shiftingTagsList = new ArrayList<>();
            if (fullFieldLayout != null && SHIFTING_ELEMENT_TAG_IDS != null) {
                for (int id : SHIFTING_ELEMENT_TAG_IDS) {
                    fullFieldLayout.getTagPose(id).ifPresent(pose3d ->
                            shiftingTagsList.add(new AprilTag(id, pose3d))
                    );
                }
            }
            if (fullFieldLayout != null) {
                shiftingElementFieldLayout = new AprilTagFieldLayout(shiftingTagsList, fullFieldLayout.getFieldLength(), fullFieldLayout.getFieldWidth());
                // For WPILib 2024.3.1+ you might need to specify origin if it's not implicitly derived:
                // shiftingElementFieldLayout.setOrigin(fullFieldLayout.getOrigin());
            } else {
                // Fallback if fullFieldLayout failed to load
                shiftingElementFieldLayout = new AprilTagFieldLayout(new ArrayList<>(), 16.46, 8.23);
            }


            // 3. Create the MAIN field layout (for global pose), EXCLUDING shifting element tags
            List<AprilTag> mainLayoutTagsList = new ArrayList<>();
            if (fullFieldLayout != null) {
                for (AprilTag tag : fullFieldLayout.getTags()) {
                    if (SHIFTING_ELEMENT_TAG_IDS == null || !SHIFTING_ELEMENT_TAG_IDS.contains(tag.ID)) {
                        mainLayoutTagsList.add(tag); // Add if NOT in the shifting set
                    }
                }
                mainFieldLayout = new AprilTagFieldLayout(mainLayoutTagsList, fullFieldLayout.getFieldLength(), fullFieldLayout.getFieldWidth());
                // mainFieldLayout.setOrigin(fullFieldLayout.getOrigin()); // If needed for older WPILib or explicit setting
            } else {
                // Fallback if fullFieldLayout failed to load
                mainFieldLayout = new AprilTagFieldLayout(new ArrayList<>(), 16.46, 8.23);
            }


        } catch (Exception e) {
            System.err.println("CRITICAL: Could not load AprilTag field layout! Using empty layouts.");
            e.printStackTrace();
            // Initialize with empty layouts on error to prevent NPEs
            var emptyList = new ArrayList<AprilTag>();
            var defaultLength = 16.46; // Approx field length
            var defaultWidth = 8.23;   // Approx field width
            fullFieldLayout = new AprilTagFieldLayout(emptyList, defaultLength, defaultWidth);
            mainFieldLayout = new AprilTagFieldLayout(emptyList, defaultLength, defaultWidth);
            shiftingElementFieldLayout = new AprilTagFieldLayout(emptyList, defaultLength, defaultWidth);
        }

        // --- Initialize PhotonPoseEstimators for each camera ---
        for (String cameraName : cameraTransforms.keySet()) {
            Transform3d robotToCam = cameraTransforms.get(cameraName);
            try {
                PhotonCamera camera = new PhotonCamera(cameraName);

                // 1. Estimator for Global Field Pose (uses mainFieldLayout - EXCLUDES shifting tags)
                if (mainFieldLayout != null && !mainFieldLayout.getTags().isEmpty()) {
                    PhotonPoseEstimator globalPhotonPoseEstimator =
                            new PhotonPoseEstimator(
                                    mainFieldLayout, // Use the layout that EXCLUDES shifting tags
                                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                    robotToCam);
                    globalPhotonPoseEstimator.setMultiTagFallbackStrategy(
                            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
                    globalCameraPoseEstimators.put(camera, globalPhotonPoseEstimator);
                } else {
                    System.err.println("Skipping global pose estimator for " + cameraName + " due to empty or unconfigured mainFieldLayout.");
                }

                // 2. Estimator for Shifting Element Pose (uses shiftingElementFieldLayout - ONLY shifting tags)
                if (shiftingElementFieldLayout != null && !shiftingElementFieldLayout.getTags().isEmpty()) {
                    PhotonPoseEstimator shiftingElementPhotonPoseEstimator =
                            new PhotonPoseEstimator(
                                    shiftingElementFieldLayout,
                                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                    robotToCam);
                    shiftingElementPhotonPoseEstimator.setMultiTagFallbackStrategy(
                            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
                    shiftingElementCameraPoseEstimators.put(camera, shiftingElementPhotonPoseEstimator);
                } else {
                    System.err.println("Skipping shifting element pose estimator for " + cameraName + " due to empty or unconfigured shiftingElementFieldLayout.");
                }

            } catch (Exception e) {
                System.err.println("Error initializing camera or pose estimator for " + cameraName);
                e.printStackTrace();
            }
        }
    }

    /**
     * Processes camera results for a given estimator.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    private Optional<EstimatedRobotPose> getEstimatedPoseFromCamera(
            PhotonCamera camera, PhotonPoseEstimator photonEstimator) {
        if (photonEstimator == null) {
            // This can happen if the corresponding field layout was empty and the estimator wasn't
            // created.
            return Optional.empty();
        }

        // Get all available results since the last call. This also clears the internal queue.
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            return Optional.empty(); // No new results from the camera
        }

        // Get the most recent result from the list (the last one in the batch)
        PhotonPipelineResult latestResultInBatch = results.get(results.size() - 1);

        // PhotonPoseEstimator.update() takes a PhotonPipelineResult.
        // It will return Optional.empty() if it can't estimate a pose
        // (e.g., latestResultInBatch has no targets, or PnP fails).
        return photonEstimator.update(latestResultInBatch);
    }

    /** Calculates new standard deviations. */
    private Matrix<N3, N1> calculateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose,
            List<PhotonTrackedTarget> targets,
            PhotonPoseEstimator photonEstimator) { // Pass the specific estimator
        if (estimatedPose.isEmpty()
                || photonEstimator == null
                || photonEstimator.getFieldTags() == null) {
            return kSingleTagStdDevs;
        }

        var estStdDevs = kSingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        for (var tgt : targets) {
            // Use the field layout associated with THIS estimator
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get()
                            .toPose2d()
                            .getTranslation()
                            .getDistance(
                                    estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
            return kSingleTagStdDevs;
        } else {
            avgDist /= numTags;
            if (numTags > 1) estStdDevs = kMultiTagStdDevs;
            if (numTags == 1 && avgDist > 4)
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
            return estStdDevs;
        }
    }

    /** Updates the main SwerveDrivePoseEstimator with global vision measurements. */
    private void updateGlobalEstimatedPose() {
        Logger.recordOutput("Vision/GlobalPose/Running", true);
        boolean foundGlobalEstimate = false;

        for (Map.Entry<PhotonCamera, PhotonPoseEstimator> entry :
                globalCameraPoseEstimators.entrySet()) {
            PhotonCamera camera = entry.getKey();
            PhotonPoseEstimator photonPoseEstimator = entry.getValue();

            /*if (useOnlyFrontCameras && !camera.getName().contains("front")) {
                continue;
            }*/

            Optional<EstimatedRobotPose> estimatedPoseOpt =
                    getEstimatedPoseFromCamera(camera, photonPoseEstimator);

            if (estimatedPoseOpt.isPresent()) {
                foundGlobalEstimate = true;
                EstimatedRobotPose estimatedPose = estimatedPoseOpt.get();
                Pose2d pose = estimatedPose.estimatedPose.toPose2d();
                Logger.recordOutput("Vision/GlobalPose/Camera_" + camera.getName() + "/Pose", pose);
                Logger.recordOutput(
                        "Vision/GlobalPose/Camera_" + camera.getName() + "/Pose3d",
                        estimatedPose.estimatedPose);

                double timestamp = estimatedPose.timestampSeconds;
                Matrix<N3, N1> stdDevs =
                        calculateEstimationStdDevs(
                                estimatedPoseOpt,
                                estimatedPose.targetsUsed,
                                photonPoseEstimator); // Pass the global estimator

                mainPoseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
            }
        }
        Logger.recordOutput("Vision/GlobalPose/FoundEstimateThisCycle", foundGlobalEstimate);
    }

    /**
     * Returns the robot's field position based *only* on the AprilTags designated as being on the
     * shifting game element. The pose is still in the global field coordinate system.
     *
     * @return An Optional<Pose2d> of the robot's estimated field position. Empty if no shifting
     *     element tags are visible or if not configured.
     */
    public Optional<Pose2d> getShiftingElementBasedFieldPose() {
        if (shiftingElementCameraPoseEstimators.isEmpty()) {
            Logger.recordOutput("Vision/ShiftingElementPose/NoEstimatorsConfigured", true);
            return Optional.empty();
        }

        EstimatedRobotPose bestEstimate = null;
        double bestTimestamp = -1;
        PhotonCamera bestCamera = null; // For logging

        for (Map.Entry<PhotonCamera, PhotonPoseEstimator> entry :
                shiftingElementCameraPoseEstimators.entrySet()) {
            PhotonCamera camera = entry.getKey();
            PhotonPoseEstimator photonPoseEstimator = entry.getValue();

            /*if (useOnlyFrontCameras && !camera.getName().contains("front")) {
                continue;
            }*/

            Optional<EstimatedRobotPose> currentEstimateOpt =
                    getEstimatedPoseFromCamera(camera, photonPoseEstimator);

            if (currentEstimateOpt.isPresent()) {
                EstimatedRobotPose currentEstimate = currentEstimateOpt.get();
                // Strategy: prefer multi-tag, then newer timestamp
                if (bestEstimate == null
                        || (currentEstimate.strategy
                                        == PhotonPoseEstimator.PoseStrategy
                                                .MULTI_TAG_PNP_ON_COPROCESSOR
                                && bestEstimate.strategy
                                        != PhotonPoseEstimator.PoseStrategy
                                                .MULTI_TAG_PNP_ON_COPROCESSOR)
                        || (currentEstimate.strategy == bestEstimate.strategy
                                && currentEstimate.timestampSeconds > bestTimestamp)
                        || (currentEstimate.targetsUsed.size() > bestEstimate.targetsUsed.size()
                                && currentEstimate.timestampSeconds >= bestTimestamp)) {
                    bestEstimate = currentEstimate;
                    bestTimestamp = currentEstimate.timestampSeconds;
                    bestCamera = camera;
                }
            }
        }

        if (bestEstimate != null) {
            Pose2d pose = bestEstimate.estimatedPose.toPose2d();
            Logger.recordOutput("Vision/ShiftingElementPose/Pose", pose);
            Logger.recordOutput("Vision/ShiftingElementPose/Pose3d", bestEstimate.estimatedPose);
            Logger.recordOutput(
                    "Vision/ShiftingElementPose/Timestamp", bestEstimate.timestampSeconds);
            Logger.recordOutput(
                    "Vision/ShiftingElementPose/NumTags", bestEstimate.targetsUsed.size());
            if (bestCamera != null) {
                Logger.recordOutput(
                        "Vision/ShiftingElementPose/SourceCamera", bestCamera.getName());
            }
            return Optional.of(pose);
        } else {
            Logger.recordOutput("Vision/ShiftingElementPose/NoTargetsVisible", true);
            return Optional.empty();
        }
    }

    /**
     * @return the fused estimated position from the main SwerveDrivePoseEstimator.
     */
    public Pose2d getFusedFieldPosition() {
        return mainPoseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        System.out.println("asdfasdfasd");
        updateGlobalEstimatedPose(); // Update main pose estimator with global vision

        // Example of how to get and log the shifting element based pose:
        Optional<Pose2d> shiftingPose = getShiftingElementBasedFieldPose();
        if (shiftingPose.isPresent()) {
            // You can use shiftingPose.get() for logic that depends on this specific pose
            // Logger already handles logging inside getShiftingElementBasedFieldPose()
        }

        // Logging current fused pose and camera transforms for debugging
        Pose3d currentFusedPose3d =
                new Pose3d(
                        mainPoseEstimator.getEstimatedPosition().getX(),
                        mainPoseEstimator.getEstimatedPosition().getY(),
                        0.0, // Assuming Z is 0 for 2D pose on field
                        new Rotation3d(
                                mainPoseEstimator
                                        .getEstimatedPosition()
                                        .getRotation())); // Get rotation from Pose2d

        // Log camera positions in field frame based on current FUSED robot pose
        // This shows where your cameras *think* they are on the field
        Transform3d frontLeftRobotToCam = cameraTransforms.get("front-left-camera");
        if (frontLeftRobotToCam != null) {
            Logger.recordOutput(
                    "Vision/Debug/FieldPose_FrontLeftCamera",
                    currentFusedPose3d.transformBy(frontLeftRobotToCam));
        }

        Transform3d frontRightRobotToCam = cameraTransforms.get("front-right-camera");
        if (frontRightRobotToCam != null) {
            Logger.recordOutput(
                    "Vision/Debug/FieldPose_FrontRightCamera",
                    currentFusedPose3d.transformBy(frontRightRobotToCam));
        }
        Logger.recordOutput("Vision/FusedFieldPose", mainPoseEstimator.getEstimatedPosition());
    }
}
