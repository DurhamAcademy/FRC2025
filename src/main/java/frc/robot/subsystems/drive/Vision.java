package frc.robot.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

/**
 * Class to handle Limelight vision find docs at: <a
 * href="https://limelightlib-wpijava-reference.limelightvision.io/frc/robot/package-summary.html">link</a>
 * TODO: tweak settings in web interface
 */
public class Vision extends SubsystemBase {
  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveModulePosition[] modulePositions;
  GyroIO.GyroIOInputs gyro;

  Pose2d currentPosition = new Pose2d(0, 0, new Rotation2d(0));

  ArrayList<CameraConfig> cameraConfigs =
      new ArrayList<>(); // holds all the cameras (initialized in constructor)

  public Vision(
      SwerveDriveKinematics kinematics,
      SwerveModulePosition[] modulePositions,
      GyroIO.GyroIOInputs gyro) {
    this.modulePositions = modulePositions;
    this.gyro = gyro;

    // create two new cameras with different positions and offsets and store them to be used for
    // position later
    // 3.66m + 2.37m = 6.03
    cameraConfigs.add(
        new CameraConfig(
            "limelight", // camera name
            new Transform3d(
                new Translation3d(0.06220, 0, 0.4683),
                new Rotation3d(0.0, Math.toRadians(-15), 0.0)), // Camera pose
            new Translation3d(0.0, 0.0, 0.0) // Fiducial offset
            ));

    /*cameraConfigs.add(
    new CameraConfig(
        "limelight2",
        new Transform3d(
            new Translation3d(0.6, 0.1, 0.4),
            new Rotation3d(0.0, Math.toRadians(20.0), 0.0)), // Camera pose
        new Transform3d(new Translation3d(0.1, 0.0, 0.6), new Rotation3d()), // Fiducial offset
        new int[] {4, 5, 6}, // Tag IDs
        1.5f // Downscaling
        ));*/

    // Initialize pose estimator
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, // Kinematics for drivetrain
            gyro.yawPosition, // Initial gyro angle
            modulePositions, // positions of swerve modules
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)) // fixme: initial position?
            );

    initializeLimelightHelpers(); // sets up all the cameras in the cameraConfigs list
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
      // accounts for the physical position of the Limelight (or other camera) relative to your
      // robot’s coordinate system or desired alignment point (e.g., the center of the robot or the
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
        cameraName, poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
  }

  /**
   * With all the cameras in the cameraConfigs array, updates the poseEstimator's estimated pose
   * using AprilTag vision. read more: <a
   * href="https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation">here</a>
   */
  private void updateEstimatedPose() {
    int iters = 0;
    for (CameraConfig camera : cameraConfigs) {
      LimelightHelpers.PoseEstimate poseEstimate = getEstimatedPoseFromCamera(camera.name);
      Logger.recordOutput("Vision/numTags" + iters, poseEstimate.tagCount);
      Logger.recordOutput("Vision/tagPose" + iters, poseEstimate.pose);
      // Reject update if no tags are detected
      if (poseEstimate.tagCount == 0) {
        continue; // skip to next camera if conditions are not met
      }
      if(Math.abs(gyro.yawVelocityRadPerSec) > 4 * Math.PI){
        continue;
      }

      // sets uncertainty values (x, y, z) - can't be trusted for z at all obviously bc robot does
      // not move in 3d space (unless actively climbing when vision doesn't really  matter)
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, Double.MAX_VALUE));

      // adds the found position to our position estimator
      poseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
      iters++;
    }
  }

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
    poseEstimator.update(gyro.yawPosition, modulePositions); // update rotation
    updateEstimatedPose(); // use camera data to estimate position
    Logger.recordOutput("RobotPosition", poseEstimator.getEstimatedPosition());
    currentPosition = poseEstimator.getEstimatedPosition();
    logRobotPosition(); // show field visualization in shuffleboard
  }
}
