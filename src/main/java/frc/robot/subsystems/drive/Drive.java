// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import frc.robot.util.LocalADStarAK;
import java.util.Map;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    public static double maxUsableSpeedMetersPerSec = maxSpeedMetersPerSec;
    private final GyroIO gyroIO;
    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private Rotation2d rawGyroRotation = new Rotation2d();

    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    static final Lock odometryLock = new ReentrantLock();
    private final Consumer<Pose2d> resetSimulationPoseCallBack;
    private final SysIdRoutine sysId;
    private final Vision vision;
    private final RobotContainer robotContainer;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
    private final SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
    private final SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(
                    kinematics,
                    rawGyroRotation,
                    lastModulePositions,
                    new Pose2d(3, 3, new Rotation2d()));

    public Constants.ReefConstants targetReef = Constants.ReefConstants.SEVEN;
    public boolean overrideReefAutoAlign = false;
    public boolean overrideTipProtection = false;

    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO,
            Consumer<Pose2d> resetSimulationPoseCallBack,
            RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.gyroIO = gyroIO;
        this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

        // Usage reporting for swerve template
        HAL.report(
                tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        SparkOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                this::runVelocity,
                new PPHolonomicDriveController(
                        new PIDConstants(10.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                ppConfig,
                () -> DriverStation.getAlliance().orElse(Blue) == Red,
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) ->
                        Logger.recordOutput(
                                "Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });

        // Configure SysId
        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) ->
                                        Logger.recordOutput("Drive/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
        vision = new Vision(gyroInputs, this);
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps =
                modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] =
                        new SwerveModulePosition(
                                modulePositions[moduleIndex].distanceMeters
                                        - lastModulePositions[moduleIndex].distanceMeters,
                                modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode == Mode.SIM);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Check if we are in game-piece-oriented mode
        /*if (isGamePieceOriented && DriverStation.getAlliance().isPresent()) {
            // Get the desired direction for the game piece
            Pose2d targetPose =
                    Constants.LocationConstants.ReefLocations.get(reefToAlign)[
                            Constants.getAllianceColor(DriverStation.getAlliance().get())];
            Translation2d targetDirection =
                    targetPose
                            .getTranslation()
                            .minus(poseEstimator.getEstimatedPosition().getTranslation());
            double angleToTarget = Math.atan2(targetDirection.getY(), targetDirection.getX());
            Rotation2d targetRotation = new Rotation2d(angleToTarget);

            // Rotate input speeds to align with target direction
            speeds =
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            speeds.vxMetersPerSecond,
                            speeds.vyMetersPerSecond,
                            speeds.omegaRadiansPerSecond,
                            targetRotation // Override field orientation with target orientation
                            );
        }*/

        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxUsableSpeedMetersPerSec);

        // Log unoptimized setpoints
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules
     * will return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = moduleTranslations[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.dynamic(direction));
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rad/sec. */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        if (Constants.currentMode == Mode.SIM && robotContainer.getDriveSimulation() != null) {
            return robotContainer.getDriveSimulation().getSimulatedDriveTrainPose();
        }
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        if (Constants.currentMode == Mode.SIM && robotContainer.getDriveSimulation() != null) {
            return robotContainer.getDriveSimulation().getSimulatedDriveTrainPose().getRotation();
        }
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        resetSimulationPoseCallBack.accept(pose);
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return maxUsableSpeedMetersPerSec;
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return maxUsableSpeedMetersPerSec / driveBaseRadius;
    }

    public Module getModule(int index) {
        return modules[index];
    }

    public Constants.ReefConstants getClosestReef() {
        Constants.ReefConstants closestReef = Constants.ReefConstants.SIX;
        if (!overrideReefAutoAlign && DriverStation.getAlliance().isPresent()) {
            int alliance = Constants.getAllianceColor(DriverStation.getAlliance().get());

            // Find closest reef position to current pose
            Pose2d estimatedReefPose =
                    poseEstimator
                            .getEstimatedPosition()
                            .nearest(Constants.LocationConstants.PosesOfAllReefLocations(alliance));

            // Find corresponding reef constant value
            closestReef =
                    Constants.LocationConstants.ReefLocations.entrySet().stream()
                            .filter(entry -> entry.getValue()[alliance].equals(estimatedReefPose))
                            .map(Map.Entry::getKey)
                            .findFirst()
                            .orElse(Constants.ReefConstants.SIX);
        }
        return closestReef;
    }

    public enum ReefAlignSide {
        LEFT,
        RIGHT
    }

    /**
     * Sets reef target to the nearest reef on a certain side
     *
     * @param side the side of each flat panel of the reef hexagon to align to
     */
    public void setTargetReefToClosest(ReefAlignSide side) {
        // Define the left-right reef pairs
        Map<Integer, Integer> reefPairs =
                Map.of(
                        10, 11,
                        2, 3,
                        9, 8,
                        7, 6,
                        5, 4,
                        12, 1);

        // Retrieve the closest reef
        Constants.ReefConstants closestReef = getClosestReef();
        int closestReefId = closestReef.ordinal() + 1; // Enums are 0-indexed

        // Determine the target reef based on the required side
        int targetReefId =
                switch (side) {
                    case RIGHT -> reefPairs.getOrDefault(
                            closestReefId, closestReefId); // Go to left
                    case LEFT -> reefPairs.entrySet().stream()
                            .filter(entry -> entry.getValue() == closestReefId)
                            .map(Map.Entry::getKey)
                            .findFirst()
                            .orElse(closestReefId); // Go to right
                };

        // Update the target reef
        Constants.ReefConstants oldTargetReef = targetReef;
        targetReef = Constants.ReefConstants.values()[targetReefId - 1];

        // Update visualization if the reef has changed
        if (oldTargetReef != targetReef) {
            updateDashboardReefVisualization(targetReef.ordinal());
        }
    }

    public Pose2d getReefPose(Constants.ReefConstants reef) {
        int alliance =
                DriverStation.getAlliance().isPresent()
                        ? Constants.getAllianceColor(DriverStation.getAlliance().get())
                        : 0;
        return Constants.LocationConstants.ReefLocations.get(reef)[alliance];
    }

    public Pose2d getTargetReefPose() {
        return getReefPose(targetReef);
    }

    public Constants.ReefConstants getTargetReef() {
        return targetReef;
    }

    /**
     * Function that returns whether the gyro pitch or roll is greater than the specified tipping
     * threshold
     */
    public boolean isTipping() {
        if (overrideTipProtection) return false;
        return (Math.abs(gyroInputs.pitchPosition.getDegrees()) > tippingThresholdDegrees
                || Math.abs(gyroInputs.rollPosition.getDegrees()) > tippingThresholdDegrees);
    }

    public Pose2d getNearestHumanPlayerStation() {
        int alliance =
                DriverStation.getAlliance().isPresent()
                        ? Constants.getAllianceColor(DriverStation.getAlliance().get())
                        : 0;
        Logger.recordOutput(
                "HumanPlayerStation/target",
                poseEstimator
                        .getEstimatedPosition()
                        .nearest(Constants.PosesOfAllHumanPlayerStations(alliance)));

        return poseEstimator
                .getEstimatedPosition()
                .nearest(Constants.PosesOfAllHumanPlayerStations(alliance));
    }

    public Pose2d getNearestHumanPlayerStation() {
        int alliance =
                DriverStation.getAlliance().isPresent()
                        ? Constants.getAllianceColor(DriverStation.getAlliance().get())
                        : 0;
        Logger.recordOutput(
                "HumanPlayerStation/target",
                poseEstimator
                        .getEstimatedPosition()
                        .nearest(Constants.PosesOfAllHumanPlayerStations(alliance)));

        return poseEstimator
                .getEstimatedPosition()
                .nearest(Constants.PosesOfAllHumanPlayerStations(alliance));
    }

    public double getMaxVelocity() {
        clampMaxUsableSpeed();
        return maxUsableSpeedMetersPerSec;
    }

    public void clampMaxUsableSpeed() {
        maxUsableSpeedMetersPerSec =
                MathUtil.clamp(maxUsableSpeedMetersPerSec, 0.0, maxSpeedMetersPerSec);
    }

    public Pose2d getProcessor() {
        int alliance =
                DriverStation.getAlliance().isPresent()
                        ? Constants.getAllianceColor(DriverStation.getAlliance().get())
                        : 0;
        return Constants.LocationConstants.processorLocation[alliance];
    }

    /**
     * @return boolean, is robot is within tolerance of target location
     */
    public boolean isAlignedToReef() {

        // Overall condition to stop this command (robot
        // must be at goal pose)
        Pose2d currentPose = getPose();
        Pose2d targetPose =
                DriveCommands.calculateRobotTargetPose(this, DriveCommands.autoAlignLocations.reef);
        // Calculate distance and rotation
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double rotationError =
                Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees());

        // Stop when BOTH distance and orientation are
        // within the thresholds

        boolean alignedToReef =
                distance
                                < Constants.coralInnerWidth
                                        - Constants.reefPipeDiameter
                                        - Units.inchesToMeters(.25)
                        && rotationError < 2.0; // 2.5 inches and < 2 degrees
        Logger.recordOutput("Vision/alignedToReef", alignedToReef);
        return alignedToReef;
    }

    public void updateDashboardReefVisualization(int reefIndex) {
        for (int i = 1; i <= 12; i++) {
            final int index = i;
            SmartDashboard.putData(
                    "Target Reef",
                    builder -> {
                        builder.setSmartDashboardType("Boolean");
                        builder.addBooleanProperty(
                                "Target Reef" + index, () -> reefIndex + 1 == index, null);
                    });
        }
    }
}
