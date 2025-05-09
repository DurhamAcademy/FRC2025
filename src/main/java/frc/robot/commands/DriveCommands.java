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

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
    public static final double DEADBAND = 0.1;
    public static final double ANGLE_KP = 1.0;
    public static final double ANGLE_KD = 0.4;

    // TODO: update these numbers
    public static final double LINEAR_MAX_ACCELERATION = 11.77;
    public static final double ANGLE_MAX_VELOCITY = 12.37;
    // TODO change this with characterization
    public static final double ANGLE_MAX_ACCELERATION = 74.34;

    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private DriveCommands() {}

    /*-----------------
    ----- MOVEMENT ----
    -----------------*/
    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     *
     * @param drive drive
     * @param xSupplier left joystick x value
     * @param ySupplier left joystick y value
     * @param omegaSupplier right joystick x value
     * @return command for robot
     */
    public static Command joystickDrive(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Get linear velocity to apply to robot
                    Translation2d linearVelocity =
                            getLinearVelocityFromJoysticks(
                                    xSupplier.getAsDouble(), ySupplier.getAsDouble());

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square rotation value for more precise control
                    omega = Math.copySign(omega * omega, omega);

                    // Convert to field relative speeds & send command
                    // Basically say, go this much x, go this much y, and turn this much
                    ChassisSpeeds speeds =
                            new ChassisSpeeds(
                                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                    omega * drive.getMaxAngularSpeedRadPerSec());

                    // See if rotation should be flipped, red = flipped, blue = normal
                    boolean isFlipped =
                            DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red;

                    Logger.recordOutput("Drive/isFlipped", isFlipped);

                    // Run the velocity on the drive
                    drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation()));
                },
                drive);
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     *
     * @param drive drive
     * @param xSupplier left joystick x value
     * @param ySupplier left joystick y value
     * @param rotationSupplier the rotation to lock onto
     * @return command for robot
     */
    public static Command joystickDriveAtAngle(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Rotation2d> rotationSupplier) {
        // Create PID controller that deals with rotation
        ProfiledPIDController angleController =
                new ProfiledPIDController(
                        ANGLE_KP,
                        0.0,
                        ANGLE_KD,
                        new TrapezoidProfile.Constraints(
                                ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));

        // Enable continuous input
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.run(
                        () -> {
                            // Get linear velocity to apply to robot
                            Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(
                                            xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            // Calculate angular speed
                            double omega =
                                    angleController.calculate(
                                            drive.getRotation().getRadians(),
                                            rotationSupplier.get().getRadians());

                            // Convert to field relative speeds & send command
                            ChassisSpeeds speeds =
                                    new ChassisSpeeds(
                                            linearVelocity.getX()
                                                    * drive.getMaxLinearSpeedMetersPerSec(),
                                            linearVelocity.getY()
                                                    * drive.getMaxLinearSpeedMetersPerSec(),
                                            omega);

                            // See if rotation should be flipped, red = flipped, blue = normal
                            boolean isFlipped =
                                    DriverStation.getAlliance().isPresent()
                                            && DriverStation.getAlliance().get() == Alliance.Red;

                            // Run the velocity on the drive
                            drive.runVelocity(
                                    ChassisSpeeds.fromFieldRelativeSpeeds(
                                            speeds,
                                            isFlipped
                                                    ? drive.getRotation()
                                                            .plus(new Rotation2d(Math.PI))
                                                    : drive.getRotation()));
                        },
                        drive)
                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }

    /*-----------------
     -CHARACTERIZATION-
    -----------------*/

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(
                        () -> {
                            velocitySamples.clear();
                            voltageSamples.clear();
                        }),

                // Allow modules to orient
                Commands.run(() -> drive.runCharacterization(0.0), drive)
                        .withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                                () -> {
                                    double voltage = timer.get() * FF_RAMP_RATE;
                                    drive.runCharacterization(voltage);
                                    velocitySamples.add(drive.getFFCharacterizationVelocity());
                                    voltageSamples.add(voltage);
                                },
                                drive)
                        .finallyDo(
                                () -> {
                                    int n = velocitySamples.size();
                                    double sumX = 0.0;
                                    double sumY = 0.0;
                                    double sumXY = 0.0;
                                    double sumX2 = 0.0;
                                    for (int i = 0; i < n; i++) {
                                        sumX += velocitySamples.get(i);
                                        sumY += voltageSamples.get(i);
                                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                                    }
                                    double kS =
                                            (sumY * sumX2 - sumX * sumXY)
                                                    / (n * sumX2 - sumX * sumX);
                                    double kV =
                                            (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                                    NumberFormat formatter = new DecimalFormat("#0.00000");
                                    System.out.println(
                                            "********** Drive FF Characterization Results"
                                                    + " **********");
                                    System.out.println("\tkS: " + formatter.format(kS));
                                    System.out.println("\tkV: " + formatter.format(kV));
                                }));
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(() -> limiter.reset(0.0)),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                drive)),

                // Measurement sequence
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Commands.waitSeconds(1.0),

                        // Record starting measurement
                        Commands.runOnce(
                                () -> {
                                    state.positions =
                                            drive.getWheelRadiusCharacterizationPositions();
                                    state.lastAngle = drive.getRotation();
                                    state.gyroDelta = 0.0;
                                }),

                        // Update gyro delta
                        Commands.run(
                                        () -> {
                                            var rotation = drive.getRotation();
                                            state.gyroDelta +=
                                                    Math.abs(
                                                            rotation.minus(state.lastAngle)
                                                                    .getRadians());
                                            state.lastAngle = rotation;
                                        })
                                .finallyDo(
                                        () -> {
                                            double[] positions =
                                                    drive.getWheelRadiusCharacterizationPositions();
                                            double wheelDelta = 0.0;
                                            for (int i = 0; i < 4; i++) {
                                                wheelDelta +=
                                                        Math.abs(positions[i] - state.positions[i])
                                                                / 4.0;
                                            }
                                            double wheelRadius =
                                                    (state.gyroDelta
                                                                    * DriveConstants
                                                                            .driveBaseRadius)
                                                            / wheelDelta;

                                            NumberFormat formatter = new DecimalFormat("#0.000");
                                            System.out.println(
                                                    "********** Wheel Radius Characterization"
                                                            + " Results **********");
                                            System.out.println(
                                                    "\tWheel Delta: "
                                                            + formatter.format(wheelDelta)
                                                            + " radians");
                                            System.out.println(
                                                    "\tGyro Delta: "
                                                            + formatter.format(state.gyroDelta)
                                                            + " radians");
                                            System.out.println(
                                                    "\tWheel Radius: "
                                                            + formatter.format(wheelRadius)
                                                            + " meters, "
                                                            + formatter.format(
                                                                    Units.metersToInches(
                                                                            wheelRadius))
                                                            + " inches");
                                        })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }

    /*-----------------
    ------ ALIGN ------
    -----------------*/

    /**
     * Calculates the target pose for the robot using the target reef pose
     *
     * @param drive, the drive subsystem
     * @return Pose2d, the target pose for the robot
     */
    public static Pose2d calculateRobotTargetPose(Drive drive, autoAlignLocations location) {
        Pose2d locationPose = new Pose2d();
        drive.currentAlignLocation = location;
        if (location == autoAlignLocations.reef) {
            // gets reef goal pose
            locationPose = drive.getTargetReefPose();

        } else if (location == autoAlignLocations.algae) {
            locationPose = drive.getTargetAlgaePose();

        } else if (location == autoAlignLocations.processor) {
            locationPose = drive.getProcessor();
        }

        double shiftDistance = DriveConstants.robotWidth - .28 - Units.inchesToMeters(1);
        Rotation2d shiftRotation =
                Rotation2d.fromDegrees(
                        locationPose.getRotation().getDegrees()
                                + 180); // Reverse the rotation by 180 degrees
        Pose2d goalPose =
                new Pose2d(
                        locationPose.getX()
                                - shiftDistance * shiftRotation.getCos(), // Move backward in X
                        // based
                        // on rotation
                        locationPose.getY()
                                - shiftDistance * shiftRotation.getSin(), // Move backward in Y
                        // based
                        // on rotation
                        shiftRotation);

        Logger.recordOutput("AutoAlign/goalPose", goalPose);

        return goalPose;
    }

    public enum autoAlignLocations {
        reef,
        algae,
        processor
    }

    /**
     * Roughly aligns to target position using AutoBuilder
     *
     * @param drive subsystem
     * @return Command, command containing auto builder to goal location
     */
    public static Command roughAlignToTarget(Drive drive, autoAlignLocations location) {
        ProfiledPIDController angleController =
                new ProfiledPIDController(
                        ANGLE_KP,
                        0.0,
                        ANGLE_KD,
                        new TrapezoidProfile.Constraints(
                                // TODO change this max acceleration
                                MathUtil.clamp(drive.getMaxVelocity(), 0, 2), 1));
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        PathConstraints constraints =
                new PathConstraints(
                        MathUtil.clamp(drive.getMaxVelocity(), 0, 2),
                        1,
                        drive.getMaxAngularSpeedRadPerSec() / 2,
                        1 / DriveConstants.driveBaseRadius);

        return Commands.defer(
                        () -> {
                            // Get goal position of robot
                            Pose2d goalPose = calculateRobotTargetPose(drive, location);

                            // Return the actual pathfinding command
                            return AutoBuilder.pathfindToPose(goalPose, constraints, 0.0);
                        },
                        Set.of(drive))
                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }

    /**
     * Precisely aligns to target position using chassis speeds & set precision values
     *
     * @param drive subsystem
     * @return Command, command containing drive.runVelocity() to goal location
     */
    public static Command preciseAlignToTarget(Drive drive, autoAlignLocations location) {
        // makes a new controller with PID values for correction
        HolonomicDriveController holonomicDriveController =
                new HolonomicDriveController(
                        // a value in the kp param is how many meters the robot should adjust by, if
                        // off by a meter
                        new PIDController(2, 0, 0),
                        new PIDController(2, 0, 0),
                        new ProfiledPIDController(
                                // max velocity of 1 rotation/s
                                2,
                                0,
                                0,
                                // max velocity and max acceleration TODO check these values
                                new TrapezoidProfile.Constraints(drive.getMaxVelocity(), 4)));
        holonomicDriveController.getThetaController().enableContinuousInput(-Math.PI, Math.PI);
        // sets 5cm and 5 degree precision
        holonomicDriveController.setTolerance(new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(5)));

        return Commands.run(
                        () -> {

                            // Get goal pose for robot
                            Pose2d goalPose = calculateRobotTargetPose(drive, location);

                            double distance =
                                    drive.getPose()
                                            .getTranslation()
                                            .getDistance(goalPose.getTranslation());
                            double rotationError =
                                    Math.abs(
                                            drive.getPose().getRotation().getDegrees()
                                                    - goalPose.getRotation().getDegrees());

                            // only run if not on target
                            if (distance > Units.inchesToMeters(.5) || rotationError > 1.0) {
                                // get speeds to move to goal pose
                                ChassisSpeeds speeds =
                                        holonomicDriveController.calculate(
                                                drive.getPose(),
                                                goalPose,
                                                0,
                                                goalPose.getRotation());

                                // go to goal pose
                                // TODO figure out why this isn't field relative?
                                drive.runVelocity(speeds);
                            }
                        },
                        drive)
                .beforeStarting(
                        () ->
                                holonomicDriveController
                                        .getThetaController()
                                        .reset(drive.getRotation().getRadians()));
    }

    /**
     * Runs both rough and precise auto align to target code
     *
     * @param drive subsystem
     * @return Commands.repeatingSequence
     */
    public static Command autoAlignToLocation(Drive drive, autoAlignLocations location) {
        return Commands.repeatingSequence(
                        new ConditionalCommand(
                                // goalPose > .5 m away
                                DriveCommands.roughAlignToTarget(drive, location),
                                // goalPose <= .5 m away
                                DriveCommands.preciseAlignToTarget(drive, location),
                                () -> {
                                    // Calculate the distance between the robot and
                                    // the target.
                                    Pose2d currentPose = drive.getPose(); // Get current robot
                                    // pose
                                    Pose2d targetPose =
                                            calculateRobotTargetPose(drive, location); // Target
                                    // pose
                                    double distance =
                                            currentPose
                                                    .getTranslation()
                                                    .getDistance(targetPose.getTranslation());

                                    // true if distance > threshold distance (m)
                                    return distance > 2.5;
                                }))
                .until(drive::isAlignedToLocation);
    }

    /**
     * Align to human player station
     *
     * @param drive subsystem
     * @param xSupplier left joystick x value
     * @param ySupplier left joystick y value
     * @return command
     */
    public static Command autoAlignToHumanPlayerStation(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        ProfiledPIDController angleController =
                new ProfiledPIDController(
                        ANGLE_KP,
                        0.0,
                        ANGLE_KD,
                        new TrapezoidProfile.Constraints(drive.getMaxVelocity(), 4));

        // Enable continuous input
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.run(
                        () -> {
                            Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(
                                            xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            Pose2d hps = drive.getNearestHumanPlayerStation();

                            double omega =
                                    angleController.calculate(
                                            drive.getRotation().getRadians(),
                                            hps.getRotation().getRadians());

                            ChassisSpeeds speeds =
                                    new ChassisSpeeds(
                                            linearVelocity.getX()
                                                    * drive.getMaxLinearSpeedMetersPerSec(),
                                            linearVelocity.getY()
                                                    * drive.getMaxLinearSpeedMetersPerSec(),
                                            omega * drive.getMaxAngularSpeedRadPerSec());

                            // See if rotation should be flipped, red = flipped, blue =
                            // normal
                            boolean isFlipped =
                                    DriverStation.getAlliance().isPresent()
                                            && DriverStation.getAlliance().get() == Alliance.Red;

                            // Run the velocity on the drive
                            drive.runVelocity(
                                    ChassisSpeeds.fromFieldRelativeSpeeds(
                                            speeds,
                                            isFlipped
                                                    ? drive.getRotation()
                                                            .plus(new Rotation2d(Math.PI))
                                                    : drive.getRotation()));
                        })
                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }

    public static Command driveForward(Drive drive, double distanceInches) {
        double distanceMeters = Units.inchesToMeters(distanceInches);
        // Supplier to get the current pose each time it's needed
        Supplier<Pose2d> currentPoseSupplier = drive::getPose;

        return new Command() {
            // These will be initialized in initialize()
            private Pose2d initialPose;
            private double targetX;
            private double targetY;

            // Initialize the PID controllers for both X and Y axes
            private final PIDController xController =
                    new PIDController(2.0, 0.0, 0.1); // Tune as needed
            private final PIDController yController =
                    new PIDController(2.0, 0.0, 0.1); // Tune as needed

            @Override
            public void initialize() {
                // Get the robot's initial position and heading when the command starts
                initialPose = currentPoseSupplier.get();
                double initialHeading = initialPose.getRotation().getRadians();

                // Calculate the target position, relative to the initial position and heading
                targetX = initialPose.getX() + (Math.cos(initialHeading) * distanceMeters);
                targetY = initialPose.getY() + (Math.sin(initialHeading) * distanceMeters);

                // Reset PID controllers
                xController.setTolerance(0.02); // 2cm tolerance for X axis
                yController.setTolerance(0.02); // 2cm tolerance for Y axis
                xController.reset();
                yController.reset();
            }

            @Override
            public void execute() {
                Pose2d currentPose = currentPoseSupplier.get();

                // Calculate the current position errors (distance to target)
                double xError = targetX - currentPose.getX();
                double yError = targetY - currentPose.getY();

                // Calculate the speed along X and Y using the PID controllers
                double xSpeed = xController.calculate(currentPose.getX(), targetX);
                double ySpeed = yController.calculate(currentPose.getY(), targetY);

                // Convert field-centric speeds to robot-centric speeds
                // This depends on your drivetrain's convention for directions
                double forwardSpeed =
                        xSpeed * Math.cos(currentPose.getRotation().getRadians())
                                + ySpeed * Math.sin(currentPose.getRotation().getRadians());
                double sidewaysSpeed =
                        -xSpeed * Math.sin(currentPose.getRotation().getRadians())
                                + ySpeed * Math.cos(currentPose.getRotation().getRadians());

                // No rotation for this command
                double omegaSpeed = 0.0;

                // Command the robot to move using the calculated robot-centric speeds
                drive.runVelocity(new ChassisSpeeds(forwardSpeed, sidewaysSpeed, omegaSpeed));
            }

            @Override
            public boolean isFinished() {
                // Check if both X and Y axes are within tolerance
                Pose2d currentPose = currentPoseSupplier.get();
                return xController.atSetpoint() && yController.atSetpoint();
            }

            @Override
            public void end(boolean interrupted) {
                // Stop movement once the goal is reached or if interrupted
                drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
            }
        };
    }

    public static Command rotateByAngle(Drive drive, double angleToRotateDegrees) {
        double angleToRotateRadians = Units.degreesToRadians(angleToRotateDegrees);
        // Supplier to get the current pose each time it's needed
        Supplier<Pose2d> currentPoseSupplier = drive::getPose;

        return new Command() {
            // Initialize the PID controller for rotation
            private final PIDController rotationController =
                    new PIDController(4.0, 0.0, 0.3); // Tune as needed
            private double targetAngle;

            @Override
            public void initialize() {
                // Get the robot's current heading when command starts
                Pose2d initialPose = currentPoseSupplier.get();
                double initialHeading = initialPose.getRotation().getRadians();

                // Calculate the target angle by adding the rotation amount to the current heading
                targetAngle = initialHeading + angleToRotateRadians;

                // Normalize the angle to be between -π and π
                targetAngle = Math.atan2(Math.sin(targetAngle), Math.cos(targetAngle));

                // Configure the rotation controller
                rotationController.setTolerance(0.02); // ~1 degree tolerance

                // Enable continuous input for angle control (handles wraparound from -π to π or 0
                // to 2π)
                rotationController.enableContinuousInput(-Math.PI, Math.PI);

                rotationController.reset();
            }

            @Override
            public void execute() {
                Pose2d currentPose = currentPoseSupplier.get();
                double currentAngle = currentPose.getRotation().getRadians();

                // Calculate the rotation speed using the PID controller
                double omegaSpeed = rotationController.calculate(currentAngle, targetAngle);

                // Limit the rotation speed if necessary
                double maxRotationSpeed = 1.5; // Adjust as needed
                omegaSpeed = Math.min(Math.max(omegaSpeed, -maxRotationSpeed), maxRotationSpeed);

                // Command the robot to rotate while staying in place
                drive.runVelocity(new ChassisSpeeds(0.0, 0.0, omegaSpeed));
            }

            @Override
            public boolean isFinished() {
                // Check if the rotation is within tolerance
                return rotationController.atSetpoint();
            }

            @Override
            public void end(boolean interrupted) {
                // Stop all movement
                drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
            }
        };
    }
}
