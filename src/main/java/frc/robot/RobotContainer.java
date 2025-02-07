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

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private SwerveDriveSimulation driveSimulation = null;

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive =
                        new Drive(
                                new GyroIOPigeon2(),
                                new ModuleIOSpark(0),
                                new ModuleIOSpark(1),
                                new ModuleIOSpark(2),
                                new ModuleIOSpark(3),
                                (pose) -> {},
                                this);
                break;

            case SIM:
                // create a maple-sim swerve drive simulation instance
                this.driveSimulation =
                        new SwerveDriveSimulation(
                                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                // add the simulated drivetrain to the simulation field
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                // Sim robot, instantiate physics sim IO implementations
                drive =
                        new Drive(
                                new GyroIOSim(driveSimulation.getGyroSimulation()),
                                new ModuleIOSim(driveSimulation.getModules()[0]),
                                new ModuleIOSim(driveSimulation.getModules()[1]),
                                new ModuleIOSim(driveSimulation.getModules()[2]),
                                new ModuleIOSim(driveSimulation.getModules()[3]),
                                driveSimulation::setSimulationWorldPose,
                                this);

                // TODO: Vision SIM
                //        vision = new Vision(
                //                drive,
                //                new VisionIOPhotonVisionSim(
                //                        camera0Name, robotToCamera0,
                // driveSimulation::getSimulatedDriveTrainPose),
                //                new VisionIOPhotonVisionSim(
                //                        camera1Name, robotToCamera1,
                // driveSimulation::getSimulatedDriveTrainPose));
                break;

            default:
                // Replayed robot, disable IO implementations
                drive =
                        new Drive(
                                new GyroIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                (pose) -> {},
                                this);
                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization",
                DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization",
                DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
        sendDataToSmartDashboard();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX()));

        // DRIVER CONTROLLER
        // Lock to 0° when A button is held
        driverController
                .a()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                drive,
                                () -> -driverController.getLeftY(),
                                () -> -driverController.getLeftX(),
                                Rotation2d::new));

        // Switch to X pattern when X button is pressed
        driverController.x().onTrue(runOnce(drive::stopWithX, drive));

        // Align to the closest reef
        driverController
                .b()
                .onTrue(runOnce(drive::setTargetReefToClosest, drive))
                .whileTrue(
                        Commands.repeatingSequence(
                                        new ConditionalCommand(
                                                // goalPose > 1m away
                                                DriveCommands.roughAlignToTarget(drive),
                                                // goalPose <= 1m away
                                                DriveCommands.preciseAlignToTarget(drive),
                                                () -> {
                                                    // Calculate the distance between the robot and
                                                    // the target.
                                                    Pose2d currentPose =
                                                            drive.getPose(); // Get current robot
                                                    // pose
                                                    Pose2d targetPose =
                                                            drive.getTargetReefPose(); // Target
                                                    // pose
                                                    double distance =
                                                            currentPose
                                                                    .getTranslation()
                                                                    .getDistance(
                                                                            targetPose
                                                                                    .getTranslation());

                                                    // true if distance > threshold distance (m)
                                                    return distance > 1;
                                                }))
                                .until(
                                        () -> {
                                            // Overall condition to stop this command (robot
                                            // must be at goal pose)
                                            Pose2d currentPose = drive.getPose();
                                            Pose2d targetPose = drive.getTargetReefPose();
                                            // Calculate distance and rotation
                                            double distance =
                                                    currentPose
                                                            .getTranslation()
                                                            .getDistance(
                                                                    targetPose.getTranslation());
                                            double rotationError =
                                                    Math.abs(
                                                            currentPose.getRotation().getDegrees()
                                                                    - targetPose
                                                                            .getRotation()
                                                                            .getDegrees());

                                            // Stop when BOTH distance and orientation are
                                            // within the thresholds
                                            return distance < 0.05
                                                    && rotationError < 2.0; // <5 cm and < 5 degrees
                                        }));

        driverController
                .leftBumper()
                .onTrue(runOnce(() -> drive.setTargetReef(drive.getTargetReef().ordinal() - 1)));
        driverController
                .rightBumper()
                .onTrue(runOnce(() -> drive.setTargetReef(drive.getTargetReef().ordinal() + 1)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    /** Sets the robot to a default position and reset's the simulation field. */
    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return;
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void displaySimFieldToAdvantageScope() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        Logger.recordOutput(
                "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }

    public SwerveDriveSimulation getDriveSimulation() {
        return driveSimulation;
    }

    public void sendDataToSmartDashboard() {
        drive.updateDashboardReefVisualization(drive.getTargetReef().ordinal());
        SmartDashboard.putData(
                "Override",
                builder -> {
                    builder.setSmartDashboardType("Boolean");
                    builder.addBooleanProperty(
                            "Override Reef AA",
                            // Getter to read the current value
                            () -> drive.overrideReefAutoAlign,
                            // Setter to update the value
                            val -> drive.overrideReefAutoAlign = val);
                });
        SmartDashboard.putData(
                "Swerve Drive",
                builder -> {
                    builder.setSmartDashboardType("SwerveDrive");

                    builder.addDoubleProperty(
                            "Front Left Angle",
                            () -> drive.getModule(0).getAngle().getRadians(),
                            null);
                    builder.addDoubleProperty(
                            "Front Left Velocity",
                            () -> drive.getModule(0).getVelocityMetersPerSec(),
                            null);

                    builder.addDoubleProperty(
                            "Front Right Angle",
                            () -> drive.getModule(1).getAngle().getRadians(),
                            null);
                    builder.addDoubleProperty(
                            "Front Right Velocity",
                            () -> drive.getModule(1).getVelocityMetersPerSec(),
                            null);

                    builder.addDoubleProperty(
                            "Back Left Angle",
                            () -> drive.getModule(2).getAngle().getRadians(),
                            null);
                    builder.addDoubleProperty(
                            "Back Left Velocity",
                            () -> drive.getModule(2).getVelocityMetersPerSec(),
                            null);

                    builder.addDoubleProperty(
                            "Back Right Angle",
                            () -> drive.getModule(3).getAngle().getRadians(),
                            null);
                    builder.addDoubleProperty(
                            "Back Right Velocity",
                            () -> drive.getModule(3).getVelocityMetersPerSec(),
                            null);

                    builder.addDoubleProperty(
                            "Robot Angle", () -> drive.getPose().getRotation().getRadians(), null);
                });
    }
}
