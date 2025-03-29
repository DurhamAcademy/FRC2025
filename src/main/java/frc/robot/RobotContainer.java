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

import static frc.robot.Constants.PosesOfAllHumanPlayerStations;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorLevel;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.manipulator.*;
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
    private final Manipulator manipulator;
    private final Intake intake;
    private final Elevator elevator;
    private SwerveDriveSimulation driveSimulation = null;

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    // overrides
    private boolean invertX = true;
    private boolean invertY = true;
    private double xDirect = 1;
    private double yDirect = 1;
    public boolean overrideSafeElevator = false;

    // states
    public boolean algaeMode = true;
    public ElevatorLevel currentLevel = null;
    public ElevatorLevel queuedLevel = ElevatorLevel.ZERO;


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

                manipulator = new Manipulator(new ManipulatorIOSparkFlex());
                intake = new Intake(new IntakeIOSparkMax());
                elevator = new Elevator(new ElevatorIOSparkMax(), new WristIOSparkMax(), drive);
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
                manipulator = new Manipulator(new ManipulatorIOSim());
                intake = new Intake(new IntakeIOSim(driveSimulation));
                elevator = new Elevator(new ElevatorIOSim(), new WristIOSim(), drive);
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

                intake = new Intake(new IntakeIO() {});
                manipulator = new Manipulator(new ManipulatorIO() {});
                elevator = new Elevator(new ElevatorIO() {}, new WristIO() {}, drive);
                break;
        }

        registerNamedCommands();

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        //        autoChooser.addOption(
        //                "Drive Wheel Radius Characterization",
        //                DriveCommands.wheelRadiusCharacterization(drive));
        //        autoChooser.addOption(
        //                "Drive Simple FF Characterization",
        //                DriveCommands.feedforwardCharacterization(drive));
        //        autoChooser.addOption(
        //                "Drive SysId (Quasistatic Forward)",
        //                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        //        autoChooser.addOption(
        //                "Drive SysId (Quasistatic Reverse)",
        //                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        //        autoChooser.addOption(
        //                "Drive SysId (Dynamic Forward)",
        //                drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        //        autoChooser.addOption(
        //                "Drive SysId (Dynamic Reverse)",
        //                drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        sendDataToSmartDashboard();
        configureButtonBindings();
    }

    public void getSwerveDirection() {

        if (invertX) {
            xDirect = -1;
        } else {
            xDirect = 1;
        }
        if (invertY) {
            yDirect = -1;
        } else {
            yDirect = 1;
        }
    }

    /**
     * Method that registers all the named commands to be used by pathplanner for autos and whatnot.
     */
    private void registerNamedCommands() {
        NamedCommands.registerCommand(
                "Elevator L1",
                ElevatorCommands.setElevatorLevel(elevator, ElevatorLevel.L1)
                        .andThen(Commands.waitUntil(elevator::elevatorIsAtSetpoint))
                        .withTimeout(3));

        NamedCommands.registerCommand(
                "Elevator L2",
                ElevatorCommands.setElevatorLevel(elevator, ElevatorLevel.L2)
                        .andThen(Commands.waitUntil(elevator::elevatorIsAtSetpoint))
                        .withTimeout(3.5));

        NamedCommands.registerCommand(
                "Elevator L3",
                ElevatorCommands.setElevatorLevel(elevator, ElevatorLevel.L3)
                        .andThen(Commands.waitUntil(elevator::elevatorIsAtSetpoint))
                        .withTimeout(4));

        NamedCommands.registerCommand(
                "Elevator L4",
                ElevatorCommands.setElevatorLevel(elevator, ElevatorLevel.L4)
                        .andThen(Commands.waitUntil(elevator::elevatorIsAtSetpoint))
                        .withTimeout(4.5));

        NamedCommands.registerCommand(
                "Elevator Zero",
                ElevatorCommands.zeroElevatorForAlgae(elevator)
                        .andThen(Commands.waitUntil(elevator::isAtSetpoint))
                        .withTimeout(4.5));

        NamedCommands.registerCommand(
                "Elevator Intake",
                ElevatorCommands.setElevatorLevel(elevator, ElevatorLevel.INTAKE)
                        .andThen(Commands.waitUntil(elevator::isAtSetpoint))
                        .withTimeout(4.5));

        NamedCommands.registerCommand(
                "Eject Coral",
                ManipulatorCommands.eject(manipulator, 1)
                        .withTimeout(.75)
                        .andThen(ManipulatorCommands.eject(manipulator, 0).withTimeout(.1)));

        NamedCommands.registerCommand(
                "Pull Coral into Intake",
                IntakeCommands.pullCoralThroughIntake(intake, manipulator));
        NamedCommands.registerCommand(
                "Pull Coral into Manipulator",
                ManipulatorCommands.pullCoralIntoManipulator(manipulator));
        NamedCommands.registerCommand(
                "Manipulator Coral Ripple", ManipulatorCommands.coralIntakeRipple(manipulator));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        Trigger safeToMoveElevator =
                new Trigger(() -> !intake.getBeamBroken() || overrideSafeElevator);

        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> (yDirect * driverController.getLeftY()),
                        () -> (xDirect * driverController.getLeftX()),
                        () -> -driverController.getRightX()));

        manipulator.setDefaultCommand(
                ManipulatorCommands.manipulatorDefaultHoldCoral(manipulator, elevator)
                        .onlyIf(manipulator::beamBroken));

        // if elevator has zeroed, run tipping prevention code
        // if not, zero the elevator for the first time
        elevator.setDefaultCommand(
                ElevatorCommands.setElevatorLevel(elevator, ElevatorLevel.ZERO)
                        .onlyIf(drive::isTipping)
                        .onlyIf(safeToMoveElevator)); // assuming that the robot has been zeroed

        Command manipulatorEject = ManipulatorCommands.eject(manipulator, 1);

        Command setAlignLeft =
                Commands.runOnce(() -> drive.setTargetReefToClosest(Drive.ReefAlignSide.LEFT));

        Command setAlignRight =
                Commands.runOnce(() -> drive.setTargetReefToClosest(Drive.ReefAlignSide.RIGHT));

        Command stopManipulator = ManipulatorCommands.runManipulator(manipulator, 0);

        // DRIVER CONTROLLER BINDINGS

        // Switch to X pattern when X button is pressed
        driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Shoot coral / algae
        driverController
                .rightBumper()
                .whileTrue(
                        Commands.either(
                                ManipulatorCommands.algaeEject(manipulator, elevator),
                                ManipulatorCommands.coralEject(manipulator),
                                () -> algaeMode))
                .onFalse(stopManipulator);

    driverController
        .leftBumper()
        .and(() -> !algaeMode)
        .onTrue(
            Commands.either(
                ElevatorCommands.setElevatorLevel(elevator, queuedLevel)
                    .alongWith(Commands.runOnce(() -> currentLevel = queuedLevel)),
                ElevatorCommands.zeroElevatorForCoral(elevator)
                    .alongWith(Commands.runOnce(() -> currentLevel = ElevatorLevel.ZERO)),
                () -> queuedLevel == currentLevel));

        // Auto align
        driverController
                .leftTrigger()
                .and(driverController.rightBumper().negate())
                .onTrue(setAlignLeft)
                .whileTrue(
                        Commands.either(
                                DriveCommands.autoAlignToLocation(
                                        drive, DriveCommands.autoAlignLocations.algae),
                                DriveCommands.autoAlignToLocation(
                                        drive, DriveCommands.autoAlignLocations.reef),
                                () -> algaeMode));
        driverController
                .rightTrigger()
                .and(driverController.rightBumper().negate())
                .onTrue(setAlignRight)
                .whileTrue(
                        Commands.either(
                                DriveCommands.autoAlignToLocation(
                                        drive, DriveCommands.autoAlignLocations.processor),
                                DriveCommands.autoAlignToLocation(
                                        drive, DriveCommands.autoAlignLocations.reef),
                                () -> algaeMode));

        // OPERATOR CONTROLLER

        // algae / coral mode
        operatorController.rightBumper().onTrue(Commands.runOnce(() -> algaeMode = true)
                .alongWith(Commands.runOnce(() -> currentLevel = null))
        );
        operatorController.leftBumper().onTrue(Commands.runOnce(() -> algaeMode = false));

        // intake
        operatorController
                .leftTrigger()
                .onTrue(
                        Commands.either(
                                ManipulatorCommands.algaeIntake(manipulator),
                                IntakeCommands.fullCoralIntakeSequence(intake, manipulator),
                                () -> algaeMode));
        operatorController
                .rightTrigger()
                .onTrue(
                        ManipulatorCommands.stopManipulator(manipulator)
                                .andThen(IntakeCommands.stopIntake(intake)));

        operatorController
                .a()
                .onTrue(
                        IntakeCommands.stopIntake(intake)
                                .alongWith(ManipulatorCommands.stopManipulator(manipulator))
                                .andThen(IntakeCommands.retryStuckIntake(intake, manipulator)));

        // elevator
        operatorController
                .start()
                .onTrue(
                        Commands.either(
                                        ElevatorCommands.zeroElevatorForAlgae(elevator),
                                        ElevatorCommands.zeroElevatorForCoral(elevator),
                                        () -> algaeMode)
                                .onlyIf(safeToMoveElevator));

        operatorController
                .povLeft()
                .onTrue(
                        Commands.either(
                                ElevatorCommands.setElevatorLevel(
                                        // fixme @valen will this use of 'safetomoveelevator' work?
                                        elevator, ElevatorLevel.NET).onlyIf(safeToMoveElevator),
                                Commands.runOnce(() -> queuedLevel = ElevatorLevel.L1),
                                () -> algaeMode
                        )
                );

        operatorController
                .povDown()
                .onTrue(
                        Commands.either(
                                ElevatorCommands.setElevatorLevel(
                                        elevator, ElevatorLevel.LOWER_ALGAE_REMOVAL).onlyIf(safeToMoveElevator),
                                Commands.runOnce(() -> queuedLevel = ElevatorLevel.L2),
                                () -> algaeMode
                        )
                );


        operatorController
                .povRight()
                .onTrue(
                        Commands.either(
                                ElevatorCommands.setElevatorLevel(
                                        elevator, ElevatorLevel.PROCESSOR).onlyIf(safeToMoveElevator),
                                Commands.runOnce(() -> queuedLevel = ElevatorLevel.L3),
                                () -> algaeMode
                        )
                );

        operatorController
                .povUp()
                .onTrue(
                        Commands.either(
                                ElevatorCommands.setElevatorLevel(
                                        elevator, ElevatorLevel.UPPER_ALGAE_REMOVAL).onlyIf(safeToMoveElevator),
                                Commands.runOnce(() -> queuedLevel = ElevatorLevel.L4),
                                () -> algaeMode
                        )
                );
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
        Logger.recordOutput("MAX SPEED", SmartDashboard.getNumber("Max Speed/Max Speed", 0));
        Logger.recordOutput("X invert", SmartDashboard.getBoolean("INVERT AXES/X INVERT", false));
        Logger.recordOutput("Y invert", SmartDashboard.getBoolean("INVERT AXES/Y INVERT", false));
        Logger.recordOutput(
                "XY invert", SmartDashboard.getBoolean("INVERT AXES/X/Y INVERT", false));
        Logger.recordOutput(
                "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }

    /** For SIM only, adds a coral to the intake if the robot is at the human player station */
    // TODO: FIX INTAKE SIM
    public void intakeCoralIfAtStation() {
        if (DriverStation.getAlliance().isEmpty()) return;
        final double DISTANCE_THRESHOLD = 1.0;
        Pose2d[] HpStations =
                PosesOfAllHumanPlayerStations(
                                Constants.getAllianceColor(DriverStation.getAlliance().get()))
                        .toArray(new Pose2d[0]);
        Logger.recordOutput("Intake/HumanPlayers", HpStations);
        for (Pose2d stationPose : HpStations) {
            Pose2d robotPose = drive.getPose();
            double distance = robotPose.getTranslation().getDistance(stationPose.getTranslation());
            Logger.recordOutput("Intake/HumanPlayerDist" + stationPose.toString(), distance);
            if (distance < DISTANCE_THRESHOLD) {
                // intake.simAddCoral(robotPose);
            }
        }
    }

    public SwerveDriveSimulation getDriveSimulation() {
        return driveSimulation;
    }

    public void resetSetpoints() {
        elevator.stopElevator();
        elevator.stopWrist();
        elevator.setWristTargetAngle(elevator.getWristAngle());
        elevator.setElevatorTargetHeight(0);
        intake.stopMotors();
        manipulator.stopMotors();
    }

    public void sendDataToSmartDashboard() {
        SmartDashboard.putData(
                "Vision",
                builder -> {
                    builder.setSmartDashboardType("Boolean");
                    builder.addBooleanProperty("alignedToTarget", drive::isAlignedToLocation, null);
                });

        drive.updateTargetDashboardVisualization(drive.getTargetAlgae().ordinal());

        SmartDashboard.putData(
                "Override",
                builder -> {
                    builder.setSmartDashboardType("Boolean");
                    builder.addBooleanProperty(
                            "Reef AA",
                            // Getter to read the current value
                            () -> drive.overrideReefAutoAlign,
                            // Setter to update the value
                            val -> drive.overrideReefAutoAlign = val);
                    builder.addBooleanProperty(
                            "Anti-Tip",
                            () -> drive.overrideTipProtection,
                            val -> drive.overrideTipProtection = val);
                    builder.addBooleanProperty(
                            "Safe Elevator",
                            () -> overrideSafeElevator,
                            val -> overrideSafeElevator = val);
                });
        SmartDashboard.putData(
                "INVERT AXES",
                builder -> {
                    builder.setSmartDashboardType("boolean");
                    builder.addBooleanProperty("X INVERT", () -> invertX, val -> invertX = val);
                    builder.addBooleanProperty("Y INVERT", () -> invertY, val -> invertY = val);
                    builder.addBooleanProperty(
                            "XY INVERT",
                            () -> invertX && invertY,
                            val -> {
                                invertX = val;
                                invertY = val;
                            });
                });
        SmartDashboard.putData(
                "Elevator",
                builder -> {
                    builder.setSmartDashboardType("boolean");
                    builder.addBooleanProperty("Zero", elevator::isZeroed, null);
                    builder.addBooleanProperty("E+W Setpoint", elevator::isAtSetpoint, null);
                });
        SmartDashboard.putData(
                "MAX SPEED",
                builder -> {
                    builder.setSmartDashboardType("double");
                    builder.addDoubleProperty(
                            "Max",
                            drive::getMaxVelocity,
                            val -> Drive.currentSpeedLimitMetersPerSec = val);
                });
    }
}
