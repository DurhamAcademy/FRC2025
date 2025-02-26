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

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public class DriveConstants {
    // update max speed meters per sec
    public static final double maxSpeedMetersPerSec = 5.05;
    public static final double odometryFrequency = 100.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(22.75);
    public static final double wheelBase = Units.inchesToMeters(22.75);
    public static final double robotWidth = Units.inchesToMeters(28.5);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations =
            new Translation2d[] {
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
            };

    // Zeroed rotation values for each module
    // 2024 robot
    //    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(1.284); // 0
    //    public static final Rotation2d frontRightZeroRotation = new Rotation2d(-1.901); // 1
    //    public static final Rotation2d backLeftZeroRotation = new Rotation2d(1.460); // 2
    //    public static final Rotation2d backRightZeroRotation = new Rotation2d(-2.835); // 3
    // 2025 robot
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(.296); // 0
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(1.763); // 1
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.650); // 2
    public static final Rotation2d backRightZeroRotation = new Rotation2d(-1.806); // 3

    // Device CAN IDs
    // 2024 and 2025 robot
    public static final int pigeonCanId = 20;

    public static final int frontLeftDriveCanId = 1;
    public static final int backLeftDriveCanId = 3;
    public static final int frontRightDriveCanId = 5;
    public static final int backRightDriveCanId = 7;

    public static final int frontLeftTurnCanId = 2;
    public static final int backLeftTurnCanId = 4;
    public static final int frontRightTurnCanId = 6;
    public static final int backRightTurnCanId = 8;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 60;
    public static final DCMotor driveGearbox = DCMotor.getNEO(1);
    // 2024 robot
    // public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
    //  public static final double driveMotorReduction = 6.75; // MK4i L2s
    // 2025 robot
    public static final double wheelRadiusMeters = Units.inchesToMeters(2.0);
    public static final double driveMotorReduction = 6.12; // MK4i L3s

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor =
            2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor =
            (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    public static final double driveKp = 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.1;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    // 2024 + 2025 robot
    public static final boolean turnInverted = true;
    public static final int turnMotorCurrentLimit = 30;
    public static final double turnMotorReduction = 150.0 / 7.0; // steering gear ratio of all MK4is
    public static final DCMotor turnGearbox = DCMotor.getNEO(1);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor =
            2 * Math.PI / (turnMotorReduction); // Rotations -> Radians
    public static final double turnEncoderVelocityFactor =
            (2 * Math.PI) / 60.0 / (turnMotorReduction); // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double turnKp = 2.0;
    public static final double turnKd = 0.0;
    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;

    // Tipping threshold angle
    public static final double tippingThresholdDegrees = 8.0;

    // PathPlanner configuration
    public static final double robotMassKg = 74.088;
    public static final double robotMOI = 6.883;
    public static final double wheelCOF = 1.2;
    public static final RobotConfig ppConfig =
            new RobotConfig(
                    robotMassKg,
                    robotMOI,
                    new ModuleConfig(
                            wheelRadiusMeters,
                            Drive.maxUsableSpeedMetersPerSec,
                            wheelCOF,
                            driveGearbox.withReduction(driveMotorReduction),
                            driveMotorCurrentLimit,
                            1),
                    moduleTranslations);

    public static final DriveTrainSimulationConfig mapleSimConfig =
            DriveTrainSimulationConfig.Default()
                    .withCustomModuleTranslations(moduleTranslations)
                    .withRobotMass(Kilogram.of(robotMassKg))
                    .withGyro(COTS.ofPigeon2())
                    // L3 gear ratio for 2025 robot
                    .withSwerveModule(COTS.ofMark4i(driveGearbox, turnGearbox, wheelCOF, 3));
}
