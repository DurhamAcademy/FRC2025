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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static class LocationConstants {

        /**
         * HashMap containing locations of individual reefs.
         *
         * <p>Key = Reef Constant.
         *
         * <p>Value = Array of Pose2D's where 0th index is blue and 1st index is red
         */
        public static final Map<ReefConstants, Pose2d[]> ReefLocations =
                new HashMap<>() {
                    {
                        // centers of the hexagons
                        double blueY = Units.inchesToMeters(158.5);
                        double redY = Units.inchesToMeters(blueY);
                        double blueX = Units.inchesToMeters(176.75);
                        double redX = Units.inchesToMeters(690.875 - blueX);

                        // for the 1, 2, 7, and 8 positions where the edge is parallel to the
                        // alliance walls
                        double changeXLarge = Units.inchesToMeters(32.75);
                        double changeYSmall = Units.inchesToMeters(6.5);

                        // for 3, 6, 9, and 12 (the parts that are closer to the parallel sides, but
                        // still on
                        // the slanted sides)
                        double changeXMedium = Units.inchesToMeters(22.004165);
                        double changeYMedium = Units.inchesToMeters(25.112332);

                        // for 4, 5, 10, and 11 (the ones in the middle)
                        double changeXSmall = Units.inchesToMeters(10.745835);
                        double changeYLarge = Units.inchesToMeters(31.612332);

                        put(
                                ReefConstants.ONE,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX - changeXLarge,
                                            blueY + changeYSmall,
                                            new Rotation2d()),
                                    new Pose2d(
                                            redX + changeXLarge,
                                            redY - changeYSmall,
                                            new Rotation2d())
                                });
                        put(
                                ReefConstants.TWO,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX - changeXLarge,
                                            blueY - changeYSmall,
                                            new Rotation2d()),
                                    new Pose2d(
                                            redX + changeXLarge,
                                            redY + changeYSmall,
                                            new Rotation2d())
                                });
                        put(
                                ReefConstants.THREE,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX - changeXMedium,
                                            blueY - changeYMedium,
                                            new Rotation2d()),
                                    new Pose2d(
                                            redX + changeXMedium,
                                            redY + changeYMedium,
                                            new Rotation2d())
                                });
                        put(
                                ReefConstants.FOUR,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX - changeXSmall,
                                            blueY - changeYLarge,
                                            new Rotation2d()),
                                    new Pose2d(
                                            redX + changeXSmall,
                                            redY + changeYLarge,
                                            new Rotation2d())
                                });
                        put(
                                ReefConstants.FIVE,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX + changeXSmall,
                                            blueY - changeYLarge,
                                            new Rotation2d()),
                                    new Pose2d(
                                            redX - changeXSmall,
                                            redY + changeYLarge,
                                            new Rotation2d())
                                });
                        put(
                                ReefConstants.SIX,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX + changeXMedium,
                                            blueY - changeYMedium,
                                            new Rotation2d()),
                                    new Pose2d(
                                            redX - changeXMedium,
                                            redY + changeYSmall,
                                            new Rotation2d())
                                });
                        put(
                                ReefConstants.SEVEN,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX + changeXLarge,
                                            blueY - changeYSmall,
                                            new Rotation2d()),
                                    new Pose2d(
                                            redX - changeXLarge,
                                            redY + changeYSmall,
                                            new Rotation2d())
                                });
                        put(
                                ReefConstants.EIGHT,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX + changeXLarge,
                                            blueY + changeYSmall,
                                            new Rotation2d()),
                                    new Pose2d(
                                            redX - changeXLarge,
                                            redY - changeYSmall,
                                            new Rotation2d())
                                });
                        put(
                                ReefConstants.NINE,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX + changeXMedium,
                                            blueY + changeYMedium,
                                            new Rotation2d()),
                                    new Pose2d(
                                            redX - changeXMedium,
                                            redY - changeYMedium,
                                            new Rotation2d())
                                });
                        put(
                                ReefConstants.TEN,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX + changeXSmall,
                                            blueY + changeYLarge,
                                            new Rotation2d()),
                                    new Pose2d(
                                            redX - changeXSmall,
                                            redY - changeYLarge,
                                            new Rotation2d())
                                });
                        put(
                                ReefConstants.ELEVEN,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX - changeXSmall,
                                            blueY + changeYLarge,
                                            new Rotation2d()),
                                    new Pose2d(
                                            redX + changeXSmall,
                                            redY - changeYLarge,
                                            new Rotation2d())
                                });
                        put(
                                ReefConstants.TWELVE,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX - changeXMedium,
                                            blueY + changeYMedium,
                                            new Rotation2d()),
                                    new Pose2d(
                                            redX + changeXMedium,
                                            redY - changeYMedium,
                                            new Rotation2d())
                                });
                    }
                };

        public static final List<ReefConstants> AllReefLocations =
                List.of(
                        ReefConstants.ONE,
                        ReefConstants.TWO,
                        ReefConstants.THREE,
                        ReefConstants.FOUR,
                        ReefConstants.FIVE,
                        ReefConstants.SIX,
                        ReefConstants.SEVEN,
                        ReefConstants.EIGHT,
                        ReefConstants.NINE,
                        ReefConstants.TEN,
                        ReefConstants.ELEVEN,
                        ReefConstants.TWELVE);

        public static List<Pose2d> PosesOfAllReefLocations(int color) {
            List<Pose2d> allPoses = new ArrayList<>();
            for (int i = 0; i < AllReefLocations.size(); i++) {
                allPoses.add(ReefLocations.get(AllReefLocations.get(i))[color]);
            }
            return allPoses;
        }
    }

    public enum ReefConstants {
        ONE,
        TWO,
        THREE,
        FOUR,
        FIVE,
        SIX,
        SEVEN,
        EIGHT,
        NINE,
        TEN,
        ELEVEN,
        TWELVE
    }
}