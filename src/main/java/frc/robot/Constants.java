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
import edu.wpi.first.wpilibj.DriverStation;
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

    /**
     * Get number associated with DriverStation alliance
     *
     * @param alliance DriverStation.getAlliance(), either Alliance.Red or Alliance.Blue
     * @return 1 for red, 0 for blue
     */
    public static int getAllianceColor(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Red ? 1 : 0;
    }

    public static class LocationConstants {
        public static enum AprilTagLocations {
            // List of locations from
            // https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf
            ONE(
                    new Pose2d(
                            Units.inchesToMeters(657.37),
                            Units.inchesToMeters(25.80),
                            new Rotation2d())),
            TWO(
                    new Pose2d(
                            Units.inchesToMeters(657.37),
                            Units.inchesToMeters(291.20),
                            new Rotation2d())),
            THREE(
                    new Pose2d(
                            Units.inchesToMeters(455.15),
                            Units.inchesToMeters(317.15),
                            new Rotation2d())),
            FOUR(
                    new Pose2d(
                            Units.inchesToMeters(365.20),
                            Units.inchesToMeters(241.64),
                            new Rotation2d())),
            FIVE(
                    new Pose2d(
                            Units.inchesToMeters(365.20),
                            Units.inchesToMeters(75.39),
                            new Rotation2d())),
            SIX(
                    new Pose2d(
                            Units.inchesToMeters(530.49),
                            Units.inchesToMeters(130.17),
                            new Rotation2d())),
            SEVEN(
                    new Pose2d(
                            Units.inchesToMeters(546.87),
                            Units.inchesToMeters(158.50),
                            new Rotation2d())),
            EIGHT(
                    new Pose2d(
                            Units.inchesToMeters(530.49),
                            Units.inchesToMeters(186.83),
                            new Rotation2d())),
            NINE(
                    new Pose2d(
                            Units.inchesToMeters(497.77),
                            Units.inchesToMeters(186.83),
                            new Rotation2d())),
            TEN(
                    new Pose2d(
                            Units.inchesToMeters(481.39),
                            Units.inchesToMeters(158.50),
                            new Rotation2d())),
            ELEVEN(
                    new Pose2d(
                            Units.inchesToMeters(497.77),
                            Units.inchesToMeters(130.17),
                            new Rotation2d())),
            TWELVE(
                    new Pose2d(
                            Units.inchesToMeters(33.51),
                            Units.inchesToMeters(25.80),
                            new Rotation2d())),
            THIRTEEN(
                    new Pose2d(
                            Units.inchesToMeters(33.51),
                            Units.inchesToMeters(291.20),
                            new Rotation2d())),
            FOURTEEN(
                    new Pose2d(
                            Units.inchesToMeters(325.68),
                            Units.inchesToMeters(241.64),
                            new Rotation2d())),
            FIFTEEN(
                    new Pose2d(
                            Units.inchesToMeters(325.68),
                            Units.inchesToMeters(75.39),
                            new Rotation2d())),
            SIXTEEN(
                    new Pose2d(
                            Units.inchesToMeters(235.73),
                            Units.inchesToMeters(-0.15),
                            new Rotation2d())),
            SEVENTEEN(
                    new Pose2d(
                            Units.inchesToMeters(160.39),
                            Units.inchesToMeters(130.17),
                            new Rotation2d())),
            EIGHTEEN(
                    new Pose2d(
                            Units.inchesToMeters(144.00),
                            Units.inchesToMeters(158.50),
                            new Rotation2d())),
            NINETEEN(
                    new Pose2d(
                            Units.inchesToMeters(160.39),
                            Units.inchesToMeters(186.83),
                            new Rotation2d())),
            TWENTY(
                    new Pose2d(
                            Units.inchesToMeters(193.10),
                            Units.inchesToMeters(186.83),
                            new Rotation2d())),
            TWENTY_ONE(
                    new Pose2d(
                            Units.inchesToMeters(209.49),
                            Units.inchesToMeters(158.50),
                            new Rotation2d())),
            TWENTY_TWO(
                    new Pose2d(
                            Units.inchesToMeters(193.10),
                            Units.inchesToMeters(130.17),
                            new Rotation2d()));

            private final Pose2d value;

            AprilTagLocations(Pose2d value) {
                this.value = value;
            }

            public Pose2d getValue() {
                return value;
            }
        }

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
                        double redY = blueY;
                        double blueX = Units.inchesToMeters(176.75);
                        double redX = Units.inchesToMeters(690.875) - blueX;

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
                                ReefConstants.SEVEN,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX - changeXLarge,
                                            blueY + changeYSmall,
                                            Rotation2d.fromDegrees(180)),
                                    new Pose2d(
                                            redX + changeXLarge,
                                            redY - changeYSmall,
                                            Rotation2d.fromDegrees(0))
                                });
                        put(
                                ReefConstants.SIX,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX - changeXLarge,
                                            blueY - changeYSmall,
                                            Rotation2d.fromDegrees(180)),
                                    new Pose2d(
                                            redX + changeXLarge,
                                            redY + changeYSmall,
                                            Rotation2d.fromDegrees(0))
                                });
                        put(
                                ReefConstants.FIVE,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX - changeXMedium,
                                            blueY - changeYMedium,
                                            Rotation2d.fromDegrees(240)),
                                    new Pose2d(
                                            redX + changeXMedium,
                                            redY + changeYMedium,
                                            Rotation2d.fromDegrees(60))
                                });
                        put(
                                ReefConstants.FOUR,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX - changeXSmall,
                                            blueY - changeYLarge,
                                            Rotation2d.fromDegrees(240)),
                                    new Pose2d(
                                            redX + changeXSmall,
                                            redY + changeYLarge,
                                            Rotation2d.fromDegrees(60))
                                });
                        put(
                                ReefConstants.THREE,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX + changeXSmall,
                                            blueY - changeYLarge,
                                            Rotation2d.fromDegrees(300)),
                                    new Pose2d(
                                            redX - changeXSmall,
                                            redY + changeYLarge,
                                            Rotation2d.fromDegrees(120))
                                });
                        put(
                                ReefConstants.TWO,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX + changeXMedium,
                                            blueY - changeYMedium,
                                            Rotation2d.fromDegrees(300)),
                                    new Pose2d(
                                            redX - changeXMedium,
                                            redY + changeYMedium,
                                            Rotation2d.fromDegrees(120))
                                });
                        put(
                                ReefConstants.ONE,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX + changeXLarge,
                                            blueY - changeYSmall,
                                            Rotation2d.fromDegrees(0)),
                                    new Pose2d(
                                            redX - changeXLarge,
                                            redY + changeYSmall,
                                            Rotation2d.fromDegrees(180))
                                });
                        put(
                                ReefConstants.TWELVE,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX + changeXLarge,
                                            blueY + changeYSmall,
                                            Rotation2d.fromDegrees(0)),
                                    new Pose2d(
                                            redX - changeXLarge,
                                            redY - changeYSmall,
                                            Rotation2d.fromDegrees(180))
                                });
                        put(
                                ReefConstants.ELEVEN,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX + changeXMedium,
                                            blueY + changeYMedium,
                                            Rotation2d.fromDegrees(60)),
                                    new Pose2d(
                                            redX - changeXMedium,
                                            redY - changeYMedium,
                                            Rotation2d.fromDegrees(240))
                                });
                        put(
                                ReefConstants.TEN,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX + changeXSmall,
                                            blueY + changeYLarge,
                                            Rotation2d.fromDegrees(60)),
                                    new Pose2d(
                                            redX - changeXSmall,
                                            redY - changeYLarge,
                                            Rotation2d.fromDegrees(240))
                                });
                        put(
                                ReefConstants.NINE,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX - changeXSmall,
                                            blueY + changeYLarge,
                                            Rotation2d.fromDegrees(120)),
                                    new Pose2d(
                                            redX + changeXSmall,
                                            redY - changeYLarge,
                                            Rotation2d.fromDegrees(300))
                                });
                        put(
                                ReefConstants.EIGHT,
                                new Pose2d[] {
                                    new Pose2d(
                                            blueX - changeXMedium,
                                            blueY + changeYMedium,
                                            Rotation2d.fromDegrees(120)),
                                    new Pose2d(
                                            redX + changeXMedium,
                                            redY - changeYMedium,
                                            Rotation2d.fromDegrees(300))
                                });
                    }
                };

        // https://drive.google.com/file/d/1K155pCUQ5puJRHuw8-uzw9tF79fYn8L3/view?usp=sharing
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

        public static Rotation2d humanPlayerStationBlue = Rotation2d.fromDegrees(54);

        public static Rotation2d humanPlayerStationRed = Rotation2d.fromDegrees(234);
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
