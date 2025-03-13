package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static class AlignmentConstants {
        public static List<Pose2d> leftREDReefList = List.of(
            new Pose2d(), //
            new Pose2d(),
            new Pose2d(),
            new Pose2d(),
            new Pose2d(),
            new Pose2d()
        );
        public static List<Pose2d> leftBLUEReefList = List.of(
            new Pose2d(), //
            new Pose2d(),
            new Pose2d(),
            new Pose2d(),
            new Pose2d(),
            new Pose2d()
        );
        public static List<Pose2d> rightREDReefList = List.of(
            new Pose2d(), //
            new Pose2d(),
            new Pose2d(),
            new Pose2d(),
            new Pose2d(),
            new Pose2d()
        );
        public static List<Pose2d> rightBLUEReefList = List.of(
            new Pose2d(), //
            new Pose2d(),
            new Pose2d(),
            new Pose2d(),
            new Pose2d(),
            new Pose2d()
        );
        public static final Map<Integer, Double> left_aprilTagOffsets = new HashMap<>();

            static {
            left_aprilTagOffsets.put(6, Units.inchesToMeters(6.488));
            left_aprilTagOffsets.put(7, Units.inchesToMeters(6.488));
            left_aprilTagOffsets.put(8, Units.inchesToMeters(6.488));
            left_aprilTagOffsets.put(9, Units.inchesToMeters(6.488));
            left_aprilTagOffsets.put(10, Units.inchesToMeters(6.488));
            left_aprilTagOffsets.put(11, Units.inchesToMeters(6.488));
            left_aprilTagOffsets.put(17, Units.inchesToMeters(6.488));
            left_aprilTagOffsets.put(18, Units.inchesToMeters(6.488));
            left_aprilTagOffsets.put(19, Units.inchesToMeters(6.488));
            left_aprilTagOffsets.put(20, Units.inchesToMeters(6.488));
            left_aprilTagOffsets.put(21, Units.inchesToMeters(7.988));
            left_aprilTagOffsets.put(22, Units.inchesToMeters(6.488));
            }

        public static final Map<Integer, Double> right_aprilTagOffsets = new HashMap<>();

            static {
            right_aprilTagOffsets.put(6, Units.inchesToMeters(-6.488));
            right_aprilTagOffsets.put(7, Units.inchesToMeters(-6.488));
            right_aprilTagOffsets.put(8, Units.inchesToMeters(-6.488));
            right_aprilTagOffsets.put(9, Units.inchesToMeters(-6.488));
            right_aprilTagOffsets.put(10, Units.inchesToMeters(-6.488));
            right_aprilTagOffsets.put(11, Units.inchesToMeters(-6.488));
            right_aprilTagOffsets.put(17, Units.inchesToMeters(-6.488));
            right_aprilTagOffsets.put(18, Units.inchesToMeters(-6.488));
            right_aprilTagOffsets.put(19, Units.inchesToMeters(-6.488));
            right_aprilTagOffsets.put(20, Units.inchesToMeters(-6.488));
            right_aprilTagOffsets.put(21, Units.inchesToMeters(-6.488));
            right_aprilTagOffsets.put(22, Units.inchesToMeters(-6.488));
            }
        public static final Map<Integer, Double> aprilTagAngles = new HashMap<>();

            static {
            aprilTagAngles.put(6, 120.0);
            aprilTagAngles.put(7, 180.0);
            aprilTagAngles.put(8, -120.0);
            aprilTagAngles.put(9, -60.0);
            aprilTagAngles.put(10, 0.0);
            aprilTagAngles.put(11, 60.0);
            aprilTagAngles.put(17, 60.0);
            aprilTagAngles.put(18, 0.0);
            aprilTagAngles.put(19, -60.0);
            aprilTagAngles.put(20, -120.0);
            aprilTagAngles.put(21, 180.0);
            aprilTagAngles.put(22, 120.0);
            }
    }
}
