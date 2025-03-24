package frc.robot;

import edu.wpi.first.math.Vector;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.opencv.core.Point;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static boolean enableTunableValues = false;

    public static class LightsConstants {
        public static int port = 2;
        public static int length = 40;

        public static enum LightsType {
            ENDGAME,
            CLIMB,
            SHOOTING,
            INTAKE,
            IDLE,
            DISABLED
        }

        public static class Colors {
            public static int[] RED = new int[] { 255, 0, 0 };
            public static int[] GREEN = new int[] { 0, 255, 0 };
            public static int[] BLUE = new int[] { 0, 0, 255 };
            public static int[] GOLD = new int[] { 175, 184, 6 };
            public static int[] MAGENTA = new int[] { 255, 0, 255 };
            public static int[] BRIGHT = new int[] { 234, 255, 48 };
        }
    }

    public static final class VisionConstants {
        /* camera stuff */
        public static double bumperToBumper = 33.25; // should be chassis length TODO: TUNE PID THIS IS A BANDAID
        public static Transform3d kRobotToCam = new Transform3d(
                new Translation3d(Units.inchesToMeters(19.41), 0, Units.inchesToMeters(6.6)),
                new Rotation3d(0, Units.degreesToRadians(23), 0)); // TODO: edit yaw
        public static String cameraName = "orangepi";
        public static String camera2Name = "orangepi2";
        /* standard deviations for vision calculations */
        public static Vector<N3> kSingleTagStdDevs = VecBuilder.fill(2, 2, 2);
        public static Vector<N3> kMultiTagStdDevs = VecBuilder.fill(1, 1, 1);
        public static Vector<N3> odoStdDEvs = VecBuilder.fill(.2, .2, .2);
        public static double odometryUpdateFrequency = 250;
        /* pose of reefs for alignment */
        public static Optional<Pose3d> blueReefPose;
        public static Optional<Pose3d> redReefPose;
        public static double leftOffsetInches = 6;
        public static double rightOffsetInches = 5.5;

    }

    public static final class AutoConstants {
        public static double kMaxSpeedMetersPerSecond = 3;
        public static double kMaxAccelerationMetersPerSecondSquared = 6.1;
        public static double kMaxAngularSpeedRadiansPerSecond = Units
                .rotationsPerMinuteToRadiansPerSecond(0.75 * (1 / 60));
        public static double kMaxAngularSpeedRadiansPerSecondSquared = Units.degreesToRadians(862);

        public static double robotMass = 44;
        public static double robotMOI = 5.469;
        public static double bumperWidth = .9;
        public static double wheelRadius = 0.048;
        public static double driveGearing = 6.746;
        public static double maxDriveSpeed = 5.45;
        public static double wheelFrictionCoefficient = 1.5;
        public static double driveCurrentLimit = 60;
        public static double moduleOffset = 0.349;
    }

    public static final class WristConstants {
        public static int wristMotorID = 51;
        public static int absoluteEncoderPort = 3;
        public static double[] wristPID = { .625, 0, 0 };
        public static int maxVelocity = 9;
        public static int maxAccel = 0;
        public static double[] wristFF = { 0, 0, 0 };
        public static double wristGearRatio = 0;
        public static Rotation2d wristOffset = new Rotation2d(Units.degreesToRadians(94.464372)); // -59.7-18.8
        public static final Rotation2d maxVelocityPerSecond = Rotation2d.fromDegrees(460); // was 500
        public static final Rotation2d maxAcceleration = Rotation2d.fromDegrees(460);

        public static final Rotation2d wristMax = Rotation2d.fromDegrees(86);
        public static final Rotation2d wristMin = Rotation2d.fromDegrees(-76);
        public static final Rotation2d tolernace = Rotation2d.fromDegrees(0.5);
    }

    public static final class IntakeConstants {
        public static int leaderID = 61; // was the intake
        public static int followerID = 62; // was the deflector
    }

    public static final class ClimberConstants {
        public static int climberMotorID = 25;
        public static double climberMax = 315;
        public static double climberMin = -97.7; // was -90
    }

    public static final class ElevatorConstants {
        public static double min = .5;
        public static double max = 18;
        public static double homingCurrentThreshold = 20;
        public static double selfHomeSpeedVoltage = 15;
        public static int desiredMin;
        public static int desiredMax;
        public static int leftElevatorMotorID = 41;
        public static int rightElevatorMotorID = 42;
        public static double[] elevatorPID = new double[] { 0.085, 0, 0 }; // p is a little aggressive
        public static double[] elevatorSGV = new double[] { 0, 0.015, 0.0009, 0 }; // g needs testing
        public static double maxVelocity = 150; // in mps
        public static double maxAccel = 150; // in mps/s
        public static double elevatorTolerance = .3;

        public static enum ElevateMode {
            UP,
            DOWN,
            L1,
            L2,
            L2AR,
            L3,
            L4,
            HP,
            OFF,
            MANUAL,
            TEST,
            HOMING,
            RESET
        }

        public static double LevelOneSetpoint = 1;
        public static double LevelTwoSetpoint = 1;
        public static double LevelTwoAlgaeSetpoint = 0.5;
        public static double LevelThreeSetpoint = 5;
        public static double LevelFourSetpoint = 18.7;
        public static double HPsetpoint = -01;
        public static double test = 16;

    }

    public static final class ArmConstants {
        // Motor and encoder IDs
        public static final int leaderID = 31;
        public static final int followerID = 32;
        public static final int encoderID = 0;

        public static final Rotation2d offset = Rotation2d.fromDegrees(-18); // **WAS 88-90//How far away the arm is
                                                                             // (at
                                                                             // rest) from 0 degrees
        public static final Rotation2d max = Rotation2d.fromDegrees(90); // Max. value for the arm's angle
        public static final Rotation2d min = Rotation2d.fromDegrees(-10); // Min. value for the arm's angle
        public static final Rotation2d tolernace = Rotation2d.fromDegrees(1); // How much the arm's exact angle can be
                                                                              // off by

        public static final Rotation2d maxVelocityPerSecond = Rotation2d.fromDegrees(480); // was 400
        public static final Rotation2d maxAcceleration = Rotation2d.fromDegrees(550); // was 400

        public static double[] armSGV = new double[] { 0.0, 0.001, 0.0 }; // Static, Gravity, and Velocity gains
        public static double[] armPID = new double[] { 1.47, 0, 0f }; // Arm PID values

        // Arm's polynomial regression (useed to predict distances from speaker)
        // public static final PolynomialRegression
        // armAngleInterpolationPolynominalRegression = new PolynomialRegression(
        // Arrays.asList(
        // new Point(1.43, Units.degreesToRadians(47f)),
        // new Point(3, Units.degreesToRadians(34f)),
        // new Point(4, Units.degreesToRadians(29f)),
        // new Point(5, Units.degreesToRadians(24.5f))),
        // 2);
    }
}
