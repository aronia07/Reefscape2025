package frc.robot;

import edu.wpi.first.math.Vector;

import java.util.Arrays;
import java.util.Optional;

import org.opencv.core.Point;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static boolean enableTunableValues = true;

    public static final class VisionConstants {
        /* camera stuff */
        public static Transform3d kRobotToCam = null;
        public static String cameraName = "orangepi";
        /* standard deviations for vision calculations */
        public static Vector<N3> kSingleTagStdDevs = VecBuilder.fill(2, 2, 4);
        public static Vector<N3> kMultiTagStdDevs = VecBuilder.fill(1, 1, 1);
        public static Vector<N3> odoStdDEvs = VecBuilder.fill(.2, .2, .2);
        public static double odometryUpdateFrequency = 250;
        /* pose of reefs for alignment */
        public static Optional<Pose3d> blueReefPose;
        public static Optional<Pose3d> redReefPose;

    }

    public static final class WristConstants {
        public static int wristMotorID = 51;
        public static int absoluteEncoderPort = 3;
        public static double[] wristPID = { 0, 0, 0 };
        public static int maxVelocity = 9;
        public static int maxAccel;
        public static double[] wristFF = { 0, 0, 0 };
        public static double wristGearRatio = 0;
        public static Rotation2d wristOffset;
    }

    public static final class IntakeConstants {
        public static int intakeID = 61;
        public static int deflectorID = 62;
        public static int[] intakePID = { 0, 0, 0 };
    }

    public static final class ElevatorConstants {
        public static double min = .5;
        public static double max = 17.5;
        public static double homingCurrentThreshold = 10;
        public static double selfHomeSpeedVoltage;
        public static int desiredMin;
        public static int desiredMax;
        public static int leftElevatorMotorID = 41;
        public static int rightElevatorMotorID = 42;
        public static double[] elevatorPID = new double[] { 0.00005, 0, 0 }; // p is .08
        public static double[] elevatorSGV = new double[] { 0, .03, 0.018, 0 }; // v is .0002
        public static double maxVelocity = 100; // in mps
        public static double maxAccel = 40; // in mps/s
        public static double elevatorPIDTolerance;

        public static enum ElevateMode {
            UP,
            DOWN,
            L1,
            L2,
            L3,
            L4,
            HP,
            OFF,
            MANUAL,
            TEST,
            HOMING
        }

        public static double LevelOneSetpoint;
        public static double LevelTwoSetpoint;
        public static double LevelThreeSetpoint;
        public static double LevelFourSetpoint;
        public static double HPsetpoint;
        public static double test = 14;

    }

    public static final class ArmConstants {
        // Motor and encoder IDs
        public static final int leaderID = 31;
        public static final int followerID = 32;
        public static final int encoderID = 0;

        public static final Rotation2d offset = Rotation2d.fromDegrees(-18); // **WAS 88-90//How far away the arm is (at
                                                                             // rest) from 0 degrees
        public static final Rotation2d max = Rotation2d.fromDegrees(90); // Max. value for the arm's angle
        public static final Rotation2d min = Rotation2d.fromDegrees(-30); // Min. value for the arm's angle
        public static final Rotation2d tolernace = Rotation2d.fromDegrees(1); // How much the arm's exact angle can be
                                                                              // off by

        public static final Rotation2d maxVelocityPerSecond = Rotation2d.fromDegrees(1); // was 500
        public static final Rotation2d maxAcceleration = Rotation2d.fromDegrees(1); // was 250

        public static double[] armSGV = new double[] { 0.0, 0.0, 0.0 }; // Static, Gravity, and Velocity gains
        public static double[] armPID = new double[] { 0, 0, 0f }; // Arm PID values

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
