package frc.robot;


import edu.wpi.first.math.Vector;

import java.util.Optional;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class Constants1 {
    public static boolean enableTunableValues = true;
    public static final class SwerveConstants {
        // Module Names
        public static String[] moduleNames = {"front left", "front right", "back left", "back right"};
        public static int WheelRadius = 2;
    }
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
    public static final class IntakeConstants {
        public static int fivefiftyID = 300;
        public static int[] intakePID = {0, 0, 0};
    }
    public static final class ElevatorConstants {
        public static int min;
        public static int max;
        public static int desiredMin;
        public static int desiredMax;
        public static int leftElevatorMotorID = 401;
        public static int rightElevatorMotorID = 402;
        public static double[] elevatorPID = new double[] {0, 0, 0};
        public static double[] elevatorSGV = new double[] {0, 0, 0};
        public static double maxVelocity; // in rpm
        public static double maxAccel; // in rpm/s
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
            MANUAL
        }
        public static double LevelOneSetpoint;
        public static double LevelTwoSetpoint;
        public static double LevelThreeSetpoint;
        public static double LevelFourSetpoint;
        public static double HPsetpoint;        
        
    }
}
