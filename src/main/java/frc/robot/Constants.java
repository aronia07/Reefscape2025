package frc.robot;


import edu.wpi.first.math.Vector;

import java.util.Arrays;

import org.opencv.core.Point;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
public final class Constants {


    public static boolean enableTunableValues = true;

    public static final class SwerveConstants {
        // Module Names
        public static String[] moduleNames = {"front left", "front right", "back left", "back right"};
        public static int WheelRadius = 2;
    }
    public static final class VisionConstants {
        public static Transform3d kRobotToCam = null;
        public static String cameraName = "orangepi";
        public static Vector<N3> kSingleTagStdDevs = VecBuilder.fill(2, 2, 4);
        public static Vector<N3> odoStdDEvs = VecBuilder.fill(.2, .2, .2);
        public static double odometryUpdateFrequency = 10;
    }

public static final class ArmConstants {
  //Motor and encoder IDs
    public static final int leaderID = 21;
    public static final int followerID = 22;
    public static final int encoderID = 0;

    public static final Rotation2d offset = Rotation2d.fromDegrees(0);   //**WAS 88-90//How far away the arm is (at rest) from 0 degrees
    public static final Rotation2d max = Rotation2d.fromDegrees(0); //Max. value for the arm's angle
    public static final Rotation2d min = Rotation2d.fromDegrees(0); //Min. value for the arm's angle
    public static final Rotation2d tolernace = Rotation2d.fromDegrees(1); //How much the arm's exact angle can be off by

    public static final Rotation2d maxVelocityPerSecond = Rotation2d.fromDegrees(1); //was 500
    public static final Rotation2d maxAcceleration = Rotation2d.fromDegrees(1); //was 250

    public static double[] armSGV = new double[] { 0.0, 0.0, 0.0 }; //Static, Gravity, and Velocity gains 
    public static double[] armPID = new double[] { 0, 0, 0f }; //Arm PID values

    //Arm's polynomial regression (useed to predict distances from speaker)
    // public static final PolynomialRegression armAngleInterpolationPolynominalRegression = new PolynomialRegression(
    //     Arrays.asList(
    //         new Point(1.43, Units.degreesToRadians(47f)),
    //         new Point(3, Units.degreesToRadians(34f)),
    //         new Point(4, Units.degreesToRadians(29f)),
    //         new Point(5, Units.degreesToRadians(24.5f))),
    //     2);
  }
}
