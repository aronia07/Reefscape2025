package frc.robot;


import edu.wpi.first.math.Vector;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class Constants {
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
}
