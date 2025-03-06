package frc.robot.commands.Drive;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class DriveToLocation {
    public static Command driveTo(Pose2d target, CommandSwerveDrivetrain drive) {
      var command = AutoBuilder.pathfindToPose(target,
          new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
              AutoConstants.kMaxAccelerationMetersPerSecondSquared,
              AutoConstants.kMaxAngularSpeedRadiansPerSecond,
              AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));
      command.addRequirements();
  
      return command;
    }
  
    public static Command driveToUnsure(Supplier<Pose2d> target, CommandSwerveDrivetrain drive) {
      return driveTo(target.get(), drive);
    }
    
    public static Command DriveToClosestTag(Transform2d distance, CommandSwerveDrivetrain swerve) {
      var closestTag = swerve.getClosestTag();
      if (closestTag.isEmpty())
        return Commands.none();
  
      var tagPose = swerve.vision.kFieldLayout.getTagPose(closestTag.get().getFiducialId());
      var targetPose = tagPose.get().toPose2d().plus(distance);
  
      return driveTo(targetPose, swerve);
    }
  }
