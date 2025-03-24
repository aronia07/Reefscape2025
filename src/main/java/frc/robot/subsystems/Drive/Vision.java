package frc.robot.subsystems.Drive;

import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  private final PhotonCamera camera;
  private final PhotonCamera camera2;
  private double lastEstTimestamp = 0;
  private double lastEstTimestamp2 = 0;
  private final PhotonPoseEstimator photonPoseEstimatorFront;
  private final PhotonPoseEstimator photonPoseEstimatorBack;
  public Optional<EstimatedRobotPose> latestVision = Optional.empty();
  public Optional<EstimatedRobotPose> latestVision2 = Optional.empty();
  public AprilTagFieldLayout kFieldLayout;

  public Vision() {
    kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    camera = new PhotonCamera(Constants.VisionConstants.cameraName);
    camera2 = new PhotonCamera(Constants.VisionConstants.camera2Name);
    photonPoseEstimatorFront = new PhotonPoseEstimator(kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        VisionConstants.kRobotToCam);
    photonPoseEstimatorBack = new PhotonPoseEstimator(kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
        VisionConstants.kRobotToCam);
    photonPoseEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    photonPoseEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // final Field2d TheField = new Field2d();
    // SmartDashboard.putData("Field", TheField);
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be
   * empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
   *         timestamp, and targets
   *         used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFront() {
    // photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    var estVision = photonPoseEstimatorFront.update(getLatestResultFront());
    double latestTimestamp = getLatestResultFront().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (newResult) {
      lastEstTimestamp = latestTimestamp;
    }
    latestVision = estVision;
    return latestVision;
  }
  
  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseBack() {
    var estVision2 = photonPoseEstimatorBack.update(getLatestResultBack());
    double latestTimestamp = getLatestResultBack().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp2) > 1e-5;
    if (newResult) {
      lastEstTimestamp2 = latestTimestamp;
    }
    latestVision2 = estVision2;
    return latestVision2;
  }

  /**
   * Gets camera's latest result
   * 
   * @return The latest {@link PhotonPipelineResult} from the camera
   */
  public PhotonPipelineResult getLatestResultFront() {
    // List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    // // return results.get(results.size()-1);
    return camera.getLatestResult();
  }

  public PhotonPipelineResult getLatestResultBack() {
    return camera2.getLatestResult();
  }

  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = Constants.VisionConstants.kSingleTagStdDevs;
    var targetsFront = getLatestResultFront().getTargets();
    var targetsBack = getLatestResultBack().getTargets();
    int numTagsFront = 0;
    int numTagsBack = 0;
    double avgDistFront = 0;
    double avgDistBack = 0;
    for (var tgt1 : targetsFront) {
      var tagPose = photonPoseEstimatorFront.getFieldTags().getTagPose(tgt1.getFiducialId());
      if (tagPose.isEmpty())
        continue;
      numTagsFront++;
      avgDistFront += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    for (var tgt2 : targetsFront) {
      var tagPose = photonPoseEstimatorFront.getFieldTags().getTagPose(tgt2.getFiducialId());
      if (tagPose.isEmpty())
        continue;
      numTagsBack++;
      avgDistBack += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTagsFront == 0)
      return estStdDevs;
    avgDistFront /= numTagsFront;
    avgDistBack /= numTagsBack;
    // Decrease std devs if multiple targets are visible
    if (numTagsFront > 1 /*|| numTagsBack > 1*/)
      estStdDevs = Constants.VisionConstants.kMultiTagStdDevs;
    // Increase std devs based on (average) distance
    if ((numTagsFront == 1 && avgDistFront > 4) /*|| (numTagsBack == 1 && avgDistBack > 4)*/)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else
      estStdDevs = estStdDevs.times(1 + (avgDistFront * avgDistFront / 30));

    return estStdDevs;
  }

}
