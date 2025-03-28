package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import org.opencv.core.Mat;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.spark.config.SmartMotionConfig;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.ScoringMode;
import frc.robot.subsystems.Drive.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    public Vision vision = new Vision();
    private Pose2d lastActivePathPose = new Pose2d();
    public ScoringMode scoringMode = ScoringMode.NORMAL;
    // for field centric path following:
    private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    // for robot centric path following
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    // the field
    public Field2d TheField = new Field2d();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    // TODO: If later you change your mind and dont want to flip controls, change
    // either red or blue to match the other
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        // if (Utils.isSimulation()) {
        // startSimThread();
        // }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        // if (Utils.isSimulation()) {
        // startSimThread();
        // }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        // if (Utils.isSimulation()) {
        // startSimThread();
        // }
        SmartDashboard.putData(TheField);
        TheField.setRobotPose(getState().Pose);
        if (vision.getEstimatedGlobalPoseFront().isEmpty()) {
        } else {
            TheField.getObject("vision pls work")
                    .setPose(vision.getEstimatedGlobalPoseFront().get().estimatedPose.toPose2d());
        }
        configureAutoBuilder();
    }

    /* Pathplanner and CTRE Swerve config */
    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getState().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // TODO: TUNE THESE VALUES
                            // PID constants for translation
                            new PIDConstants(4, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(3, 0, 0)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
            );
            PathPlannerLogging.setLogTargetPoseCallback(
                    (targetPose) -> {
                        TheField.getObject("pathplanner target pose").setPose(targetPose);
                    });

            PathPlannerLogging.setLogActivePathCallback(this::handleActivePathLogger);

            SmartDashboard.putData("Vision field", TheField);

            Timer.delay(4); // Let the can bus cool down
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajlogger) {
        return new AutoFactory(
                () -> getState().Pose,
                this::resetPose,
                this::followPath,
                true,
                this,
                trajlogger);
    }

    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {
        });
    }

    public void followPath(SwerveSample target) {
        AutoBuilder.pathfindToPose(target.getPose(),
                new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                        AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                        AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared),
                0);
    }

    private void handleActivePathLogger(List<Pose2d> poses) {
        if (poses.isEmpty())
            return;

        lastActivePathPose = poses.get(poses.size() - 1);
    }

    public void setScoringMode(ScoringMode mode) {
        this.scoringMode = mode;
    }

    public ScoringMode getScoringMode() {
        return this.scoringMode;
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    /**
     * Get both camera's pose estimates, check if present, then take avg of all
     * three components
     * 
     * @return {@link Pose2d} estimated pose based on both vision measurements
     */
    public Pose2d getPoseFromBoth() {
        var visionEst = vision.getEstimatedGlobalPoseFront();
        var visionEst2 = vision.getEstimatedGlobalPoseBack();
        Pose2d combinedEstimate = new Pose2d();
        if (visionEst.isPresent() && visionEst2.isPresent()) {
            Pose2d poseFromFront = visionEst.get().estimatedPose.toPose2d();
            Pose2d poseFromBack = visionEst2.get().estimatedPose.toPose2d();

            double estXfromFront = poseFromFront.getX();
            double estXfromBack = poseFromBack.getX();

            double estYfromFront = poseFromFront.getY();
            double estYfromBack = poseFromBack.getY();

            double estCOSfromFront = poseFromFront.getRotation().getCos();
            double estCOSfromBack = poseFromBack.getRotation().getCos();

            double estSINfromFront = poseFromFront.getRotation().getSin();
            double estSINfromBack = poseFromBack.getRotation().getSin();

            double avgX = (estXfromBack + estXfromFront) / 2;
            double avgY = (estYfromBack + estYfromFront) / 2;

            double avgCos = (estCOSfromBack + estCOSfromFront) / 2;
            double avgSin = (estSINfromBack + estSINfromFront) / 2;

            combinedEstimate = new Pose2d(avgX, avgY, new Rotation2d(avgCos, avgSin));
        } else if (visionEst.isPresent() && (visionEst2.isEmpty())) {

        } else if (visionEst2.isPresent() && (visionEst.isEmpty())) {

        } else {
            combinedEstimate = new Pose2d();
        }
        return combinedEstimate;
    }

    /**
     * Update the pose estimation and std devs with new vision data
     */
    public void updateVisionMeasurements() {
        var visionEst = vision.getEstimatedGlobalPoseFront();
        var visionEst2 = vision.getEstimatedGlobalPoseBack();
        if (visionEst.isPresent() && visionEst2.isPresent()) {
            var estPose = getPoseFromBoth();
            var estStdDevs = vision.getEstimationStdDevs(estPose);
            super.addVisionMeasurement(
                    estPose,
                    Utils.fpgaToCurrentTime(visionEst.get().timestampSeconds),
                    estStdDevs);
        } else if (visionEst.isPresent()) {
            visionEst.ifPresent(est -> {
                var estPose = est.estimatedPose.toPose2d();
                var estStdDevs = vision.getEstimationStdDevs(estPose);
                super.addVisionMeasurement(
                        estPose,
                        Utils.fpgaToCurrentTime(est.timestampSeconds),
                        estStdDevs);
            });
        } else if (visionEst2.isPresent()) {
            visionEst2.ifPresent(est -> {
                var estPose = est.estimatedPose.toPose2d();
                var estStdDevs = vision.getEstimationStdDevs(estPose);
                super.addVisionMeasurement(
                        estPose,
                        Utils.fpgaToCurrentTime(est.timestampSeconds),
                        estStdDevs);
            });
        }
    }

    /**
     * Repeatedly check targets: if not too ambigious, compare with previous
     * closest target to determine which is closer and set
     * closest one as result
     * 
     * @return result: The closest tag (if available)
     */
    public Optional<PhotonTrackedTarget> getClosestTag() {
        var targets = vision.getLatestResultFront().getTargets();
        Optional<PhotonTrackedTarget> result = Optional.empty();

        for (var target : targets) {
            if (target.getPoseAmbiguity() > 0.3) {
                break;
            }
            if (result.isPresent()) {
                if (result.get().getBestCameraToTarget().getTranslation().getNorm() > target.getBestCameraToTarget()
                        .getTranslation().getNorm())
                    result = Optional.of(target);
            } else {
                result = Optional.of(target);
            }
        }
        return result;
    }

    public Pose2d getCenterReefPose() {
        Pose2d target;
        int firstTag;
        int endtag;
        Pose2d currentPose = getState().Pose;
        Pose2d currentPoseCopy = getStateCopy().Pose;
        List<Pose2d> reefTagPoseList = new ArrayList<>(12);
        List<Pose2d> flippedReefTagListwID = new ArrayList<>(6);
        HashMap<Integer, Pose2d> reefTagPoseListwID = new HashMap<>(6);

        if (isRedAlliance()) {
            firstTag = 6;
            endtag = 12;
        } else {
            firstTag = 17;
            endtag = 23;
        }
        for (int i = firstTag; i < endtag; i++) {
            Pose2d aprilTag = vision.kFieldLayout.getTagPose(i).get().toPose2d();
            reefTagPoseList.add(aprilTag);
            reefTagPoseList.add(aprilTag.rotateAround(aprilTag.getTranslation(), Rotation2d.k180deg));
            // flippedReefTagListwID.put(i,
            // aprilTag.rotateAround(aprilTag.getTranslation(), Rotation2d.k180deg));
            flippedReefTagListwID
                    .add(new Pose2d(aprilTag.getTranslation(), aprilTag.getRotation().rotateBy(Rotation2d.k180deg)));

            reefTagPoseListwID.put(i, aprilTag);

        }
        Pose2d nearestPose = currentPose.nearest(reefTagPoseList);
        // Pose2d mostRecent = currentPoseCopy.nearest(reefTagPoseList);
        // used to be rotated by
        // 180, now is accounted for
        if (flippedReefTagListwID.contains(nearestPose)) {
            double angle = nearestPose.getRotation().getRadians();
            target = new Pose2d(
                    nearestPose.getX() + (-Units.inchesToMeters(VisionConstants.bumperToBumper / 2) * Math.cos(angle)),
                    nearestPose.getY() + (-Units.inchesToMeters(VisionConstants.bumperToBumper / 2) * Math.sin(angle)),
                    nearestPose.getRotation());
        } else {
            double angle = nearestPose.getRotation().rotateBy(Rotation2d.k180deg).getRadians();
            target = new Pose2d(
                    nearestPose.getX() + (-Units.inchesToMeters(VisionConstants.bumperToBumper / 2) * Math.cos(angle)),
                    nearestPose.getY() + (-Units.inchesToMeters(VisionConstants.bumperToBumper / 2) * Math.sin(angle)),
                    nearestPose.getRotation());
        }

        if (flippedReefTagListwID.contains(nearestPose)) {
            setScoringMode(ScoringMode.NORMAL);
        } else {
            setScoringMode(ScoringMode.MODIFIED);
        }
        TheField.getObject("nearestpose").setPose(nearestPose);
        TheField.getObject("target").setPose(target);

        return target;
    }

    public ScoringMode decideScoringMode() {
        int firstTag;
        int endtag;
        Pose2d currentPose = getState().Pose;
        Pose2d currentPoseCopy = getStateCopy().Pose;
        List<Pose2d> reefTagPoseList = new ArrayList<>(12);
        List<Pose2d> flippedReefTagListwID = new ArrayList<>(6);
        List<Pose2d> reefTaflistID = new ArrayList<>(6);
        if (isRedAlliance()) {
            firstTag = 6;
            endtag = 12;
        } else {
            firstTag = 17;
            endtag = 23;
        }
        for (int i = firstTag; i < endtag; i++) {
            Pose2d aprilTag = vision.kFieldLayout.getTagPose(i).get().toPose2d();
            reefTagPoseList.add(aprilTag);
            reefTagPoseList
                    .add(new Pose2d(aprilTag.getTranslation(), aprilTag.getRotation().rotateBy(Rotation2d.k180deg)));
            // flippedReefTagListwID.put(i,
            // aprilTag.rotateAround(aprilTag.getTranslation(), Rotation2d.k180deg));
            flippedReefTagListwID
                    .add(new Pose2d(aprilTag.getTranslation(), aprilTag.getRotation().rotateBy(Rotation2d.k180deg)));
            reefTaflistID.add(aprilTag);

        }
        Pose2d nearestPose = currentPose.nearest(reefTagPoseList);
        Pose2d mostRecent = currentPoseCopy.nearest(reefTagPoseList);

        if (flippedReefTagListwID.contains(nearestPose)) {
            return ScoringMode.NORMAL;
        } else if (reefTaflistID.contains(nearestPose)) {
            return ScoringMode.MODIFIED;
        }
        return ScoringMode.MODIFIED;
    }

    public Pose2d addOffset(boolean left) {
        double leftOffset = Units.inchesToMeters(VisionConstants.leftOffsetInches);
        double rightOffset = Units.inchesToMeters(VisionConstants.rightOffsetInches);
        Pose2d centerTarget = getCenterReefPose();
        double angle = (Math.PI / 2) - centerTarget.getRotation().getRadians();
        Pose2d offsetTarget;
        if (!left && (decideScoringMode() == ScoringMode.NORMAL)) {
            offsetTarget = new Pose2d(
                    centerTarget.getX() + (rightOffset * Math.cos(angle)),
                    centerTarget.getY() + (-rightOffset * Math.sin(angle)),
                    centerTarget.getRotation());
        } else if (left && (decideScoringMode() == ScoringMode.NORMAL)) {
            offsetTarget = new Pose2d(
                    centerTarget.getX() + (-leftOffset * Math.cos(angle)),
                    centerTarget.getY() + (leftOffset * Math.sin(angle)),
                    centerTarget.getRotation());
        } else if (!left && (decideScoringMode() == ScoringMode.MODIFIED)) {
            offsetTarget = new Pose2d(
                    centerTarget.getX() + (-rightOffset * Math.cos(angle)),
                    centerTarget.getY() + (rightOffset * Math.sin(angle)),
                    centerTarget.getRotation());
        } else {
            offsetTarget = new Pose2d(
                    centerTarget.getX() + (leftOffset * Math.cos(angle)),
                    centerTarget.getY() + (-leftOffset * Math.sin(angle)),
                    centerTarget.getRotation());
        }
        TheField.getObject("offset target").setPose(offsetTarget);
        return offsetTarget;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("corign mode", this.decideScoringMode() == ScoringMode.NORMAL);

        // updateVisionMeasurements();
        // var globalPose = vision.getEstimatedGlobalPose();
        // if (vision.getEstimatedGlobalPose().isEmpty()){
        // } else {
        // TheField.setRobotPose(vision.getEstimatedGlobalPose().get().estimatedPose.toPose2d());
        // }
        // logValues();

        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
