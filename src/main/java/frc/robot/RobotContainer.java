// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import javax.sound.midi.Sequencer;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.TriggerEvent;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.LightsConstants;
import frc.robot.Constants.WantedSuperState;
import frc.robot.Constants.ArmConstants.ArmWantedMode;
import frc.robot.Constants.ElevatorConstants.ElevateMode;
import frc.robot.Constants.ElevatorConstants.ElevatorWantedMode;
import frc.robot.Constants.IntakeConstants.IntakeWantedMode;
import frc.robot.Constants.IntakeConstants.IntakeWantedMode;
import frc.robot.Constants.VisionConstants.ScoringMode;
import frc.robot.Constants.WristConstants.WristWantedMode;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Arm.ManualArm;
import frc.robot.commands.Arm.ToAngle;
import frc.robot.commands.Climber.Climb;
import frc.robot.commands.Climber.ClimbDown;
import frc.robot.commands.Drive.DriveToLocation;
import frc.robot.commands.Elevator.ElevateLevel;
import frc.robot.commands.Elevator.ElevateManual;
import frc.robot.commands.Elevator.ElevatorCommand;
import frc.robot.commands.Elevator.ElevatorReset;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Wrist.ToWristAngle;
import frc.robot.commands.Wrist.WristMove;
import frc.robot.commands.Intake.IntakeOut;
import frc.robot.commands.Intake.IntakeOut2;
import frc.robot.commands.Intake.IntakeOutL1;
import frc.robot.commands.Intake.IntakeOutVar;
import frc.robot.commands.Intake.Modify;
import frc.robot.commands.Lights.WPIlib.RunPattern;
import frc.robot.commands.Lights.WPIlib.ScrollPattern;
import frc.robot.commands.Lights.WPIlib.SetBreathingPattern;
import frc.robot.commands.Lights.WPIlib.SetSolidColor;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.TunerConstants;
import frc.robot.subsystems.Drive.Vision;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Climber.Climber;

public class RobotContainer {
        /* Subsystems */
        final Arm arm = new Arm();
        final Elevator elevator = new Elevator();
        final Intake intake = new Intake();
        final Wrist wrist = new Wrist();
        final LEDSubsystem_WPIlib wpiLights = new LEDSubsystem_WPIlib();
        final Climber climber = new Climber();
        final Vision vision = new Vision();
        public static boolean isModified = false;
        public SendableChooser<Command> m_chooser = new SendableChooser<>();
        // private Superstructure superstructure = new Superstructure();

        public RobotContainer() {
                configureBindings();
                configureNamedCommands();
                configureTestCommands();
                configureAutoCommands();
                // getDashboardCommand();
        }

        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */ // TODO: move to a drive command
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        // .withSteerRequestType(SteerRequestType.Position);
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final SwerveRequest.FieldCentricFacingAngle faceReef = new SwerveRequest.FieldCentricFacingAngle()
                                        .withHeadingPID(2, 0, 0);
        

        private final Telemetry logger = new Telemetry(MaxSpeed);

        public final CommandXboxController driver = new CommandXboxController(0);
        public final CommandXboxController operator = new CommandXboxController(1);
        public final CommandXboxController controller = new CommandXboxController(2);


        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        public final Trigger beamBroken = new Trigger(() -> intake.hasCoral());
        public final Trigger beamNotBroken = new Trigger(() -> !intake.hasCoral());
        public final Trigger BATTERY_SIDE = new Trigger(() -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE);
        public final Trigger PIVOT_SIDE = new Trigger(() -> drivetrain.decideScoringMode() == ScoringMode.PIVOT_SIDE);

        public void idle() {
                getIdleCommands().schedule();
        }

        public void TESTidle() {
                getTestIdleCommands().schedule();
        }

        public void intakeIdle() {
                getIntakeIdleSeq().schedule();
        }

        private void configureBindings() {

                // Nperte that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.

                drivetrain.registerTelemetry(logger::telemeterize);

                /* DRIVER CONTROLS */
                // drive with joysticks
                drivetrain.setDefaultCommand(
                //                 // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive
                                                                                                                 // forward
                                                                                                                 // with
                                                                                                                 // negative
                                                                                                                 // Y
                                                                                                                 // (forward)
                                                .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with
                                                                                              // negative X (left)
                                                .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive
                                                                                                          // counterclockwise
                                                                                                          // with
                                                                                                          // negative X
                                                                                                          // (left)
                                ));
                // buttons
                // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
                // driver.b().whileTrue(drivetrain
                // .applyRequest(() -> point.withModuleDirection(
                // new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

                // Run SysId routines when holding back/start and X/Y.
                // CHANGED: changed from back/start+x/y to pov buttons
                // Note that each routine should be run exactly once in a single log.
                // d-pad

                // driver.povUp().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // driver.povDown().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // driver.povLeft().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // driver.povRight().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
                // Bumpers and Triggers
                // reset the field-centric heading on left bumper press
                driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // // HP Pickup
                // driver.leftBumper().whileTrue(new SequentialCommandGroup(
                //                 new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                //                 new ParallelCommandGroup(
                //                                 new ToWristAngle(() -> Units.degreesToRadians(60), wrist),
                //                                 new ToAngle(() -> Units.degreesToRadians(1.8), arm), 
                //                                 // new IntakeIn(intake),
                //                                 new ElevateLevel(elevator, ElevateMode.L2))));
                // driver.leftBumper().onFalse(getIntakeIdleSeq());                
                
                //Ground intake
                driver.rightBumper()
                        .onTrue(
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> wrist.setWantedWristMode(WristWantedMode.INTAKE_CORAL)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.INTAKE_CORAL)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.INTAKE_CORAL)),
                                        new InstantCommand(() -> intake.setWantedIntakeMode(IntakeWantedMode.INTAKE_CORAL))))
                        .onFalse(
                                new ParallelCommandGroup(
                                        new InstantCommand(()-> wrist.setWantedWristMode(WristWantedMode.IDLE)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.IDLE)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.IDLE)),
                                        new InstantCommand(() -> intake.setWantedIntakeMode(IntakeWantedMode.IDLE))));

                // driver.rightBumper().onTrue(new SequentialCommandGroup(
                //         new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                //         new ParallelCommandGroup(
                //                         new ToWristAngle(() -> Units.degreesToRadians(34), wrist),
                //                         new ArmCommand(arm, ArmWantedMode.INTAKE_CORAL),
                //                         // new ToAngle(() -> Units.degreesToRadians(-7), arm), 
                //                         new IntakeCommand(intake, IntakeWantedMode.INTAKE_CORAL),
                //                         new ElevateLevel(elevator, ElevateMode.L2))));
                // driver.rightBumper().onFalse(getIntakeIdleSeq());

                // driver.rightBumper().whileTrue(new SequentialCommandGroup(
                //                 new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                //                 new ParallelCommandGroup(
                //                                 new ToWristAngle(() -> Units.degreesToRadians(34), wrist),
                //                                 new ToAngle(() -> Units.degreesToRadians(-6), arm),
                //                                 new IntakeIn(intake),
                //                                 new ElevateLevel(elevator, ElevateMode.HP))
                //                                 .finallyDo(this::intakeIdle)));

                // 9659 inspired align
                driver.leftTrigger().whileTrue(
                        drivetrain.defer(
                                () -> DriveToLocation.driveTo(drivetrain.addOffset(true), drivetrain)));
                driver.rightTrigger().whileTrue(
                        drivetrain.defer(
                                () -> DriveToLocation.driveTo(drivetrain.addOffset(false), drivetrain)));
                // FACE REEF WHEN HAVE CORAL
                // drivetrain.applyRequest(() -> faceReef.withTargetDirection(drivetrain.getReefFaceAngle()));
                /* OPERATOR CONTROLS */
                // joysticks
                arm.setDefaultCommand(new ManualArm(() -> -operator.getLeftY(), arm));
                wrist.setDefaultCommand(new WristMove(() -> -operator.getRightY(), wrist));

                // buttons
                // operator.rightBumper().whileTrue(new IntakeOut(intake));
                // operator.leftBumper().whileTrue(new IntakeCommand(intake, IntakeWantedMode.INTAKE_ALGAE));
                // operator.leftBumper().onFalse(new IntakeCommand(intake, IntakeWantedMode.IDLE));
                operator.leftBumper()
                        .onTrue(new InstantCommand(() -> intake.setWantedIntakeMode(IntakeWantedMode.INTAKE_ALGAE)))
                        .onFalse(new InstantCommand(() -> intake.setWantedIntakeMode(IntakeWantedMode.IDLE)));
                // BATTERY_SIDE outtake
                operator.rightBumper().and(()-> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE).and(() -> (operator.x().getAsBoolean() == false))
                        .onTrue(new InstantCommand(() -> intake.setWantedIntakeMode(IntakeWantedMode.SCORE_CORAL_BATTERYSIDE)))
                        .onFalse(new InstantCommand(() -> intake.setWantedIntakeMode(IntakeWantedMode.IDLE)));

                operator.rightBumper().and(operator.x())
                        .onTrue(new InstantCommand(() -> intake.setWantedIntakeMode(IntakeWantedMode.SCORE_CORAL_L1)))
                        .onFalse(new InstantCommand(() -> intake.setWantedIntakeMode(IntakeWantedMode.IDLE)));

                operator.rightBumper().and(() -> drivetrain.decideScoringMode() == ScoringMode.PIVOT_SIDE).and(() -> (operator.x().getAsBoolean() == false))
                        .onTrue(new InstantCommand(() -> intake.setWantedIntakeMode(IntakeWantedMode.SCORE_CORAL_PIVOTSIDE)))
                        .onFalse(new InstantCommand(() -> intake.setWantedIntakeMode(IntakeWantedMode.IDLE)));

                // operator.rightBumper().onFalse(new IntakeCommand(intake, IntakeWantedMode.IDLE));
                // operator.rightBumper().and(operator.x()).whileTrue(
                //                 new IntakeOutL1(intake));

                // operator.rightBumper().and(operator.b())
                //                 .and(() -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE).whileTrue(
                //                                 new IntakeOut(intake));

                // operator.rightBumper().and(operator.y())
                //                 .and(() -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE)
                //                 .whileTrue(new IntakeOut(intake));

                // operator.rightBumper().and(operator.a()).whileTrue(new IntakeOutVar(intake, () -> -0.2));

                // operator.rightBumper().and(() -> drivetrain.decideScoringMode() == ScoringMode.PIVOT_SIDE).and(operator.b())
                //                 .whileTrue(new IntakeOut2(intake));

                // operator.rightBumper().and(() -> drivetrain.decideScoringMode() == ScoringMode.PIVOT_SIDE).and(operator.y())
                //                 .whileTrue(new IntakeOutVar(intake, () -> 0.7));

                // operator.rightBumper().and(operator.b()).and(operator.rightTrigger()).whileTrue(
                //                 new IntakeOut2(intake));

                // operator.rightBumper().and(operator.a()).and(operator.rightTrigger()).whileTrue(
                //                 new IntakeOut2(intake));

                // operator.x().onTrue(new SetSolidColor(wpiLights, Color.kMagenta));

                // if(drivetrain.getScoringMode() == ScoringMode.PIVOT_SIDE) {
                // if (drivetrain.decideScoringMode() == ScoringMode.PIVOT_SIDE) {
                // L4
                // operator.y().and(() -> drivetrain.decideScoringMode() == ScoringMode.PIVOT_SIDE)
                //         .whileTrue(new SequentialCommandGroup(
                //                 new ParallelCommandGroup(
                //                         new ArmCommand(arm,ArmWantedMode.L4_CORAL_PIVOT),
                //                         new ToWristAngle(() -> Units.degreesToRadians(-88), wrist)),
                //                 new ElevatorCommand(elevator, ElevatorWantedMode.L4_CORAL_PIVOT)));
                // L4 PIVOT SIDE
                operator.y().and(() -> drivetrain.decideScoringMode() == ScoringMode.PIVOT_SIDE).and(beamBroken)
                        .onTrue(
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> wrist.setWantedWristMode(WristWantedMode.L4_CORAL_PIVOT)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.L4_CORAL_PIVOT)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.L4_CORAL_PIVOT))))
                        .onFalse(
                                new ParallelCommandGroup(
                                        new InstantCommand(()-> wrist.setWantedWristMode(WristWantedMode.IDLE)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.IDLE)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.IDLE))));
                // operator.y().and(() -> drivetrain.decideScoringMode() == ScoringMode.PIVOT_SIDE)
                //         .whileTrue(
                //                 new ParallelCommandGroup(
                //                         new ArmCommand(arm, ArmWantedMode.L4_CORAL_PIVOT),
                //                         new ToWristAngle(() -> Units.degreesToRadians(-88), wrist),
                //                         new ElevatorCommand(elevator, ElevatorWantedMode.L4_CORAL_PIVOT)));
                // operator.y().onFalse(getIdleCommands());
                // L3
                // operator.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.PIVOT_SIDE)
                //         .whileTrue(new SequentialCommandGroup(
                //                 new ParallelCommandGroup(
                //                         new ToWristAngle(() -> Units.degreesToRadians(-80), wrist),
                //                         new ArmCommand(arm, ArmWantedMode.L3_CORAL_PIVOT)),
                //                 new ElevatorCommand(elevator, ElevatorWantedMode.L3_CORAL_PIVOT)));
                // L3 PIVOT SIDE
                operator.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.PIVOT_SIDE).and(beamBroken)
                        .onTrue(
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> wrist.setWantedWristMode(WristWantedMode.L3_CORAL_PIVOT)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.L3_CORAL_PIVOT)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.L3_CORAL_PIVOT))))
                        .onFalse(
                                new ParallelCommandGroup(
                                        new InstantCommand(()-> wrist.setWantedWristMode(WristWantedMode.IDLE)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.IDLE)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.IDLE))));
                // L2
                operator.a()
                        .onTrue(
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> wrist.setWantedWristMode(WristWantedMode.L2_CORAL)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.L2_CORAL)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.L2_CORAL))))
                        .onFalse(
                                new ParallelCommandGroup(
                                        new InstantCommand(()-> wrist.setWantedWristMode(WristWantedMode.IDLE)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.IDLE)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.IDLE))));
                // operator.a().whileTrue(new SequentialCommandGroup(
                //                 new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                //                 new ParallelCommandGroup(
                //                                 new ToAngle(() -> Units.degreesToRadians(45), arm),
                //                                 new ToWristAngle(() -> Units.degreesToRadians(-40), wrist)),
                //                 new ElevateLevel(elevator, ElevateMode.L2)));
                // operator.a().onFalse(getIdleCommands());
                // L1
                operator.x()
                        .onTrue(
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> wrist.setWantedWristMode(WristWantedMode.L1)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.L1)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.L1))))
                        .onFalse(
                                new ParallelCommandGroup(
                                        new InstantCommand(()-> wrist.setWantedWristMode(WristWantedMode.IDLE)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.IDLE)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.IDLE))));
                // operator.x().whileTrue(new SequentialCommandGroup(
                //                 new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                //                 new ParallelCommandGroup(
                //                                 new ToAngle(() -> Units.degreesToRadians(20), arm),
                //                                 new ToWristAngle(() -> Units.degreesToRadians(-3), wrist)),
                //                 new ElevateLevel(elevator, ElevateMode.L1)));
                // operator.x().onFalse(getIdleCommands());

                // } else {
                // L4 battery side
                operator.y().and(() -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE).and(beamBroken)
                        .onTrue(
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> wrist.setWantedWristMode(WristWantedMode.L4_CORAL_BATTERY)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.L4_CORAL_BATTERY)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.L4_CORAL_BATTERY))))
                        .onFalse(
                                new ParallelCommandGroup(
                                        new InstantCommand(()-> wrist.setWantedWristMode(WristWantedMode.IDLE)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.IDLE)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.IDLE))));
                // operator.y().and(() -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE)
                //         .whileTrue(
                //                 new ParallelCommandGroup(
                //                         new ArmCommand(arm, ArmWantedMode.L4_CORAL_BATTERY),
                //                         new ToWristAngle(() -> Units.degreesToRadians(3), wrist),
                //                         new ElevatorCommand(elevator, ElevatorWantedMode.L4_CORAL_BATTERY)));

                // operator.y().and(() -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE)
                //         .whileTrue(new SequentialCommandGroup(
                //                 new ParallelCommandGroup(
                //                         new ArmCommand(arm, ArmWantedMode.L4_CORAL_BATTERY),
                //                         new ToWristAngle(() -> Units.degreesToRadians(3), wrist)),
                //                 new ElevatorCommand(elevator, ElevatorWantedMode.L4_CORAL_BATTERY)));
                // L3
                // operator.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE)
                //         .whileTrue(new SequentialCommandGroup(
                //                 new ParallelCommandGroup(
                //                         new ToWristAngle(() -> Units.degreesToRadians(-58), wrist),
                //                         new ArmCommand(arm, ArmWantedMode.L3_CORAL_BATTERY)),
                //                 new ElevatorCommand(elevator, ElevatorWantedMode.L3_CORAL_BATTERY)));
                // L3 battery side
                operator.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE).and(beamBroken)
                        .onTrue(
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> wrist.setWantedWristMode(WristWantedMode.L3_CORAL_BATTERY)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.L3_CORAL_BATTERY)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.L3_CORAL_BATTERY))))
                        .onFalse(
                                new ParallelCommandGroup(
                                        new InstantCommand(()-> wrist.setWantedWristMode(WristWantedMode.IDLE)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.IDLE)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.IDLE))));
                // operator.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE)
                //         .whileTrue(
                //                 new ParallelCommandGroup(
                //                         new ArmCommand(arm, ArmWantedMode.L3_CORAL_BATTERY),
                //                         new ToWristAngle(() -> Units.degreesToRadians(-58), wrist),
                //                         new ElevatorCommand(elevator, ElevatorWantedMode.L3_CORAL_BATTERY)));
        
                // operator.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE)
                //         .whileTrue(new SequentialCommandGroup(
                //                 new ToWristAngle(() -> Units.degreesToRadians(-58), wrist),
                //                 new ArmCommand(arm, ArmWantedMode.L3_CORAL_BATTERY),
                //                 new ElevatorCommand(elevator, ElevatorWantedMode.L3_CORAL_BATTERY)));


                // L3 Algae Removal dunk
                operator.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.PIVOT_SIDE).and(beamNotBroken)
                        .onTrue(
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> wrist.setWantedWristMode(WristWantedMode.L3_ALGAE_PIVOT)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.L3_ALGAE_PIVOT)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.L3_ALGAE_PIVOT))))
                        .onFalse(
                                new ParallelCommandGroup(
                                        new InstantCommand(()-> wrist.setWantedWristMode(WristWantedMode.IDLE)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.IDLE)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.IDLE))));
                // driver.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.PIVOT_SIDE)
                //                 .whileTrue(new SequentialCommandGroup(
                //                                 new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                //                                 new ParallelCommandGroup(
                //                                                 new ToWristAngle(() -> Units.degreesToRadians(-53),
                //                                                                 wrist),
                //                                                 new ToAngle(() -> Units.degreesToRadians(87), arm)),
                //                                 new ElevateLevel(elevator, ElevateMode.L3AR)));

                
                // driver.a().and(() -> drivetrain.decideScoringMode() == ScoringMode.PIVOT_SIDE)
                //                 .whileTrue(new SequentialCommandGroup(
                //                                 new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                //                                 new ParallelCommandGroup(
                //                                                 new ToWristAngle(() -> Units.degreesToRadians(-76),
                //                                                                 wrist),
                //                                                 new ToAngle(() -> Units.degreesToRadians(88.67), arm)),
                //                                 new ElevateLevel(elevator, ElevateMode.L2AR)));

                // L3 Algae Removal reach
                operator.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE).and(beamNotBroken)
                        .onTrue(
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> wrist.setWantedWristMode(WristWantedMode.L3_ALGAE_BATTERY)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.L3_ALGAE_BATTERY)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.L3_ALGAE_BATTERY))))
                        .onFalse(
                                new ParallelCommandGroup(
                                        new InstantCommand(()-> wrist.setWantedWristMode(WristWantedMode.IDLE)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.IDLE)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.IDLE))));
                // driver.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE)
                //                 .whileTrue(new SequentialCommandGroup(
                //                                 new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                //                                 new ParallelCommandGroup(
                //                                                 new ToWristAngle(() -> Units.degreesToRadians(44.5),
                //                                                                 wrist),
                //                                                 new ToAngle(() -> Units.degreesToRadians(65), arm)),
                //                                 new ElevateLevel(elevator, ElevateMode.L3R)));
                // Algae align
                driver.b()
                        .whileTrue(drivetrain.defer(
                                () -> DriveToLocation.driveTo(drivetrain.getCenterReefPose(), drivetrain))
                                        .until(() -> drivetrain.getState().Pose == drivetrain.getCenterReefPose()));
                // // L3 algae align forward
                // driver.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.PIVOT_SIDE).whileTrue(
                //         drivetrain.defer(
                //                 () -> DriveToLocation.driveTo(drivetrain.getCenterReefPose(), drivetrain)));

                // L2 Algae Removal Battery side
                operator.a().and(() -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE).and(beamNotBroken)
                        .onTrue(
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> wrist.setWantedWristMode(WristWantedMode.L2_ALGAE_BATTERY)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.L2_ALGAE_BATTERY)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.L2_ALGAE_BATTERY))))
                        .onFalse(
                                new ParallelCommandGroup(
                                        new InstantCommand(()-> wrist.setWantedWristMode(WristWantedMode.IDLE)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.IDLE)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.IDLE))));
                // driver.a().and(() -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE)
                //                 .whileTrue(new SequentialCommandGroup(
                //                                 new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                //                                 new ParallelCommandGroup(
                //                                                 new ToWristAngle(() -> Units.degreesToRadians(-3),
                //                                                                 wrist),
                //                                                 new ToAngle(() -> Units.degreesToRadians(25), arm)),
                //                                 new ElevateLevel(elevator, ElevateMode.L2AR)));
                // L2 Algae Removal Pivot side
                operator.a().and(() -> drivetrain.decideScoringMode() == ScoringMode.PIVOT_SIDE).and(beamNotBroken)
                        .onTrue(
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> wrist.setWantedWristMode(WristWantedMode.L2_ALGAE_PIVOT)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.L2_ALGAE_PIVOT)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.L2_ALGAE_PIVOT))))
                        .onFalse(
                                new ParallelCommandGroup(
                                        new InstantCommand(()-> wrist.setWantedWristMode(WristWantedMode.IDLE)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.IDLE)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.IDLE))));
                                        
                operator.start().onTrue(new ElevatorReset(elevator));
                beamBroken.onTrue(new SetSolidColor(wpiLights, LightsConstants.GRBColors.get("green")));
                beamBroken.onFalse(new SetSolidColor(wpiLights, LightsConstants.GRBColors.get("blue")));
                // d-pad
                operator.povDown()
                        .onTrue(new InstantCommand(() -> climber.climberOpen()))
                        .onFalse(new InstantCommand(() -> climber.climberMotor.set(0)));
                operator.povUp()
                        .onTrue(new InstantCommand(() -> climber.climberClimb()))
                        .onFalse(new InstantCommand(() -> climber.climberMotor.set(0)));

                // operator.povDown().whileTrue(new Climb(climber, () -> -1));
                // operator.povUp().whileTrue(new Climb(climber, () -> 1));

                // operator.povLeft()
                //         .onTrue(new InstantCommand(() -> intake.setWantedIntakeMode(IntakeWantedMode.SCORE_CORAL_BATTERYSIDE)))
                //         .onFalse(new InstantCommand(() -> intake.setWantedIntakeMode(IntakeWantedMode.IDLE)));
                // operator.povRight()
                //         .onTrue(new InstantCommand(() -> intake.setWantedIntakeMode(IntakeWantedMode.SCORE_CORAL_PIVOTSIDE)))
                //         .onFalse(new InstantCommand(() -> intake.setWantedIntakeMode(IntakeWantedMode.IDLE)));
                
                
                //barge
                operator.leftTrigger()
                        .onTrue(
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> wrist.setWantedWristMode(WristWantedMode.ALGAE_BARGE)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.ALGAE_BARGE)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.ALGAE_BARGE))))
                        .onFalse(
                                new ParallelCommandGroup(
                                        new InstantCommand(()-> wrist.setWantedWristMode(WristWantedMode.IDLE)),
                                        new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.IDLE)),
                                        new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.IDLE))));
                // operator.leftTrigger().whileTrue(new SequentialCommandGroup(
                //                 new ParallelCommandGroup(
                //                                 new ToAngle(() -> Units.degreesToRadians(78), arm), // 75
                //                                 new ToWristAngle(() -> Units.degreesToRadians(32), wrist)), // 36.5
                //                 new ElevateLevel(elevator, ElevateMode.L4).withTimeout(0.7),
                //                 new IntakeOut(intake).withTimeout(0.5)));
                // operator.leftTrigger().onFalse(getIdleCommands());

/* OFFSEASON CODE TESTING GROUND */ ////////////////////////////////////////////////////////////////////
        // //Combined controllers test
        // drivetrain.setDefaultCommand(
        //         // Drivetrain will execute this command periodically
        //         drivetrain.applyRequest(() -> drive.withVelocityX(-controller.getLeftY() * MaxSpeed) 
        //                         .withVelocityY(-controller.getLeftX() * MaxSpeed)
        //                         .withRotationalRate(-controller.getRightX() * MaxAngularRate)));
        // /*intaking coral*/
        controller.rightBumper().whileTrue(new SequentialCommandGroup(
                        new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                        new ParallelCommandGroup(
                                        new ToWristAngle(() -> Units.degreesToRadians(34), wrist),
                                        new ToAngle(() -> Units.degreesToRadians(-7), arm), 
                                        new IntakeCommand(intake, IntakeWantedMode.INTAKE_CORAL),
                                        new ElevateLevel(elevator, ElevateMode.L2))));
        
        /*handling algae*/
        // operator.povDown().whileTrue(new IntakeCommand(intake, IntakeWantedMode.INTAKE_ALGAE));
        // operator.povUp().whileTrue(new IntakeCommand(intake, IntakeWantedMode.SCORE_ALGAE));

        // /* SCORING */
        //scoring coral
        controller.leftBumper().and(()-> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE).and(() -> (controller.x().getAsBoolean() == false)).whileTrue(
                new IntakeCommand(intake, IntakeWantedMode.SCORE_CORAL_BATTERYSIDE));

        controller.leftBumper().and(() -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE).and(controller.x()).whileTrue(
                new IntakeCommand(intake, IntakeWantedMode.SCORE_CORAL_L1));

        controller.leftBumper().and(() -> drivetrain.decideScoringMode() == ScoringMode.PIVOT_SIDE).and(() -> (controller.x().getAsBoolean() == false)).whileTrue(
                new IntakeCommand(intake, IntakeWantedMode.SCORE_CORAL_PIVOTSIDE));
        //L4 Coral pivot side
        controller.y().and(() -> drivetrain.decideScoringMode() == ScoringMode.PIVOT_SIDE).and(beamBroken)
                .whileTrue(new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new ToAngle(() -> Units.degreesToRadians(82), arm),
                                new ToWristAngle(() -> Units.degreesToRadians(-88), wrist)),
                        new ElevateLevel(elevator, ElevateMode.L4)));
        //L4 Coral battery side
        controller.y().and(() -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE).and(beamBroken)
                .whileTrue(new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new ToAngle(() -> Units.degreesToRadians(77), arm),
                                new ToWristAngle(() -> Units.degreesToRadians(3), wrist)),
                        new ElevateLevel(elevator, ElevateMode.L4)));
        //L3 Coral pivot side
        controller.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE).and(beamBroken)
                .onTrue(new SequentialCommandGroup(
                                new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                                new ParallelCommandGroup(
                                        new ToWristAngle(() -> Units.degreesToRadians(-58), wrist),
                                        new ToAngle(() -> Units.degreesToRadians(55), arm)),
                                new ElevateLevel(elevator, ElevateMode.L3M)));
        //L3 Algae removal pivot side
        controller.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.PIVOT_SIDE).and(beamNotBroken)
                .onTrue(new SequentialCommandGroup(
                                new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                                new ParallelCommandGroup(
                                        new ToWristAngle(() -> Units.degreesToRadians(-53), wrist),
                                        new ToAngle(() -> Units.degreesToRadians(87), arm)),
                                new ElevateLevel(elevator, ElevateMode.L3AR)));
        //L3 Coral battery side
        controller.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.PIVOT_SIDE).and(beamBroken)
                .onTrue(new SequentialCommandGroup(
                        new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                        new ParallelCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(-80), wrist),
                                new ToAngle(() -> Units.degreesToRadians(77), arm)),
                        new ElevateLevel(elevator, ElevateMode.L3)));
        //L3 Algae removal battery side
        controller.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE).and(beamNotBroken)
                .onTrue(new SequentialCommandGroup(
                                new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                                new ParallelCommandGroup(
                                                new ToWristAngle(() -> Units.degreesToRadians(44.5), wrist),
                                                new ToAngle(() -> Units.degreesToRadians(65), arm)),
                                new ElevateLevel(elevator, ElevateMode.L3R)));
        //L2 Coral 
        controller.a().and(beamBroken).whileTrue(new SequentialCommandGroup(
                new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                new ParallelCommandGroup(
                        new ToAngle(() -> Units.degreesToRadians(45), arm),
                        new ToWristAngle(() -> Units.degreesToRadians(-40), wrist)),
                new ElevateLevel(elevator, ElevateMode.L2)));
        //L2 Algae Removal battery side
        controller.a().and(() -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE).and(beamNotBroken)
                .whileTrue(new SequentialCommandGroup(
                        new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                        new ParallelCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(-3), wrist),
                                new ToAngle(() -> Units.degreesToRadians(25), arm)),
                        new ElevateLevel(elevator, ElevateMode.L2AR)));
        //L2 Algae Removal pivot side
        controller.a().and(() -> drivetrain.decideScoringMode() == ScoringMode.PIVOT_SIDE).and(beamNotBroken)
                .whileTrue(new SequentialCommandGroup(
                        new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                        new ParallelCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(-76), wrist),
                                new ToAngle(() -> Units.degreesToRadians(88.67), arm)),
                        new ElevateLevel(elevator, ElevateMode.L2AR)));
        //L1 Coral
        controller.x().whileTrue(new SequentialCommandGroup(
                new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                new ParallelCommandGroup(
                        new ToAngle(() -> Units.degreesToRadians(20), arm),
                        new ToWristAngle(() -> Units.degreesToRadians(-3), wrist)),
                new ElevateLevel(elevator, ElevateMode.L1)));

        /* IDLE */
        controller.rightBumper().onFalse(getIntakeIdleSeq());
        controller.leftBumper().onFalse(new IntakeCommand(intake, IntakeWantedMode.IDLE));
        controller.a().onFalse(getIdleCommands());
        controller.b().onFalse(getIdleCommands());
        controller.x().onFalse(getIdleCommands());
        controller.y().onFalse(getIdleCommands());
        controller.povDown().onFalse(new IntakeCommand(intake, IntakeWantedMode.IDLE));
        controller.povUp().onFalse(new IntakeCommand(intake, IntakeWantedMode.IDLE));

        
        }

        public void getDashboardCommand() {
        }

        public Command getIdleCommands() {
                return new ParallelCommandGroup(
                        new ToWristAngle(() -> Units.degreesToRadians(-77), wrist),
                                new ElevatorCommand(elevator, ElevatorWantedMode.IDLE),
                                new ArmCommand(arm, ArmWantedMode.IDLE));
                                // new ToAngle(() -> Units.degreesToRadians(60), arm)));
        }

        public Command getTestIdleCommands() {
                return new SequentialCommandGroup(
                        new ToWristAngle(() -> Units.degreesToRadians(-77), wrist),
                        new ParallelCommandGroup(
                                new ElevateLevel(elevator, ElevateMode.L1),
                                new ToAngle(() -> Units.degreesToRadians(60), arm)));
        }

        public Command getIntakeIdleSeq() {
                return new ParallelCommandGroup(
                        new IntakeCommand(intake, IntakeWantedMode.IDLE),
                        new ToWristAngle(() -> Units.degreesToRadians(-76), wrist),
                        new ToAngle(() -> Units.degreesToRadians(20), arm));
        }

        public Command getAutonomousCommand() {
                // return Commands.print("No autonomous command configured");
                return m_chooser.getSelected();
        }

        public void configureTestCommands() {
                // SmartDashboard.putBoolean("is it modified", Intake.modified);
                // SmartDashboard.putData("Elevate", new ElevateLevel(elevator,
                // ElevateMode.TEST));
                // SmartDashboard.putData("Go down", new ElevateLevel(elevator,
                // ElevateMode.DOWN));

                // SmartDashboard.putData("Wrist Up", new ToWristAngle(() -> 70, wrist));
                // SmartDashboard.putData("Wrist Down", new ToWristAngle(() -> -35, wrist));
                // SmartDashboard.putData("Wrist Neuteral", new ToWristAngle(() -> 0, wrist));

        }

        public void configureAutoCommands() {
                m_chooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("autos", m_chooser);
        }

       public void disabledActions() {
                // new SetBreathingPattern(wpiLights, LEDPattern.solid(LightsConstants.GBRColors.get("magenta")), 1);
                // beamBroken.onTrue(new SetSolidColor(wpiLights, LightsConstants.GRBColors.get("green")));
                // beamBroken.onFalse(new SetSolidColor(wpiLights, LightsConstants.GRBColors.get("blue")));
                new ScrollPattern(wpiLights, LEDPattern.rainbow(255, 64), 100);
                arm.resetI();
                arm.runState(new TrapezoidProfile.State(Arm.getEncoderPosition().getRadians(), 0));
                wrist.runState(new TrapezoidProfile.State(wrist.getEncoderPosition().getRadians(), 0));
        }

        public void configureNamedCommands() {
                // NamedCommands.registerCommand("L4", 
                //         new ParallelCommandGroup(
                //                 new InstantCommand(() -> wrist.setWantedWristMode(WristWantedMode.L4_CORAL_PIVOT)),
                //                 new InstantCommand(() -> elevator.setWantedElevatorMode(ElevatorWantedMode.L4_CORAL_PIVOT)),
                //                 new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.L4_CORAL_PIVOT))));

                // NamedCommands.registerCommand("armToL4", 
                //         new ParallelCommandGroup(
                //                 new InstantCommand(() -> wrist.setWantedWristMode(WristWantedMode.L4_CORAL_PIVOT)),
                //                 new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.L4_CORAL_PIVOT))));

                // NamedCommands.registerCommand("outtake", 
                //         new InstantCommand(() -> intake.setWantedIntakeMode(IntakeWantedMode.SCORE_CORAL_PIVOTSIDE)));

                // NamedCommands.registerCommand("intake", 
                //         new ParallelCommandGroup(
                //                 new InstantCommand(() -> intake.setWantedIntakeMode(IntakeWantedMode.INTAKE_CORAL)),
                //                 new InstantCommand(() -> arm.setWantedArmMode(ArmWantedMode.INTAKE_CORAL)),
                //                 new InstantCommand(() -> wrist.setWantedWristMode(WristWantedMode.INTAKE_CORAL))));

                NamedCommands.registerCommand("reset", new SequentialCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(-61.5), wrist),
                                new ParallelCommandGroup(
                                                new ToAngle(() -> Units.degreesToRadians(70), arm),
                                                new ElevateLevel(elevator, ElevateMode.L2).withTimeout(0.5))));
                NamedCommands.registerCommand("PathFindLeft", drivetrain.defer(
                                () -> DriveToLocation.driveTo(drivetrain.addOffset(true), drivetrain)));
                NamedCommands.registerCommand("PathFindRight", drivetrain.defer(
                                () -> DriveToLocation.driveTo(drivetrain.addOffset(false), drivetrain)));

        }
}
