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
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants.ElevateMode;
import frc.robot.commands.Arm.ManualArm;
import frc.robot.commands.Arm.ToAngle;
import frc.robot.commands.Climber.Climb;
import frc.robot.commands.Climber.ClimbDown;
import frc.robot.commands.Drive.DriveToLocation;
import frc.robot.commands.Elevator.ElevateLevel;
import frc.robot.commands.Elevator.ElevateManual;
import frc.robot.commands.Elevator.ElevatorReset;
import frc.robot.commands.Intake.IntakeIn;
//import frc.robot.commands.Intake.IntakeNotifier;
import frc.robot.commands.Wrist.ToWristAngle;
import frc.robot.commands.Wrist.WristMove;
import frc.robot.commands.Intake.IntakeOut;
import frc.robot.commands.Intake.IntakeOut2;
import frc.robot.commands.Intake.Modify;
import frc.robot.commands.Lights.WPIlib.RunPattern;
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

        public static Command switchOuttake(boolean modification) {
                RobotContainer.isModified = modification;
                return Commands.none();
        }

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

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController driver = new CommandXboxController(0);
        private final CommandXboxController operator = new CommandXboxController(1);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        public void idle() {
                getIdleCommands().schedule();
        }

        private void configureBindings() {
                // Nperte that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.

                drivetrain.registerTelemetry(logger::telemeterize);

                /* DRIVER CONTROLS */
                // drive with joysticks
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
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
                driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
                driver.b().whileTrue(drivetrain
                                .applyRequest(() -> point.withModuleDirection(
                                                new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
                driver.x().whileTrue(new ParallelCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(-77), wrist),
                                new ToAngle(() -> Units.degreesToRadians(35), arm),
                                new ElevateLevel(elevator, ElevateMode.L2),
                                new IntakeIn(intake)).withTimeout(0.8));
                // driver.x().whileTrue(
                // new ProxyCommand(
                // DriveToLocation.driveTo(new Pose2d(
                // vision.kFieldLayout.getTagPose(
                // drivetrain.getClosestTag().get().getFiducialId()).get().toPose2d().getTranslation(),
                // vision.kFieldLayout.getTagPose(
                // drivetrain.getClosestTag().get().getFiducialId()).get().toPose2d().getRotation().unaryMinus()),
                // drivetrain)));
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

                // HP Pickup
                driver.rightBumper().whileTrue(new ParallelCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(-77), wrist),
                                new ToAngle(() -> Units.degreesToRadians(34), arm),
                                new IntakeIn(intake),
                                new ElevateLevel(elevator, ElevateMode.HP)).finallyDo(this::idle));

                // 353 inspired align
                // driver.leftTrigger().whileTrue(drivetrain.reefAlign(true));
                // 9659 inspired align
                driver.leftTrigger().whileTrue(
                                drivetrain.defer(
                                                () -> DriveToLocation.driveTo(drivetrain.addOffset(true), drivetrain)));
                driver.rightTrigger().whileTrue(
                                drivetrain.defer(() -> DriveToLocation.driveTo(drivetrain.addOffset(false),
                                                drivetrain)));
                // driver.rightTrigger().whileTrue(
                // drivetrain.defer(() ->
                // DriveToLocation.driveTo(drivetrain.getCenterReefPose(), drivetrain)));
                /* OPERATOR CONTROLS */
                // joysticks
                arm.setDefaultCommand(new ManualArm(() -> -operator.getLeftY(), arm));
                // elevator.setDefaultCommand(new ElevateManual(() -> operator.getRightY(),
                // elevator));
                wrist.setDefaultCommand(new WristMove(() -> -operator.getRightY(), wrist));

                // buttons
                // if(!isModified) {
                // operator.leftTrigger().whileTrue(new IntakeOut(intake));
                // } else {
                // operator.leftTrigger().whileTrue(new IntakeOut2(intake));
                // }
                operator.rightBumper().whileTrue(new IntakeOut(intake));
                operator.leftBumper().whileTrue(new IntakeOut2(intake));
                // modified outtake
                operator.rightBumper().and(operator.x()).whileTrue(
                                new IntakeOut2(intake));

                operator.rightBumper().and(operator.b()).and(operator.rightTrigger()).whileTrue(
                                new IntakeOut(intake));

                operator.rightBumper().and(operator.a()).and(operator.rightTrigger()).whileTrue(
                                new IntakeOut2(intake));

                // operator.x().onTrue(new SetSolidColor(wpiLights, Color.kMagenta));

                // L4
                operator.y().whileTrue(new SequentialCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(-40), wrist),
                                new ParallelCommandGroup(
                                                new ToAngle(() -> Units.degreesToRadians(87), arm),
                                                new ElevateLevel(elevator, ElevateMode.L4)))
                                .finallyDo(this::idle));

                // Modified L4
                operator.y().and(operator.rightTrigger()).whileTrue(new SequentialCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(10), wrist),
                                new ParallelCommandGroup(
                                                new ToAngle(() -> 75, arm),
                                                new ElevateLevel(elevator, ElevateMode.L4)))
                                .finallyDo(this::idle));

                // L3
                operator.b().whileTrue(new SequentialCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(-48), wrist),
                                new ParallelCommandGroup(
                                                new ToAngle(() -> Units.degreesToRadians(86), arm),
                                                new ElevateLevel(elevator, ElevateMode.L3)))
                                .finallyDo(this::idle));

                // L3 Algae Removal
                operator.b().and(operator.rightTrigger()).whileTrue(new ParallelCommandGroup(
                                new ToAngle(() -> Units.degreesToRadians(87), arm),
                                new ElevateLevel(elevator, ElevateMode.L1),
                                new ToWristAngle(() -> Units.degreesToRadians(-53), wrist)).finallyDo(this::idle));

                // L2
                operator.a().whileTrue(new SequentialCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(-67), wrist),
                                new ProxyCommand(RobotContainer.switchOuttake(false)),
                                new ParallelCommandGroup(
                                                new ToAngle(() -> Units.degreesToRadians(78), arm),
                                                new ElevateLevel(elevator, ElevateMode.L2)))
                                .finallyDo(this::idle));

                // L2 Algae Removal
                operator.a().and(operator.rightTrigger()).whileTrue(new SequentialCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(-72), wrist),
                                new ProxyCommand(RobotContainer.switchOuttake(false)),
                                new ParallelCommandGroup(
                                                new ToAngle(() -> Units.degreesToRadians(88.67), arm),
                                                new ElevateLevel(elevator, ElevateMode.L2AR)))
                                .finallyDo(this::idle));

                // L1
                operator.x().whileTrue(new SequentialCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(-74), wrist),
                                new ProxyCommand(RobotContainer.switchOuttake(true)),
                                new ParallelCommandGroup(
                                                new ToAngle(() -> Units.degreesToRadians(12.4), arm),
                                                new ElevateLevel(elevator, ElevateMode.L1)))
                                .finallyDo(this::idle));

                // Reset
                driver.rightBumper().onFalse(new ElevatorReset(elevator));

                // d-pad
                operator.povDown().whileTrue(new ClimbDown(climber, () -> 1));
                operator.povUp().whileTrue(new Climb(climber, () -> 1));

                operator.start().whileTrue(new ElevateLevel(elevator, ElevateMode.RESET));
        }

        public void getDashboardCommand() {
        }

        public Command getIdleCommands() {
                return new ParallelCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(-77), wrist),
                                // new ElevateLevel(elevator, () -> 5),
                                new ToAngle(() -> Units.degreesToRadians(60), arm));

                // arm 60
                // elevator 0
                // wrist 70
        }

        public Command getAutonomousCommand() {
                // return Commands.print("No autonomous command configured");
                return m_chooser.getSelected();
        }

        public void configureTestCommands() {
                SmartDashboard.putBoolean("is it modified", Intake.modified);
                SmartDashboard.putData("Elevate", new ElevateLevel(elevator, ElevateMode.TEST));
                SmartDashboard.putData("Go down", new ElevateLevel(elevator, ElevateMode.DOWN));

                SmartDashboard.putData("Wrist Up", new ToWristAngle(() -> 70, wrist));
                SmartDashboard.putData("Wrist Down", new ToWristAngle(() -> -35, wrist));
                SmartDashboard.putData("Wrist Neuteral", new ToWristAngle(() -> 0, wrist));

        }

        public void configureAutoCommands() {
                m_chooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("autos", m_chooser);
        }

        public void disabledActions() {
                new SetSolidColor(wpiLights, Color.kMagenta);
                // arm.resetI();
                // arm.runState(new
                // TrapezoidProfile.State(arm.getEncoderPosition().getRadians(), 0));
        }

        public void configureNamedCommands() {
                NamedCommands.registerCommand("L4", new SequentialCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(-35), wrist),
                                new ParallelCommandGroup(
                                                new ToAngle(() -> Units.degreesToRadians(87), arm),
                                                new ElevateLevel(elevator, ElevateMode.L4))));

                NamedCommands.registerCommand("outtake", new IntakeOut(intake).withTimeout(0.5));

                NamedCommands.registerCommand("hpIntake", new ParallelCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(-77), wrist),
                                new ToAngle(() -> Units.degreesToRadians(35), arm),
                                new ElevateLevel(elevator, ElevateMode.L2),
                                new IntakeIn(intake)).withTimeout(0.7));

                NamedCommands.registerCommand("reset", new SequentialCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(-61.5), wrist),
                                new ParallelCommandGroup(
                                                new ToAngle(() -> Units.degreesToRadians(70), arm),
                                                new ElevateLevel(elevator, ElevateMode.L2).withTimeout(0.5))));

        }
}
