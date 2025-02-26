// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Arm.ManualArm;
import frc.robot.commands.Arm.ToAngle;
import frc.robot.commands.Elevator.ElevateLevel;
import frc.robot.commands.Elevator.ElevateManual;
import frc.robot.commands.Elevator.ElevateTest;
import frc.robot.commands.Intake.ActualIntake;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Wrist.WristMove;
import frc.robot.commands.Intake.IntakeOut;
import frc.robot.commands.Wrist.WristMove;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.TunerConstants;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Intake.Intake;
//import frc.robot.subsystems.Wrist.Wrist;
//import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Arm.Arm;

public class RobotContainer {
    /* Subsystems */
    final Arm arm = new Arm();
    final Elevator elevator = new Elevator();
    final Intake intake = new Intake();
    final Wrist wrist = new Wrist();

    public RobotContainer() {
        configureBindings();
        configureTestCommands();
        // getDashboardCommand();
    }

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */ // TODO: move to a drive command
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // .withSteerRequestType(SteerRequestType.Position);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private void configureBindings() {
        // Nperte that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        /* DRIVER CONTROLS */
        // drive with joysticks
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                 // negative Y (forward)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                  // negative X (left)
                ));
        // buttons
        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // CHANGED: changed from back/start+x/y to pov buttons
        // Note that each routine should be run exactly once in a single log.
        // d-pad
        driver.povUp().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.povDown().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.povLeft().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.povRight().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // drivetrain.registerTelemetry(logger::telemeterize);

        /* OPERATOR CONTROLS */
        // joysticks
        arm.setDefaultCommand(new ManualArm(() -> operator.getLeftY(), arm));
        //elevator.setDefaultCommand(new ElevateManual(() -> operator.getRightY(), elevator));
        wrist.setDefaultCommand(new WristMove(() -> operator.getRightY(), wrist));

        
        // operator.leftTrigger().whileTrue(new WristMove(() ->
        // operator.getLeftTriggerAxis(), wrist));
        // operator.rightTrigger().whileTrue(new WristMove(() ->
        // -operator.getRightTriggerAxis(), wrist));

        // buttons
        operator.rightBumper().whileTrue(new IntakeIn(intake));
        operator.leftBumper().whileTrue(new IntakeOut(intake));

        operator.b().whileTrue(new ActualIntake(intake));
        
        operator.y().whileTrue(new ElevateLevel(elevator, () -> 7));
        operator.a().whileTrue(new ElevateLevel(elevator, () -> 8));

        // operator.x().onTrue(new SequentialCommandGroup(new ToAngle()
        // Units.degreesToRadians(40),
        // ));

        // d-pad

    }

    public void getDashboardCommand() {

    }

    public void getIdleCommands() {
        // new ToAngle(() -> 0, arm);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
        // return m_chooser.getSelected();
    }

    public void configureTestCommands() {
        SmartDashboard.putData("Elevator test", new ElevateTest(elevator));
        SmartDashboard.putData("Elevate", new ElevateLevel(elevator, () -> 7));
        SmartDashboard.putData("Go down", new ElevateLevel(elevator, () -> 5));

        // SmartDashboard.putData("Arm up", new ToAngle(() ->
        // Units.degreesToRadians(90), arm));
        // SmartDashboard.putData("Arm down", new ToAngle(() ->
        // Units.degreesToRadians(37), arm));
    }

    public void disabledActions() {
        // arm.resetI();
        // arm.runState(new
        // TrapezoidProfile.State(arm.getEncoderPosition().getRadians(), 0));
    }
}
