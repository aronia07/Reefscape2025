// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Timer gcTimer = new Timer();
  private final RobotContainer m_robotContainer;
  private boolean runningAuton = false;

  public Robot() {
    gcTimer.start();
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    PortForwarder.add(5800, "photonvision.local", 5800);
    PathfindingCommand.warmupCommand().schedule();

  }

  @Override
  public void robotPeriodic() {
    if (gcTimer.advanceIfElapsed(5)) {
      System.gc();
    }
    CommandScheduler.getInstance().run();

    // Use vision measurements
    m_robotContainer.drivetrain.updateVisionMeasurements();

  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    if (!runningAuton) {
      m_robotContainer.disabledActions();
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    runningAuton = true;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    runningAuton = false;

    m_robotContainer.getIdleCommands();
  }

  @Override
  public void teleopPeriodic() {
    // if (m_robotContainer.operator.y().getAsBoolean() ||
    // m_robotContainer.operator.b().getAsBoolean()) {
    // m_robotContainer.scoringBindings(m_robotContainer.drivetrain.decideScoringMode());

    // }
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
