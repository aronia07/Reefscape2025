package frc.robot.commands.Wrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist.Wrist;

public class ToWristAngle extends Command {


  private Timer m_timer = new Timer(); //Add timer

  private final DoubleSupplier m_angleSupplier; //Ensures the arm's angle is recorded as a double
  protected final Wrist wrist_y; // The arm subsystem this command will control

  private State initialState; //The arm's initial state

  //Trapezoid Motion Profiler
  private TrapezoidProfile m_profiler = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
          Constants.WristConstants.maxVelocityPerSecond.getRadians(),
          Constants.WristConstants.maxAcceleration.getRadians()));

  public ToWristAngle(DoubleSupplier targetAngle, Wrist wrist) {
    this.m_angleSupplier = targetAngle; 
    this.wrist_y = wrist;//wrist subsystem
    ///Add a dependency to prevent it from running simulataneously wth other arm commands
  }

  @Override
  public void initialize() {
    m_timer.reset(); //Reset Timer
    m_timer.start(); //Start timer
    initialState = wrist_y.getCurrenState(); //Get the arm's current state
    wrist_y.setGoal(Rotation2d.fromRadians(m_angleSupplier.getAsDouble())); //Set the arm goal to the angleSupplier
  }

  @Override
  public void execute() {

    //Calculates the next state
    var nextState = m_profiler.calculate(m_timer.get(),
        initialState,
        new TrapezoidProfile.State(m_angleSupplier.getAsDouble(), 0));

    //Sets the arn angle goal
    wrist_y.setGoal(Rotation2d.fromRadians(m_angleSupplier.getAsDouble()));

    //Runs the next state
    wrist_y.runState(nextState);
  }

  @Override
  //Returns true if the profiler has reached its goal
  public boolean isFinished() {
    // return m_profiler.isFinished(m_timer.get()) && wrist_y.atSetpoint();
    return m_profiler.isFinished(m_timer.get());
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop(); //Stops timer
    wrist_y.setGoal(Rotation2d.fromRadians(Units.degreesToRadians(70)));
  }

}

     

