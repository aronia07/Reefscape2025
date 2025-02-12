package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.Arm;

public class ManualArm extends Command {

  private final DoubleSupplier m_angleSupplier; //Ensures the arm angle is recorded as a double
  private final Arm m_arm; // The arm subsystem this command will control

  public ManualArm(DoubleSupplier angleSupplier, Arm arm) {
    
    m_angleSupplier = angleSupplier; 
    m_arm = arm; 

    this.addRequirements(m_arm); //Add a dependency to prevent it from running simulataneously wth other arm commands
  }

  @Override
  public void execute() {
    
    //Applies a deadband to the Arm's y value.
    //Makes any armY values between -0.3 and 0.3 equal to 0
    double armY = MathUtil.applyDeadband(this.m_angleSupplier.getAsDouble(), 0.3);
    // double armY = this.m_angleSupplier.getAsDouble();
    
    
    double armChange = (50 / 50) * armY;

    // Update the arm's setpoint to a new angle by adding the change to the current setpoint
    this.m_arm.runSetpoint(new Rotation2d(
      Units.degreesToRadians(armChange + m_arm.getSetpoint().getDegrees())
    ));
  }
}
