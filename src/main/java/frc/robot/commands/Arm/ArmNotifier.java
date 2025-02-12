package frc.robot.commands.Arm;

import javax.naming.InsufficientResourcesException;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;

public class ArmNotifier extends Command {
  // The arm subsystem this command will control
  private Arm arm;

  //Constructor that takes in the arm subsystem and sets dependencies 
  public ArmNotifier(Arm arm) {
    this.arm = arm;
  }

  @Override
  //Returns true if arm is at goal
  public boolean isFinished() {
    return arm.atGoal();
  }
}

