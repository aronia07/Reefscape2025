package frc.robot.commands.Intake;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake.Intake;

public class Modify {
  private static final RobotContainer robotContainer = new RobotContainer();
  // private BooleanSupplier booleanSupplier;
  // public static boolean modifiedState;
  // private void modifyTrue() {
  //     modified = true;
  // }
  // private void modifyFalse() {
  //     modified = false;
  // }
  
  // public Modify(RobotContainer robotContainer, Intake intake) {
  //   this.robotContainer = robotContainer;
  //   m_intake = intake;
  //   // this.booleanSupplier = booleanSupplier;
  // }
  public static Command switchOuttake(boolean modification) {
    robotContainer.isModified = modification;
    return Commands.none();
    }

  // @Override
  // public void initialize() {
  //   // boolean modifiedState = booleanSupplier.getAsBoolean();
  //   // if(modifiedState){
  //   //   Intake.modified = modifiedState;
  //   // } else {
  //   //   Intake.modified = modifiedState;
  //   // }
  // }

  // @Override
  // public void end(boolean interrupted) {

  // }
}
