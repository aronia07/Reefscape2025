package frc.robot.commands.Intake;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class Modify extends Command {
  protected final Intake m_intake;
  private BooleanSupplier booleanSupplier;
  public static boolean modified;
  // private void modifyTrue() {
  //     modified = true;
  // }
  // private void modifyFalse() {
  //     modified = false;
  // }
  private void modifyIT(boolean state) {
    modified = state;
  }
  
  public Modify(Intake intake, BooleanSupplier booleanSupplier) {
    m_intake = intake;
    this.booleanSupplier = booleanSupplier;
  }

  @Override
  public void initialize() {
    boolean modificationState = booleanSupplier.getAsBoolean();
    modifyIT(modificationState);
  }

  @Override
  public void end(boolean interrupted) {

  }
}
