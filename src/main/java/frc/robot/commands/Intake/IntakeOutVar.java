package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class IntakeOutVar extends Command {
  protected final Intake m_intake;
  private DoubleSupplier doubleSupplier;

  public IntakeOutVar(Intake intake, DoubleSupplier powerSupplier) {
    m_intake = intake;
    this.doubleSupplier = powerSupplier;
    // addRequirements(intake);
  }

  @Override
  public void execute() {
    double power = this.doubleSupplier.getAsDouble();
    m_intake.outTake(power);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }
}
