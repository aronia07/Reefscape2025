package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class IntakeOutL1 extends Command {
  protected final Intake m_intake;

  public IntakeOutL1(Intake intake) {
    m_intake = intake;
    // addRequirements(intake);
  }

  @Override
  public void execute() {
    m_intake.outTake(0.2);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }
}
