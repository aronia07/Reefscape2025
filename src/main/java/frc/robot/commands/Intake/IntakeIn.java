package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class IntakeIn extends Command {
    protected final Intake m_intake;

  public IntakeIn(Intake intake) {
    m_intake = intake;
    // addRequirements(intake);
  }

  @Override
  public void execute() {
    m_intake.hpIntake(-0.4);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }
}
