package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class IntakeOut extends Command {
    protected final Intake m_intake;

  public IntakeOut(Intake intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    m_intake.setSpeed(-0.5);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }
}
