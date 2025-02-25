package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class ActualIntake extends Command {
    protected final Intake m_intake;

  public ActualIntake(Intake intake) {
    m_intake = intake;
    // addRequirements(intake);
  }

  @Override
  public void initialize() {
    m_intake.setActualSpeed(-0.15);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntake();
  }
}
