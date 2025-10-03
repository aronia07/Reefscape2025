package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.WantedMode;

public class IntakeCommand extends Command {
    private Intake intake;
    private WantedMode desiredMode;

    public IntakeCommand(Intake m_intake, WantedMode m_desiredMode) {
        this.intake = m_intake;
        this.desiredMode = m_desiredMode;
    }

    @Override
    public void initialize() {
        intake.setWantedIntakeMode(this.desiredMode);
    }
}
