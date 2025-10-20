package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakeWantedMode;
import frc.robot.subsystems.Intake.Intake;


public class IntakeCommand extends Command {
    private Intake intake;
    private IntakeWantedMode desiredMode;

    public IntakeCommand(Intake m_intake, IntakeWantedMode m_desiredMode) {
        this.intake = m_intake;
        this.desiredMode = m_desiredMode;
    }

    @Override
    public void initialize() {
        intake.setWantedIntakeMode(this.desiredMode);
    }
}
