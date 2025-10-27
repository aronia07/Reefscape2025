package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants.ArmWantedMode;
import frc.robot.subsystems.Arm.Arm;


public class ArmCommand extends Command {
    private Arm arm;
    private ArmWantedMode desiredMode;

    public ArmCommand(Arm m_arm, ArmWantedMode m_desiredMode) {
        this.arm = m_arm;
        this.desiredMode = m_desiredMode;
    }

    @Override
    public void execute() {
        arm.setWantedArmMode(this.desiredMode);
    }

    @Override
    public void end(boolean interrupted){
        
    }
}
