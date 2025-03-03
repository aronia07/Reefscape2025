package frc.robot.commands.Lights.LuminLabs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem_LuminLabs;

public class ResetLED_LuminLabs extends Command {
    private final LEDSubsystem_LuminLabs ledSubsystem;

    public ResetLED_LuminLabs(LEDSubsystem_LuminLabs subsystem) {
        this.ledSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_Reset();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
