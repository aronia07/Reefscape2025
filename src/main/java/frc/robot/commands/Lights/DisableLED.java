package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem;

public class DisableLED extends Command {
    private final LEDSubsystem ledSubsystem;

    public DisableLED(LEDSubsystem subsystem) {
        this.ledSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_Disable();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}